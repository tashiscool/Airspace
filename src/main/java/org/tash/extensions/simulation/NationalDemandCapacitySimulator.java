package org.tash.extensions.simulation;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Random;

public class NationalDemandCapacitySimulator {
    private static final Map<String, double[]> AIRPORT_COORDS = Map.ofEntries(
            Map.entry("KJFK", new double[]{40.6413, -73.7781}),
            Map.entry("KEWR", new double[]{40.6895, -74.1745}),
            Map.entry("KLGA", new double[]{40.7769, -73.8740}),
            Map.entry("KBOS", new double[]{42.3656, -71.0096}),
            Map.entry("KIAD", new double[]{38.9531, -77.4565}),
            Map.entry("KATL", new double[]{33.6407, -84.4277}),
            Map.entry("KORD", new double[]{41.9742, -87.9073}),
            Map.entry("KDFW", new double[]{32.8998, -97.0403}),
            Map.entry("KDEN", new double[]{39.8561, -104.6737}),
            Map.entry("KLAX", new double[]{33.9416, -118.4085}),
            Map.entry("KSFO", new double[]{37.6213, -122.3790}),
            Map.entry("KSEA", new double[]{47.4502, -122.3088}),
            Map.entry("KMIA", new double[]{25.7959, -80.2870}),
            Map.entry("KPHX", new double[]{33.4342, -112.0116}),
            Map.entry("KMSP", new double[]{44.8848, -93.2223}),
            Map.entry("KDTW", new double[]{42.2162, -83.3554})
    );
    private static final List<String> DEFAULT_AIRPORTS = List.of("KJFK", "KEWR", "KLGA", "KBOS", "KIAD", "KATL", "KORD", "KDFW", "KDEN", "KLAX", "KSFO", "KSEA", "KMIA", "KPHX", "KMSP", "KDTW");
    private static final List<String> DEFAULT_SECTORS = List.of("ZNY-N90", "ZNY-HIGH", "ZBW-BOS", "ZDC-IAD", "ZTL-ATL", "ZAU-ORD", "ZKC-MCI", "ZFW-DFW", "ZDV-DEN", "ZLA-LAX", "ZOA-SFO", "ZSE-SEA", "ZMA-MIA", "ZAB-PHX", "ZMP-MSP", "ZOB-DTW", "ZID-HIGH", "ZHU-HOU", "ZLC-SLC", "ZME-MEM", "ZJX-JAX", "ZAN-ANC", "HCF-HNL", "ZSU-SJU");

    public NationalDemandCapacityReport run(NationalDemandCapacityConfig requested) {
        NationalDemandCapacityConfig config = normalize(requested);
        List<String> airports = ids(config.getAirportIds(), DEFAULT_AIRPORTS, config.getAirportCount());
        List<String> sectors = ids(config.getSectorIds(), DEFAULT_SECTORS, config.getSectorCount());
        List<FlightSeed> flights = flights(config, airports, sectors);
        TrafficReplayBundle replay = replayBundle(config, flights, airports, sectors);
        List<NationalDemandCapacitySnapshot> snapshots = snapshots(config, flights, replay);
        return NationalDemandCapacityReport.builder()
                .id("national-demand-" + valueOr(config.getId(), "synthetic"))
                .sourceMode(config.getSourceMode())
                .flightCount(flights.size())
                .airportCount(airports.size())
                .sectorCount(sectors.size())
                .durationMinutes(config.getDurationMinutes())
                .tickIntervalMinutes(config.getTickIntervalMinutes())
                .peakAirportDemandCapacityRatio(snapshots.stream().mapToDouble(NationalDemandCapacitySnapshot::getMaxAirportDemandCapacityRatio).max().orElse(0))
                .peakSectorDemandCapacityRatio(snapshots.stream().mapToDouble(NationalDemandCapacitySnapshot::getMaxSectorDemandCapacityRatio).max().orElse(0))
                .peakOverloadedAirportCount(snapshots.stream().mapToInt(NationalDemandCapacitySnapshot::getOverloadedAirportCount).max().orElse(0))
                .peakOverloadedSectorCount(snapshots.stream().mapToInt(NationalDemandCapacitySnapshot::getOverloadedSectorCount).max().orElse(0))
                .totalTmiRecommendationCount(snapshots.stream().mapToInt(NationalDemandCapacitySnapshot::getGeneratedTmiRecommendationCount).sum())
                .trafficReplay(replay)
                .snapshots(snapshots)
                .assumptions(List.of(
                        "Synthetic national demand/capacity model for local simulation only.",
                        "Airport and sector capacities are representative, not authoritative NAS values.",
                        "Demand spikes and capacity reductions are deterministic from the configured seed.",
                        "Generated TMI recommendations require human review and do not mutate official systems."))
                .diagnostics(List.of("Generated " + flights.size() + " synthetic flights across "
                        + airports.size() + " airport(s) and " + sectors.size() + " sector(s)."))
                .build();
    }

    public NationalDemandCapacitySnapshot snapshotAtMinute(NationalDemandCapacityReport report, int minute) {
        if (report == null || report.getSnapshots() == null || report.getSnapshots().isEmpty()) {
            return null;
        }
        return report.getSnapshots().stream()
                .filter(snapshot -> snapshot.getMinute() <= minute)
                .max(Comparator.comparingInt(NationalDemandCapacitySnapshot::getMinute))
                .orElse(report.getSnapshots().get(0));
    }

    private NationalDemandCapacityConfig normalize(NationalDemandCapacityConfig requested) {
        NationalDemandCapacityConfig safe = requested == null ? NationalDemandCapacityConfig.builder().build() : requested;
        return safe.toBuilder()
                .flightCount(clamp(safe.getFlightCount(), 1, 20000))
                .airportCount(clamp(safe.getAirportCount(), 1, DEFAULT_AIRPORTS.size()))
                .sectorCount(clamp(safe.getSectorCount(), 1, DEFAULT_SECTORS.size()))
                .durationMinutes(clamp(safe.getDurationMinutes(), 15, 1440))
                .tickIntervalMinutes(clamp(safe.getTickIntervalMinutes(), 1, 120))
                .demandSpikeFactor(safe.getDemandSpikeFactor() <= 0 ? 1.0 : safe.getDemandSpikeFactor())
                .capacityReductionFactor(safe.getCapacityReductionFactor() <= 0 ? 1.0 : Math.min(1.0, safe.getCapacityReductionFactor()))
                .sourceMode(valueOr(safe.getSourceMode(), "LOCAL_SYNTHETIC_NAS_SCALE"))
                .build();
    }

    private List<FlightSeed> flights(NationalDemandCapacityConfig config, List<String> airports, List<String> sectors) {
        Random random = new Random(config.getRandomSeed());
        List<FlightSeed> flights = new ArrayList<>();
        int bankStart = Math.max(5, config.getDurationMinutes() / 3);
        int bankEnd = Math.min(config.getDurationMinutes() - 5, bankStart + Math.max(15, config.getDurationMinutes() / 4));
        for (int i = 0; i < config.getFlightCount(); i++) {
            String origin = airports.get(Math.floorMod(i + random.nextInt(airports.size()), airports.size()));
            String destination = airports.get(Math.floorMod(i * 7 + random.nextInt(airports.size()), airports.size()));
            if (origin.equals(destination)) {
                destination = airports.get((airports.indexOf(origin) + 1) % airports.size());
            }
            boolean banked = random.nextDouble() < Math.min(0.85, Math.max(0.0, config.getDemandSpikeFactor() - 0.7));
            int departure = banked
                    ? bankStart + random.nextInt(Math.max(1, bankEnd - bankStart))
                    : random.nextInt(Math.max(1, config.getDurationMinutes()));
            int duration = 40 + random.nextInt(140);
            int arrival = Math.min(config.getDurationMinutes(), departure + duration);
            String sector = sectors.get(Math.floorMod(i * 5 + airports.indexOf(origin), sectors.size()));
            AircraftClass aircraftClass = switch (i % 5) {
                case 0 -> AircraftClass.HEAVY_JET;
                case 1 -> AircraftClass.NARROWBODY;
                case 2 -> AircraftClass.REGIONAL_JET;
                case 3 -> AircraftClass.TURBOPROP;
                default -> AircraftClass.GENERIC;
            };
            flights.add(new FlightSeed("NAS-" + String.format(Locale.US, "%05d", i), "NAS" + i, origin, destination, sector, departure, arrival, aircraftClass));
        }
        return flights;
    }

    private TrafficReplayBundle replayBundle(NationalDemandCapacityConfig config, List<FlightSeed> flights, List<String> airports, List<String> sectors) {
        List<TrafficReplayFlightPlan> plans = new ArrayList<>();
        List<TrafficReplayPosition> positions = new ArrayList<>();
        for (FlightSeed flight : flights) {
            List<List<Double>> route = routeFor(flight.origin(), flight.destination());
            plans.add(TrafficReplayFlightPlan.builder()
                    .flightId(flight.flightId())
                    .callsign(flight.callsign())
                    .aircraftClass(flight.aircraftClass())
                    .operator("NAS_SIM")
                    .origin(flight.origin())
                    .destination(flight.destination())
                    .filedRouteText(flight.origin() + " DCT " + flight.sector() + " DCT " + flight.destination())
                    .filedRoutePoints(route)
                    .scheduledDepartureTime("T+" + flight.departureMinute() + "M")
                    .estimatedDepartureTime("T+" + flight.departureMinute() + "M")
                    .estimatedArrivalTime("T+" + flight.arrivalMinute() + "M")
                    .requestedAltitudeFeet(altitudeFor(flight.aircraftClass()))
                    .requestedAltitudeBlock("FL" + ((altitudeFor(flight.aircraftClass()) - 2000) / 100) + "-FL" + ((altitudeFor(flight.aircraftClass()) + 2000) / 100))
                    .filedSpeedKnots(speedFor(flight.aircraftClass()))
                    .sourceRefs(List.of("NAS_SCALE_FLIGHT:" + flight.flightId()))
                    .build());
            positions.add(positionFor(flight, route.get(0), flight.departureMinute(), 0.0, "DEPARTURE"));
            positions.add(positionFor(flight, route.get(1), Math.min(flight.arrivalMinute(), flight.departureMinute() + Math.max(1, (flight.arrivalMinute() - flight.departureMinute()) / 2)), 0.5, "ENROUTE"));
            positions.add(positionFor(flight, route.get(2), flight.arrivalMinute(), 1.0, "ARRIVAL"));
        }
        DemandProducts demandProducts = demandProducts(config, flights, airports, sectors);
        return TrafficReplayBundle.builder()
                .id("national-traffic-replay-" + valueOr(config.getId(), "synthetic"))
                .sourceId("national-demand-capacity:" + valueOr(config.getId(), "synthetic"))
                .sourceMode(config.getSourceMode())
                .providerFamily("NATIONAL_DEMAND_CAPACITY_SYNTHETIC")
                .timeBasis("OFFSET_MINUTES")
                .authorizationMode("LOCAL_FIXTURE_ONLY")
                .flightPlans(plans)
                .positions(positions)
                .airportDemand(demandProducts.airports())
                .sectorDemand(demandProducts.sectors())
                .trafficManagementInitiatives(demandProducts.tmis())
                .assumptions(List.of("Synthetic NAS-scale replay generated from demand/capacity config.", "No live SWIM or official TFMS/FMDS feed was used."))
                .diagnostics(List.of("Synthetic national traffic replay contains " + flights.size() + " flight plan(s)."))
                .build();
    }

    private List<NationalDemandCapacitySnapshot> snapshots(NationalDemandCapacityConfig config, List<FlightSeed> flights, TrafficReplayBundle replay) {
        TrafficReplayAdapter adapter = new TrafficReplayAdapter();
        List<NationalDemandCapacitySnapshot> snapshots = new ArrayList<>();
        for (int minute = 0; minute <= config.getDurationMinutes(); minute += config.getTickIntervalMinutes()) {
            final int currentMinute = minute;
            int active = (int) flights.stream().filter(flight -> flight.departureMinute() <= currentMinute && flight.arrivalMinute() >= currentMinute).count();
            List<TrafficReplayAirportDemand> airportDemand = replay.getAirportDemand().stream().filter(item -> item.getOffsetMinutes() == currentMinute).toList();
            List<TrafficReplaySectorDemand> sectorDemand = replay.getSectorDemand().stream().filter(item -> item.getOffsetMinutes() == currentMinute).toList();
            double maxAirportRatio = 0.0;
            String busiestAirport = null;
            int overloadedAirports = 0;
            int totalDepDemand = 0;
            int totalArrDemand = 0;
            int totalDepCapacity = 0;
            int totalArrCapacity = 0;
            int totalDelay = 0;
            for (TrafficReplayAirportDemand airport : airportDemand) {
                totalDepDemand += airport.getDepartureDemandPerHour();
                totalArrDemand += airport.getArrivalDemandPerHour();
                totalDepCapacity += airport.getDepartureCapacityPerHour();
                totalArrCapacity += airport.getArrivalCapacityPerHour();
                totalDelay += airport.getAverageDelaySeconds() / 60;
                double ratio = ratio(Math.max(airport.getDepartureDemandPerHour(), airport.getArrivalDemandPerHour()),
                        Math.max(1, Math.min(positiveOr(airport.getDepartureCapacityPerHour(), 1), positiveOr(airport.getArrivalCapacityPerHour(), 1))));
                if (ratio > 1.0) overloadedAirports++;
                if (ratio > maxAirportRatio) {
                    maxAirportRatio = ratio;
                    busiestAirport = airport.getAirportId();
                }
            }
            double maxSectorRatio = 0.0;
            String busiestSector = null;
            int overloadedSectors = 0;
            for (TrafficReplaySectorDemand sector : sectorDemand) {
                double ratio = ratio(sector.getActiveAircraft(), sector.getBaselineCapacity());
                if (ratio > 1.0) overloadedSectors++;
                if (ratio > maxSectorRatio) {
                    maxSectorRatio = ratio;
                    busiestSector = sector.getSectorId();
                }
            }
            List<TmiRecommendationModel> recommendations = new ArrayList<>();
            for (TrafficReplayAirportDemand airport : airportDemand) {
                recommendations.addAll(adapter.recommendationsAtMinute(replay, minute, airport, null, "MONITOR"));
            }
            for (TrafficReplaySectorDemand sector : sectorDemand) {
                recommendations.addAll(adapter.recommendationsAtMinute(replay, minute, null, sector, "MONITOR"));
            }
            recommendations = recommendations.stream().distinct().limit(25).toList();
            snapshots.add(NationalDemandCapacitySnapshot.builder()
                    .minute(minute)
                    .totalFlightCount(flights.size())
                    .activeFlightCount(active)
                    .airportCount(airportDemand.size())
                    .sectorCount(sectorDemand.size())
                    .totalAirportDepartureDemandPerHour(totalDepDemand)
                    .totalAirportArrivalDemandPerHour(totalArrDemand)
                    .totalAirportDepartureCapacityPerHour(totalDepCapacity)
                    .totalAirportArrivalCapacityPerHour(totalArrCapacity)
                    .overloadedAirportCount(overloadedAirports)
                    .overloadedSectorCount(overloadedSectors)
                    .maxAirportDemandCapacityRatio(maxAirportRatio)
                    .maxSectorDemandCapacityRatio(maxSectorRatio)
                    .busiestAirportId(busiestAirport)
                    .busiestSectorId(busiestSector)
                    .totalExpectedDelayMinutes(totalDelay)
                    .generatedTmiRecommendationCount(recommendations.size())
                    .recommendations(recommendations)
                    .diagnostics(List.of("Snapshot aggregates local synthetic demand/capacity at T+" + minute + "M."))
                    .build());
        }
        return snapshots;
    }

    private DemandProducts demandProducts(NationalDemandCapacityConfig config, List<FlightSeed> flights, List<String> airports, List<String> sectors) {
        List<TrafficReplayAirportDemand> airportDemand = new ArrayList<>();
        List<TrafficReplaySectorDemand> sectorDemand = new ArrayList<>();
        List<TrafficManagementInitiative> tmis = new ArrayList<>();
        for (int minute = 0; minute <= config.getDurationMinutes(); minute += config.getTickIntervalMinutes()) {
            final int currentMinute = minute;
            boolean reduced = config.isIncludeWeatherCapacityReduction() && minute >= config.getDurationMinutes() / 3 && minute <= (config.getDurationMinutes() * 2) / 3;
            for (String airport : airports) {
                int depFlights = countDepartures(flights, airport, minute, config.getTickIntervalMinutes());
                int arrFlights = countArrivals(flights, airport, minute, config.getTickIntervalMinutes());
                int baseCapacity = airportBaseCapacity(airport);
                int capacity = (int) Math.max(4, Math.round(baseCapacity * (reduced && isHub(airport) ? config.getCapacityReductionFactor() : 1.0)));
                int depDemandRate = rate(depFlights, config.getTickIntervalMinutes());
                int arrDemandRate = rate(arrFlights, config.getTickIntervalMinutes());
                int queue = Math.max(0, Math.max(depDemandRate - capacity, arrDemandRate - capacity) / 4);
                int delay = Math.max(0, queue * 90);
                airportDemand.add(TrafficReplayAirportDemand.builder()
                        .airportId(airport)
                        .timestamp("T+" + minute + "M")
                        .offsetMinutes(minute)
                        .departureDemandPerHour(depDemandRate)
                        .arrivalDemandPerHour(arrDemandRate)
                        .departureCapacityPerHour(capacity)
                        .arrivalCapacityPerHour(capacity)
                        .departureQueueDepth(queue)
                        .arrivalQueueDepth(Math.max(0, arrDemandRate - capacity) / 4)
                        .averageDelaySeconds(delay)
                        .runwayConfiguration("SIM_CONFIG")
                        .sourceRefs(List.of("NATIONAL_AIRPORT_DEMAND:" + airport + ":T+" + minute))
                        .build());
                if (queue > 0) {
                    TrafficManagementInitiativeType type = arrDemandRate > capacity ? TrafficManagementInitiativeType.GDP : TrafficManagementInitiativeType.DEPARTURE_METERING;
                    tmis.add(TrafficManagementInitiative.builder()
                            .id("TMI-" + type.name() + "-" + airport + "-" + minute)
                            .type(type.name())
                            .primitiveType(type)
                            .scope("AIRPORT")
                            .targetResourceId(airport)
                            .reason("Synthetic airport demand exceeds capacity.")
                            .startOffsetMinutes(minute)
                            .endOffsetMinutes(Math.min(config.getDurationMinutes(), minute + config.getTickIntervalMinutes()))
                            .expectedDelayMinutes(delay / 60)
                            .confidence(0.74)
                            .flowProgram(type == TrafficManagementInitiativeType.GDP ? TrafficFlowProgramModel.builder()
                                    .programId("GDP-" + airport + "-" + minute)
                                    .programType(type)
                                    .targetAirport(airport)
                                    .arrivalRatePerHour(capacity)
                                    .departureRatePerHour(capacity)
                                    .edctWindowMinutes(5)
                                    .delayAssignmentPolicy("SYNTHETIC_PROPORTIONAL")
                                    .sourceRefs(List.of("NATIONAL_AIRPORT_DEMAND:" + airport + ":T+" + minute))
                                    .build() : null)
                            .departureMetering(type == TrafficManagementInitiativeType.DEPARTURE_METERING ? DepartureMeteringModel.builder()
                                    .meteringId("DM-" + airport + "-" + minute)
                                    .commonPoint(airport)
                                    .intervalSeconds(Math.max(60, (int) Math.round(3600.0 / Math.max(1, capacity))))
                                    .releaseRatePerHour(capacity)
                                    .departureAirports(List.of(airport))
                                    .sourceRefs(List.of("NATIONAL_AIRPORT_DEMAND:" + airport + ":T+" + minute))
                                    .build() : null)
                            .sourceRefs(List.of("NATIONAL_TMI:" + airport + ":T+" + minute))
                            .build());
                }
            }
            for (String sector : sectors) {
                int active = (int) flights.stream().filter(flight -> flight.sector().equals(sector) && flight.departureMinute() <= currentMinute && flight.arrivalMinute() >= currentMinute).count();
                int capacity = (int) Math.max(6, Math.round(sectorBaseCapacity(sector) * (reduced && isCoreSector(sector) ? config.getCapacityReductionFactor() : 1.0)));
                int queue = Math.max(0, active - capacity);
                double utilization = ratio(active, capacity);
                int delay = (int) Math.round(Math.max(0, utilization - 0.85) * 240);
                sectorDemand.add(TrafficReplaySectorDemand.builder()
                        .sectorId(sector)
                        .timestamp("T+" + minute + "M")
                        .offsetMinutes(minute)
                        .activeAircraft(active)
                        .baselineCapacity(capacity)
                        .handoffQueueDepth(queue)
                        .frequencyUtilization(Math.min(1.0, utilization * 0.75))
                        .estimatedHandoffDelaySeconds(delay)
                        .sourceRefs(List.of("NATIONAL_SECTOR_DEMAND:" + sector + ":T+" + minute))
                        .build());
                if (active > capacity) {
                    tmis.add(TrafficManagementInitiative.builder()
                            .id("TMI-SECTOR-" + sector + "-" + minute)
                            .type("SECTOR_CAPACITY")
                            .primitiveType(TrafficManagementInitiativeType.SECTOR_CAPACITY)
                            .scope("SECTOR")
                            .targetResourceId(sector)
                            .reason("Synthetic sector demand exceeds capacity.")
                            .startOffsetMinutes(minute)
                            .endOffsetMinutes(Math.min(config.getDurationMinutes(), minute + config.getTickIntervalMinutes()))
                            .expectedDelayMinutes(Math.max(1, delay / 60))
                            .confidence(0.76)
                            .sectorCapacity(SectorCapacityModel.builder()
                                    .capacityId("SC-" + sector + "-" + minute)
                                    .sectorId(sector)
                                    .baselineCapacity(sectorBaseCapacity(sector))
                                    .reducedCapacity(capacity)
                                    .demandCount(active)
                                    .capacityState(utilization >= 1.3 ? "SATURATED" : "COMPRESSED")
                                    .reason("Synthetic national demand/capacity imbalance.")
                                    .sourceRefs(List.of("NATIONAL_SECTOR_DEMAND:" + sector + ":T+" + minute))
                                    .build())
                            .sourceRefs(List.of("NATIONAL_TMI:" + sector + ":T+" + minute))
                            .build());
                }
            }
        }
        return new DemandProducts(airportDemand, sectorDemand, tmis);
    }

    private TrafficReplayPosition positionFor(FlightSeed flight, List<Double> point, int minute, double progress, String phase) {
        return TrafficReplayPosition.builder()
                .flightId(flight.flightId())
                .timestamp("T+" + minute + "M")
                .offsetMinutes(minute)
                .latitude(point.get(0))
                .longitude(point.get(1))
                .altitudeFeet(phase.equals("DEPARTURE") ? 0 : altitudeFor(flight.aircraftClass()))
                .groundSpeedKnots(phase.equals("ARRIVAL") ? 180 : speedFor(flight.aircraftClass()))
                .headingDegrees(80)
                .routeProgress(progress)
                .phase(phase)
                .sourceRefs(List.of("NATIONAL_TRACK:" + flight.flightId() + ":T+" + minute))
                .build();
    }

    private List<List<Double>> routeFor(String origin, String destination) {
        double[] o = AIRPORT_COORDS.getOrDefault(origin, new double[]{39.0, -98.0});
        double[] d = AIRPORT_COORDS.getOrDefault(destination, new double[]{40.0, -97.0});
        return List.of(
                List.of(o[0], o[1], 0.0),
                List.of((o[0] + d[0]) / 2.0, (o[1] + d[1]) / 2.0, 32000.0),
                List.of(d[0], d[1], 0.0)
        );
    }

    private List<String> ids(List<String> requested, List<String> defaults, int count) {
        if (requested != null && !requested.isEmpty()) {
            return requested.stream().limit(count).toList();
        }
        return defaults.stream().limit(count).toList();
    }

    private int countDepartures(List<FlightSeed> flights, String airport, int minute, int windowMinutes) {
        return (int) flights.stream()
                .filter(flight -> flight.origin().equals(airport))
                .filter(flight -> flight.departureMinute() >= minute && flight.departureMinute() < minute + windowMinutes)
                .count();
    }

    private int countArrivals(List<FlightSeed> flights, String airport, int minute, int windowMinutes) {
        return (int) flights.stream()
                .filter(flight -> flight.destination().equals(airport))
                .filter(flight -> flight.arrivalMinute() >= minute && flight.arrivalMinute() < minute + windowMinutes)
                .count();
    }

    private int rate(int count, int windowMinutes) {
        return (int) Math.round(count * 60.0 / Math.max(1, windowMinutes));
    }

    private int airportBaseCapacity(String airport) {
        return isHub(airport) ? 60 : 38;
    }

    private int sectorBaseCapacity(String sector) {
        return isCoreSector(sector) ? 42 : 30;
    }

    private boolean isHub(String airport) {
        return List.of("KJFK", "KEWR", "KLGA", "KATL", "KORD", "KDFW", "KLAX").contains(airport);
    }

    private boolean isCoreSector(String sector) {
        return sector.contains("ZNY") || sector.contains("ZAU") || sector.contains("ZDC") || sector.contains("ZLA") || sector.contains("ZTL");
    }

    private int altitudeFor(AircraftClass aircraftClass) {
        return switch (aircraftClass) {
            case HEAVY_JET -> 37000;
            case NARROWBODY -> 35000;
            case REGIONAL_JET -> 31000;
            case TURBOPROP -> 18000;
            default -> 33000;
        };
    }

    private int speedFor(AircraftClass aircraftClass) {
        return switch (aircraftClass) {
            case HEAVY_JET -> 470;
            case NARROWBODY -> 450;
            case REGIONAL_JET -> 410;
            case TURBOPROP -> 285;
            default -> 420;
        };
    }

    private double ratio(int demand, int capacity) {
        return capacity <= 0 ? 0.0 : demand / (double) capacity;
    }

    private int positiveOr(int value, int fallback) {
        return value > 0 ? value : fallback;
    }

    private int clamp(int value, int min, int max) {
        return Math.max(min, Math.min(max, value));
    }

    private String valueOr(String value, String fallback) {
        return value == null || value.isBlank() ? fallback : value;
    }

    private record FlightSeed(String flightId, String callsign, String origin, String destination, String sector, int departureMinute, int arrivalMinute, AircraftClass aircraftClass) {
    }

    private record DemandProducts(List<TrafficReplayAirportDemand> airports, List<TrafficReplaySectorDemand> sectors, List<TrafficManagementInitiative> tmis) {
    }
}
