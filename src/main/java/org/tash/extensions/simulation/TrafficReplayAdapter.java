package org.tash.extensions.simulation;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import java.util.Optional;

public class TrafficReplayAdapter {
    private final ObjectMapper objectMapper = new ObjectMapper().findAndRegisterModules();

    public TrafficReplayBundle parseJson(String rawJson) {
        try {
            return objectMapper.readValue(rawJson, TrafficReplayBundle.class);
        } catch (JsonProcessingException e) {
            throw new IllegalArgumentException("Malformed traffic replay JSON: " + e.getMessage(), e);
        }
    }

    public String toJson(TrafficReplayBundle bundle) {
        try {
            return objectMapper.writerWithDefaultPrettyPrinter().writeValueAsString(bundle);
        } catch (JsonProcessingException e) {
            throw new IllegalArgumentException("Unable to serialize traffic replay bundle: " + e.getMessage(), e);
        }
    }

    public List<String> validate(TrafficReplayBundle bundle) {
        List<String> diagnostics = new ArrayList<>();
        if (bundle == null) {
            diagnostics.add("Traffic replay bundle is missing.");
            return diagnostics;
        }
        if (blank(bundle.getId())) diagnostics.add("Traffic replay id is missing.");
        if (blank(bundle.getSourceMode())) diagnostics.add("Traffic replay sourceMode is missing; LOCAL_FIXTURE_REPLAY should be explicit for local data.");
        if (bundle.getFlightPlans() == null || bundle.getFlightPlans().isEmpty()) diagnostics.add("Traffic replay has no flight plans.");
        if (bundle.getPositions() == null || bundle.getPositions().isEmpty()) diagnostics.add("Traffic replay has no aircraft positions.");
        if (bundle.getAirportDemand() == null || bundle.getAirportDemand().isEmpty()) diagnostics.add("Traffic replay has no airport demand snapshots.");
        if (bundle.getSectorDemand() == null || bundle.getSectorDemand().isEmpty()) diagnostics.add("Traffic replay has no sector demand snapshots.");
        if (bundle.getTrafficManagementInitiatives() != null) {
            for (TrafficManagementInitiative tmi : bundle.getTrafficManagementInitiatives()) {
                if (canonicalTmiType(tmi) == TrafficManagementInitiativeType.UNKNOWN) {
                    diagnostics.add("Traffic replay TMI " + valueOr(tmi.getId(), "unknown") + " has unknown type " + valueOr(tmi.getType(), "UNKNOWN") + ".");
                }
            }
        }
        return diagnostics;
    }

    public TrafficFlowScenario toTrafficFlowScenario(TrafficReplayBundle bundle) {
        TrafficReplayBundle safe = bundle == null ? emptyBundle() : bundle;
        return TrafficFlowScenario.builder()
                .id("traffic-flow-" + valueOr(safe.getId(), "local-replay"))
                .sourceMode(valueOr(safe.getSourceMode(), "LOCAL_FIXTURE_REPLAY"))
                .aircraft(aircraftAtMinute(safe, 0, refsForBundle(safe)))
                .assumptions("Generated from recorded SWIM/TFMS-like replay bundle; transport/authentication is outside this local adapter.")
                .build();
    }

    public List<SimulatedAircraft> aircraftAtMinute(TrafficReplayBundle bundle, int minute, List<String> inheritedSourceRefs) {
        if (bundle == null || bundle.getFlightPlans() == null || bundle.getFlightPlans().isEmpty()) {
            return List.of();
        }
        List<SimulatedAircraft> aircraft = new ArrayList<>();
        for (TrafficReplayFlightPlan plan : bundle.getFlightPlans()) {
            TrafficReplayPosition position = positionAtMinute(bundle, plan.getFlightId(), minute).orElse(null);
            if (position == null) {
                continue;
            }
            List<String> sourceRefs = mergedRefs(inheritedSourceRefs, plan.getSourceRefs(), position.getSourceRefs(), refsForBundle(bundle));
            AircraftClass aircraftClass = plan.getAircraftClass() == null ? AircraftClass.GENERIC : plan.getAircraftClass();
            aircraft.add(SimulatedAircraft.builder()
                    .id(valueOr(plan.getFlightId(), "traffic-" + aircraft.size()))
                    .callsign(valueOr(plan.getCallsign(), plan.getFlightId()))
                    .aircraftClass(aircraftClass)
                    .performanceProfile(profileFor(aircraftClass))
                    .flightPlan(FlightPlanIntent.builder()
                            .origin(plan.getOrigin())
                            .destination(plan.getDestination())
                            .requestedAltitudeBlock(altitudeBlock(plan))
                            .route(plan.getFiledRoutePoints() == null ? List.of() : plan.getFiledRoutePoints())
                            .missionId(plan.getMissionId())
                            .reservationId(plan.getReservationId())
                            .rationale("Recorded traffic replay flight plan " + valueOr(plan.getFlightId(), "UNKNOWN"))
                            .build())
                    .trajectory(AircraftTrajectoryState.builder()
                            .latitude(position.getLatitude())
                            .longitude(position.getLongitude())
                            .altitudeFeet(position.getAltitudeFeet())
                            .groundSpeedKnots(position.getGroundSpeedKnots())
                            .routeProgress(clampedProgress(position))
                            .delaySeconds(Math.max(0.0, position.getDelaySeconds()))
                            .phase(valueOr(position.getPhase(), phaseFromSpeed(position.getGroundSpeedKnots())))
                            .build())
                    .rerouteAssignment(assignmentForActiveTmis(activeTmisForFlight(bundle, plan.getFlightId(), minute)))
                    .impacted(!sourceRefs.isEmpty() || !activeTmisForFlight(bundle, plan.getFlightId(), minute).isEmpty())
                    .impactedSourceRefs(sourceRefs)
                    .build());
        }
        return aircraft;
    }

    public Optional<TrafficReplayPosition> positionAtMinute(TrafficReplayBundle bundle, String flightId, int minute) {
        if (bundle == null || bundle.getPositions() == null || blank(flightId)) {
            return Optional.empty();
        }
        List<TrafficReplayPosition> matching = bundle.getPositions().stream()
                .filter(position -> flightId.equals(position.getFlightId()))
                .sorted(Comparator.comparingInt(TrafficReplayPosition::getOffsetMinutes))
                .toList();
        if (matching.isEmpty()) {
            return Optional.empty();
        }
        return matching.stream()
                .filter(position -> position.getOffsetMinutes() <= minute)
                .max(Comparator.comparingInt(TrafficReplayPosition::getOffsetMinutes));
    }

    public Optional<TrafficReplayAirportDemand> airportDemandAtMinute(TrafficReplayBundle bundle, int minute) {
        if (bundle == null || bundle.getAirportDemand() == null || bundle.getAirportDemand().isEmpty()) {
            return Optional.empty();
        }
        return latestAtOrBefore(bundle.getAirportDemand(), minute, TrafficReplayAirportDemand::getOffsetMinutes);
    }

    public Optional<TrafficReplaySectorDemand> sectorDemandAtMinute(TrafficReplayBundle bundle, int minute) {
        if (bundle == null || bundle.getSectorDemand() == null || bundle.getSectorDemand().isEmpty()) {
            return Optional.empty();
        }
        return latestAtOrBefore(bundle.getSectorDemand(), minute, TrafficReplaySectorDemand::getOffsetMinutes);
    }

    public List<TrafficManagementInitiative> activeTmisAtMinute(TrafficReplayBundle bundle, int minute) {
        if (bundle == null || bundle.getTrafficManagementInitiatives() == null) {
            return List.of();
        }
        return bundle.getTrafficManagementInitiatives().stream()
                .filter(tmi -> minute >= tmi.getStartOffsetMinutes() && minute <= Math.max(tmi.getStartOffsetMinutes(), tmi.getEndOffsetMinutes()))
                .toList();
    }

    public int activeFlightCountAtMinute(TrafficReplayBundle bundle, int minute) {
        return aircraftAtMinute(bundle, minute, refsForBundle(bundle)).size();
    }

    public List<TmiRecommendationModel> recommendationsAtMinute(TrafficReplayBundle bundle,
                                                                 int minute,
                                                                 TrafficReplayAirportDemand airportDemand,
                                                                 TrafficReplaySectorDemand sectorDemand,
                                                                 String engineAction) {
        List<TmiRecommendationModel> recommendations = new ArrayList<>();
        for (TrafficManagementInitiative tmi : activeTmisAtMinute(bundle, minute)) {
            if (tmi.getRecommendation() != null) {
                recommendations.add(tmi.getRecommendation());
            } else {
                recommendations.add(recommendationForTmi(tmi, minute));
            }
        }
        if (airportDemand != null && airportDemand.getDepartureCapacityPerHour() > 0
                && airportDemand.getDepartureDemandPerHour() > airportDemand.getDepartureCapacityPerHour()) {
            recommendations.add(TmiRecommendationModel.builder()
                    .id("rec-airport-demand-" + airportDemand.getAirportId() + "-" + minute)
                    .recommendedType(TrafficManagementInitiativeType.GDP)
                    .action("EVALUATE_GDP_OR_DEPARTURE_METERING")
                    .targetResourceType("AIRPORT")
                    .targetResourceId(airportDemand.getAirportId())
                    .trigger("Departure demand " + airportDemand.getDepartureDemandPerHour()
                            + "/hr exceeds capacity " + airportDemand.getDepartureCapacityPerHour() + "/hr")
                    .severity(airportDemand.getDepartureDemandPerHour() >= airportDemand.getDepartureCapacityPerHour() * 2 ? "HIGH" : "MEDIUM")
                    .expectedDelayMinutes(Math.max(1, airportDemand.getAverageDelaySeconds() / 60))
                    .confidence(0.76)
                    .rationale("Airport demand/capacity replay indicates ground-side metering should be reviewed.")
                    .sourceRefs(airportDemand.getSourceRefs())
                    .build());
        }
        if (sectorDemand != null && sectorDemand.getBaselineCapacity() > 0
                && sectorDemand.getActiveAircraft() > sectorDemand.getBaselineCapacity()) {
            recommendations.add(TmiRecommendationModel.builder()
                    .id("rec-sector-capacity-" + sectorDemand.getSectorId() + "-" + minute)
                    .recommendedType(TrafficManagementInitiativeType.SECTOR_CAPACITY)
                    .action("EVALUATE_AFP_MIT_OR_REROUTE")
                    .targetResourceType("SECTOR")
                    .targetResourceId(sectorDemand.getSectorId())
                    .trigger("Sector active aircraft " + sectorDemand.getActiveAircraft()
                            + " exceeds baseline capacity " + sectorDemand.getBaselineCapacity())
                    .severity(sectorDemand.getActiveAircraft() >= sectorDemand.getBaselineCapacity() * 1.3 ? "HIGH" : "MEDIUM")
                    .expectedDelayMinutes(Math.max(1, sectorDemand.getEstimatedHandoffDelaySeconds() / 60))
                    .confidence(0.78)
                    .rationale("Sector capacity replay suggests a flow restriction or reroute review before saturation.")
                    .sourceRefs(sectorDemand.getSourceRefs())
                    .build());
        }
        if ("BLOCKED".equalsIgnoreCase(engineAction) || "REROUTE".equalsIgnoreCase(engineAction)) {
            recommendations.add(TmiRecommendationModel.builder()
                    .id("rec-action-" + minute)
                    .recommendedType(TrafficManagementInitiativeType.REROUTE_ADVISORY)
                    .action("DRAFT_REROUTE_ADVISORY")
                    .targetResourceType("ROUTE")
                    .targetResourceId("SIMULATION_ROUTE")
                    .trigger("Engine action " + engineAction + " requires traffic-flow coordination.")
                    .severity("BLOCKED".equalsIgnoreCase(engineAction) ? "HIGH" : "MEDIUM")
                    .expectedDelayMinutes("BLOCKED".equalsIgnoreCase(engineAction) ? 30 : 10)
                    .confidence("BLOCKED".equalsIgnoreCase(engineAction) ? 0.86 : 0.74)
                    .rationale("Route impact guidance should be paired with a human-reviewed reroute advisory.")
                    .sourceRefs(refsForBundle(bundle))
                    .build());
        }
        return recommendations.stream()
                .filter(recommendation -> recommendation != null && recommendation.getRecommendedType() != null)
                .toList();
    }

    public TrafficReplayBundle defaultReplayForScenario(SimulationScenario scenario) {
        List<List<Double>> route = scenario == null || scenario.getRoute() == null || scenario.getRoute().isEmpty()
                ? List.of(List.of(30.0, -150.0, 25000.0), List.of(31.0, -149.0, 26000.0))
                : scenario.getRoute();
        String scenarioId = scenario == null ? "default" : scenario.getId();
        TrafficReplayFlightPlan lead = flightPlan("TRF-" + scenarioId + "-1", "LEAD1", AircraftClass.MILITARY_REFUELING, route, "JFK", "DOV", scenarioId);
        TrafficReplayFlightPlan trail = flightPlan("TRF-" + scenarioId + "-2", "TRAIL2", AircraftClass.NARROWBODY, offsetRoute(route, -0.08), "JFK", "DOV", scenarioId);
        TrafficReplayFlightPlan flow = flightPlan("TRF-" + scenarioId + "-3", "FLOW3", AircraftClass.REGIONAL_JET, offsetRoute(route, 0.08), "JFK", "DOV", scenarioId);
        List<TrafficReplayPosition> positions = new ArrayList<>();
        positions.addAll(positionsFor(lead, 0));
        positions.addAll(positionsFor(trail, 1));
        positions.addAll(positionsFor(flow, 2));
        return TrafficReplayBundle.builder()
                .id("traffic-replay-" + scenarioId)
                .sourceId("simulation-corpus:" + scenarioId)
                .sourceMode("LOCAL_FIXTURE_REPLAY")
                .providerFamily("TFMS_LIKE_RECORDED_REPLAY")
                .timeBasis("OFFSET_MINUTES")
                .authorizationMode("LOCAL_FIXTURE_ONLY")
                .sourceRefs(List.of("TFMS_REPLAY:local-fixture:" + scenarioId))
                .flightPlans(List.of(lead, trail, flow))
                .positions(positions)
                .airportDemand(List.of(
                        airportDemand("KJFK", 0, 20, 22, 24, 24, 1, 45),
                        airportDemand("KJFK", 5, 28, 24, 12, 18, 6, 480),
                        airportDemand("KJFK", 15, 34, 28, 10, 16, 10, 540)))
                .sectorDemand(List.of(
                        sectorDemand("SIM-SECTOR", 0, 12, 20, 0, 0.42, 15),
                        sectorDemand("SIM-SECTOR", 5, 24, 20, 4, 0.78, 90),
                        sectorDemand("SIM-SECTOR", 15, 32, 20, 12, 0.96, 180)))
                .trafficManagementInitiatives(List.of(TrafficManagementInitiative.builder()
                        .id("TMI-" + scenarioId)
                        .type("REROUTE_REVIEW")
                        .primitiveType(TrafficManagementInitiativeType.REROUTE_ADVISORY)
                        .status("PROPOSED")
                        .scope("ROUTE")
                        .targetResourceId("scenario:" + scenarioId)
                        .reason("Local replay TMI generated from scenario route impact.")
                        .constraintId("scenario:" + scenarioId)
                        .startOffsetMinutes(5)
                        .endOffsetMinutes(30)
                        .confidence(0.72)
                        .affectedFlightIds(List.of(lead.getFlightId(), trail.getFlightId()))
                        .rerouteAdvisory(RerouteAdvisoryModel.builder()
                                .advisoryId("RR-" + scenarioId)
                                .advisoryType("LOCAL_REPLAY_REROUTE")
                                .routeName("SIM_DOGLEG")
                                .routeText("Review dogleg around constrained route segment")
                                .routePoints(offsetRoute(route, 0.18))
                                .reason("Local replay route-impact mitigation.")
                                .startOffsetMinutes(5)
                                .endOffsetMinutes(30)
                                .affectedFlightIds(List.of(lead.getFlightId(), trail.getFlightId()))
                                .sourceRefs(List.of("REROUTE_ADVISORY:local-replay:" + scenarioId))
                                .build())
                        .recommendation(TmiRecommendationModel.builder()
                                .id("REC-" + scenarioId)
                                .recommendedType(TrafficManagementInitiativeType.REROUTE_ADVISORY)
                                .action("REVIEW_REROUTE_ADVISORY")
                                .targetResourceType("ROUTE")
                                .targetResourceId("scenario:" + scenarioId)
                                .trigger("Local replay route impact")
                                .severity("MEDIUM")
                                .expectedDelayMinutes(10)
                                .confidence(0.72)
                                .rationale("Default scenario replay proposes a human-reviewed reroute advisory.")
                                .sourceTmiIds(List.of("TMI-" + scenarioId))
                                .affectedFlightIds(List.of(lead.getFlightId(), trail.getFlightId()))
                                .sourceRefs(List.of("TMI:local-replay:" + scenarioId))
                                .build())
                        .sourceRefs(List.of("TMI:local-replay:" + scenarioId))
                        .build()))
                .assumptions(List.of(
                        "Recorded replay shape follows SWIM/TFMS/FMDS concepts but is not live SWIM data.",
                        "Offset-minute timing is used for deterministic local simulation.",
                        "Airport and sector demand are replay snapshots, not authoritative NAS capacity data."))
                .diagnostics(List.of("Fixture-backed traffic replay generated by Airspace simulator."))
                .build();
    }

    private <T> Optional<T> latestAtOrBefore(List<T> items, int minute, OffsetExtractor<T> extractor) {
        return items.stream()
                .filter(item -> extractor.offsetMinutes(item) <= minute)
                .max(Comparator.comparingInt(extractor::offsetMinutes))
                .or(() -> items.stream().min(Comparator.comparingInt(extractor::offsetMinutes)));
    }

    private List<TrafficManagementInitiative> activeTmisForFlight(TrafficReplayBundle bundle, String flightId, int minute) {
        if (blank(flightId)) return List.of();
        return activeTmisAtMinute(bundle, minute).stream()
                .filter(this::isFlightAffectingTmi)
                .filter(tmi -> tmi.getAffectedFlightIds() == null || tmi.getAffectedFlightIds().isEmpty() || tmi.getAffectedFlightIds().contains(flightId))
                .toList();
    }

    public TrafficManagementInitiativeType canonicalTmiType(TrafficManagementInitiative tmi) {
        if (tmi == null) return TrafficManagementInitiativeType.UNKNOWN;
        if (tmi.getPrimitiveType() != null) return tmi.getPrimitiveType();
        return TrafficManagementInitiativeType.fromCode(tmi.getType());
    }

    public String assignmentForActiveTmis(List<TrafficManagementInitiative> activeTmis) {
        if (activeTmis == null || activeTmis.isEmpty()) return "NONE";
        if (activeTmis.stream().map(this::canonicalTmiType).anyMatch(type -> type == TrafficManagementInitiativeType.GROUND_STOP)) return "GROUND_STOP_REVIEW";
        if (activeTmis.stream().map(this::canonicalTmiType).anyMatch(type -> type == TrafficManagementInitiativeType.GDP || type == TrafficManagementInitiativeType.AFP || type == TrafficManagementInitiativeType.DEPARTURE_METERING || type == TrafficManagementInitiativeType.ARRIVAL_RATE)) return "EDCT_OR_METERING_REVIEW";
        if (activeTmis.stream().map(this::canonicalTmiType).anyMatch(type -> type == TrafficManagementInitiativeType.MILES_IN_TRAIL || type == TrafficManagementInitiativeType.MINUTES_IN_TRAIL)) return "SPACING_RESTRICTION_REVIEW";
        if (activeTmis.stream().map(this::canonicalTmiType).anyMatch(type -> type == TrafficManagementInitiativeType.REROUTE_ADVISORY || type == TrafficManagementInitiativeType.REQUIRED_REROUTE)) return "REROUTE_ADVISORY_REVIEW";
        return "TMI_REVIEW";
    }

    private boolean isFlightAffectingTmi(TrafficManagementInitiative tmi) {
        TrafficManagementInitiativeType type = canonicalTmiType(tmi);
        return type != TrafficManagementInitiativeType.FEA && type != TrafficManagementInitiativeType.UNKNOWN;
    }

    private TmiRecommendationModel recommendationForTmi(TrafficManagementInitiative tmi, int minute) {
        TrafficManagementInitiativeType type = canonicalTmiType(tmi);
        String id = valueOr(tmi.getId(), "tmi-" + minute);
        return TmiRecommendationModel.builder()
                .id("rec-" + id)
                .recommendedType(type == TrafficManagementInitiativeType.UNKNOWN ? TrafficManagementInitiativeType.LOCAL_REVIEW : type)
                .action(actionForTmiType(type))
                .targetResourceType(valueOr(tmi.getScope(), "TRAFFIC_FLOW"))
                .targetResourceId(valueOr(tmi.getTargetResourceId(), tmi.getConstraintId()))
                .trigger(valueOr(tmi.getReason(), "Active traffic management initiative at T+" + minute + "M"))
                .severity(severityForTmiType(type))
                .expectedDelayMinutes(Math.max(0, tmi.getExpectedDelayMinutes()))
                .confidence(tmi.getConfidence() <= 0 ? 0.70 : tmi.getConfidence())
                .rationale("Recommendation generated from active replay TMI " + id + ". Human approval remains required.")
                .sourceTmiIds(List.of(id))
                .affectedFlightIds(tmi.getAffectedFlightIds() == null ? List.of() : tmi.getAffectedFlightIds())
                .sourceRefs(tmi.getSourceRefs() == null ? List.of() : tmi.getSourceRefs())
                .build();
    }

    private String actionForTmiType(TrafficManagementInitiativeType type) {
        return switch (type) {
            case GROUND_STOP -> "REVIEW_GROUND_STOP";
            case GDP -> "REVIEW_GROUND_DELAY_PROGRAM";
            case AFP -> "REVIEW_AIRSPACE_FLOW_PROGRAM";
            case FCA -> "REVIEW_FLOW_CONSTRAINED_AREA";
            case FEA -> "MONITOR_FLOW_EVALUATION_AREA";
            case MILES_IN_TRAIL, MINUTES_IN_TRAIL -> "REVIEW_TRAIL_SPACING";
            case DEPARTURE_METERING -> "REVIEW_DEPARTURE_METERING";
            case ARRIVAL_RATE -> "REVIEW_ARRIVAL_RATE";
            case SECTOR_CAPACITY -> "REVIEW_SECTOR_CAPACITY";
            case REROUTE_ADVISORY, REQUIRED_REROUTE -> "REVIEW_REROUTE";
            default -> "REVIEW_TMI";
        };
    }

    private String severityForTmiType(TrafficManagementInitiativeType type) {
        return switch (type) {
            case GROUND_STOP -> "HIGH";
            case GDP, AFP, SECTOR_CAPACITY, REQUIRED_REROUTE -> "MEDIUM_HIGH";
            case MILES_IN_TRAIL, MINUTES_IN_TRAIL, DEPARTURE_METERING, ARRIVAL_RATE, REROUTE_ADVISORY, FCA -> "MEDIUM";
            case FEA -> "LOW";
            default -> "REVIEW";
        };
    }

    private TrafficReplayBundle emptyBundle() {
        return TrafficReplayBundle.builder()
                .id("empty-replay")
                .sourceMode("LOCAL_FIXTURE_REPLAY")
                .authorizationMode("LOCAL_FIXTURE_ONLY")
                .build();
    }

    private TrafficReplayFlightPlan flightPlan(String flightId,
                                               String callsign,
                                               AircraftClass aircraftClass,
                                               List<List<Double>> route,
                                               String origin,
                                               String destination,
                                               String scenarioId) {
        int requestedAltitude = route.stream()
                .filter(point -> point.size() > 2)
                .mapToInt(point -> point.get(2).intValue())
                .findFirst()
                .orElse(25000);
        return TrafficReplayFlightPlan.builder()
                .flightId(flightId)
                .callsign(callsign)
                .aircraftClass(aircraftClass)
                .operator("SIM")
                .origin(origin)
                .destination(destination)
                .filedRouteText(routeText(route))
                .filedRoutePoints(route)
                .scheduledDepartureTime("T+0M")
                .estimatedDepartureTime("T+0M")
                .estimatedArrivalTime("T+60M")
                .requestedAltitudeFeet(requestedAltitude)
                .requestedAltitudeBlock("FL" + Math.max(0, requestedAltitude / 100 - 20) + "-FL" + (requestedAltitude / 100 + 20))
                .filedSpeedKnots(430)
                .sourceRefs(List.of("TFMS_REPLAY:" + scenarioId + ":" + flightId))
                .build();
    }

    private List<TrafficReplayPosition> positionsFor(TrafficReplayFlightPlan plan, int aircraftIndex) {
        List<List<Double>> route = plan.getFiledRoutePoints();
        List<TrafficReplayPosition> positions = new ArrayList<>();
        for (int i = 0; i < route.size(); i++) {
            List<Double> point = route.get(i);
            double progress = route.size() <= 1 ? 1.0 : i / (double) (route.size() - 1);
            positions.add(TrafficReplayPosition.builder()
                    .flightId(plan.getFlightId())
                    .timestamp("T+" + (i * 5) + "M")
                    .offsetMinutes(i * 5)
                    .latitude(point.get(0))
                    .longitude(point.get(1))
                    .altitudeFeet(point.size() > 2 ? point.get(2) : 25000.0)
                    .groundSpeedKnots(aircraftIndex == 0 ? 430.0 : aircraftIndex == 1 ? 450.0 : 410.0)
                    .headingDegrees(80.0)
                    .routeProgress(progress)
                    .phase(i == 0 ? "DEPARTURE" : i == route.size() - 1 ? "ARRIVAL" : "ENROUTE")
                    .sourceRefs(List.of("TRACK:" + plan.getFlightId() + ":T+" + (i * 5) + "M"))
                    .build());
        }
        return positions;
    }

    private TrafficReplayAirportDemand airportDemand(String airport, int minute, int depDemand, int arrDemand, int depCapacity, int arrCapacity, int queue, int delay) {
        return TrafficReplayAirportDemand.builder()
                .airportId(airport)
                .timestamp("T+" + minute + "M")
                .offsetMinutes(minute)
                .departureDemandPerHour(depDemand)
                .arrivalDemandPerHour(arrDemand)
                .departureCapacityPerHour(depCapacity)
                .arrivalCapacityPerHour(arrCapacity)
                .departureQueueDepth(queue)
                .averageDelaySeconds(delay)
                .runwayConfiguration("04R/04L")
                .sourceRefs(List.of("AIRPORT_DEMAND:" + airport + ":T+" + minute + "M"))
                .build();
    }

    private TrafficReplaySectorDemand sectorDemand(String sector, int minute, int active, int baseline, int queue, double frequency, int delay) {
        return TrafficReplaySectorDemand.builder()
                .sectorId(sector)
                .timestamp("T+" + minute + "M")
                .offsetMinutes(minute)
                .activeAircraft(active)
                .baselineCapacity(baseline)
                .handoffQueueDepth(queue)
                .frequencyUtilization(frequency)
                .estimatedHandoffDelaySeconds(delay)
                .sourceRefs(List.of("SECTOR_DEMAND:" + sector + ":T+" + minute + "M"))
                .build();
    }

    private List<List<Double>> offsetRoute(List<List<Double>> route, double offset) {
        return route.stream()
                .map(point -> {
                    List<Double> shifted = new ArrayList<>();
                    shifted.add(point.get(0) + offset);
                    shifted.add(point.get(1) + offset);
                    if (point.size() > 2) shifted.add(point.get(2));
                    return shifted;
                })
                .toList();
    }

    private String routeText(List<List<Double>> route) {
        return route.stream()
                .map(point -> String.format(Locale.US, "%.3f,%.3f", point.get(0), point.get(1)))
                .reduce((a, b) -> a + " " + b)
                .orElse("");
    }

    private String altitudeBlock(TrafficReplayFlightPlan plan) {
        if (!blank(plan.getRequestedAltitudeBlock())) return plan.getRequestedAltitudeBlock();
        if (plan.getRequestedAltitudeFeet() != null) return "FL" + (plan.getRequestedAltitudeFeet() / 100);
        return null;
    }

    private double clampedProgress(TrafficReplayPosition position) {
        return Math.max(0.0, Math.min(1.0, position.getRouteProgress()));
    }

    private String phaseFromSpeed(double speed) {
        return speed <= 1.0 ? "HELD" : "ENROUTE";
    }

    private AircraftPerformanceProfile profileFor(AircraftClass aircraftClass) {
        return switch (aircraftClass) {
            case HEAVY_JET -> AircraftPerformanceProfile.builder().aircraftClass(aircraftClass).cruiseSpeedKnots(470).climbRateFeetPerMinute(1800).descentRateFeetPerMinute(1600).fuelBurnPoundsPerMinute(210).takeoffDistanceFeet(9500).landingDistanceFeet(6200).minimumRvrFeet(1200).brakingActionPenaltyFactor(1.25).assumptions("Replay-derived representative heavy jet profile.").build();
            case NARROWBODY -> AircraftPerformanceProfile.builder().aircraftClass(aircraftClass).cruiseSpeedKnots(450).climbRateFeetPerMinute(2200).descentRateFeetPerMinute(1800).fuelBurnPoundsPerMinute(145).takeoffDistanceFeet(7600).landingDistanceFeet(5200).minimumRvrFeet(1000).brakingActionPenaltyFactor(1.18).assumptions("Replay-derived representative narrowbody profile.").build();
            case REGIONAL_JET -> AircraftPerformanceProfile.builder().aircraftClass(aircraftClass).cruiseSpeedKnots(410).climbRateFeetPerMinute(2000).descentRateFeetPerMinute(1700).fuelBurnPoundsPerMinute(92).takeoffDistanceFeet(6000).landingDistanceFeet(4300).minimumRvrFeet(1200).brakingActionPenaltyFactor(1.15).assumptions("Replay-derived representative regional jet profile.").build();
            case TURBOPROP -> AircraftPerformanceProfile.builder().aircraftClass(aircraftClass).cruiseSpeedKnots(280).climbRateFeetPerMinute(1400).descentRateFeetPerMinute(1300).fuelBurnPoundsPerMinute(48).takeoffDistanceFeet(3800).landingDistanceFeet(3000).minimumRvrFeet(1600).brakingActionPenaltyFactor(1.10).assumptions("Replay-derived representative turboprop profile.").build();
            case MILITARY_REFUELING -> AircraftPerformanceProfile.builder().aircraftClass(aircraftClass).cruiseSpeedKnots(430).climbRateFeetPerMinute(1500).descentRateFeetPerMinute(1500).fuelBurnPoundsPerMinute(180).takeoffDistanceFeet(8400).landingDistanceFeet(5600).minimumRvrFeet(1200).brakingActionPenaltyFactor(1.22).assumptions("Replay-derived representative refueling profile.").build();
            default -> AircraftPerformanceProfile.builder().aircraftClass(AircraftClass.GENERIC).cruiseSpeedKnots(400).climbRateFeetPerMinute(1600).descentRateFeetPerMinute(1500).fuelBurnPoundsPerMinute(100).takeoffDistanceFeet(6500).landingDistanceFeet(4500).minimumRvrFeet(1200).brakingActionPenaltyFactor(1.15).assumptions("Replay-derived generic profile.").build();
        };
    }

    private List<String> refsForBundle(TrafficReplayBundle bundle) {
        if (bundle == null) return List.of();
        List<String> refs = new ArrayList<>();
        refs.add("TRAFFIC_REPLAY:" + valueOr(bundle.getId(), "unknown"));
        if (bundle.getSourceRefs() != null) {
            refs.addAll(bundle.getSourceRefs());
        }
        return refs.stream().filter(ref -> ref != null && !ref.isBlank()).distinct().toList();
    }

    @SafeVarargs
    private List<String> mergedRefs(List<String>... refs) {
        List<String> merged = new ArrayList<>();
        for (List<String> group : refs) {
            if (group != null) {
                group.stream().filter(ref -> ref != null && !ref.isBlank()).forEach(merged::add);
            }
        }
        return merged.stream().distinct().toList();
    }

    private String valueOr(String value, String fallback) {
        return blank(value) ? fallback : value;
    }

    private boolean blank(String value) {
        return value == null || value.trim().isEmpty();
    }

    @FunctionalInterface
    private interface OffsetExtractor<T> {
        int offsetMinutes(T item);
    }
}
