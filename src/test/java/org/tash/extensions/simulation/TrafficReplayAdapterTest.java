package org.tash.extensions.simulation;

import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class TrafficReplayAdapterTest {
    private final TrafficReplayAdapter adapter = new TrafficReplayAdapter();

    @Test
    void parsesRecordedSwimTfmsLikeReplayAndConvertsToTrafficFlow() {
        TrafficReplayBundle bundle = adapter.parseJson(sampleReplayJson());

        assertEquals("tfms-replay-jfk-lowvis", bundle.getId());
        assertEquals("LOCAL_FIXTURE_REPLAY", bundle.getSourceMode());
        assertEquals(2, bundle.getFlightPlans().size());
        assertEquals(4, bundle.getPositions().size());
        assertTrue(adapter.validate(bundle).isEmpty());

        TrafficFlowScenario trafficFlow = adapter.toTrafficFlowScenario(bundle);

        assertEquals("traffic-flow-tfms-replay-jfk-lowvis", trafficFlow.getId());
        assertEquals("LOCAL_FIXTURE_REPLAY", trafficFlow.getSourceMode());
        assertEquals(2, trafficFlow.getAircraft().size());
        assertEquals("SPEEDBIRD1", trafficFlow.getAircraft().get(0).getCallsign());
    }

    @Test
    void returnsMinuteScopedAircraftDemandSectorAndTmiState() {
        TrafficReplayBundle bundle = adapter.parseJson(sampleReplayJson());

        List<SimulatedAircraft> aircraft = adapter.aircraftAtMinute(bundle, 5, List.of("WEATHER:sigmet-1"));

        assertEquals(2, aircraft.size());
        assertEquals("SPEEDBIRD1", aircraft.get(0).getCallsign());
        assertEquals(30.45, aircraft.get(0).getTrajectory().getLatitude(), 0.001);
        assertEquals("EDCT_OR_METERING_REVIEW", aircraft.get(0).getRerouteAssignment());
        assertTrue(aircraft.get(0).getImpactedSourceRefs().contains("WEATHER:sigmet-1"));
        assertTrue(aircraft.get(0).getImpactedSourceRefs().contains("TRAFFIC_REPLAY:tfms-replay-jfk-lowvis"));

        TrafficReplayAirportDemand airport = adapter.airportDemandAtMinute(bundle, 5).orElseThrow();
        assertEquals("KJFK", airport.getAirportId());
        assertEquals(36, airport.getDepartureDemandPerHour());
        assertEquals(9, airport.getDepartureQueueDepth());

        TrafficReplaySectorDemand sector = adapter.sectorDemandAtMinute(bundle, 5).orElseThrow();
        assertEquals("ZNY-N90", sector.getSectorId());
        assertEquals(34, sector.getActiveAircraft());
        assertEquals(0.94, sector.getFrequencyUtilization(), 0.001);

        assertEquals(1, adapter.activeTmisAtMinute(bundle, 5).size());
        assertEquals(4, adapter.recommendationsAtMinute(bundle, 5, airport, sector, "REROUTE").size());
        assertEquals(2, adapter.activeFlightCountAtMinute(bundle, 5));
    }

    @Test
    void doesNotCountFlightBeforeFirstRecordedPosition() {
        TrafficReplayBundle bundle = TrafficReplayBundle.builder()
                .id("activation-window")
                .sourceMode("LOCAL_FIXTURE_REPLAY")
                .flightPlans(List.of(TrafficReplayFlightPlan.builder()
                        .flightId("FUTURE1")
                        .callsign("FUTURE1")
                        .aircraftClass(AircraftClass.NARROWBODY)
                        .sourceRefs(List.of())
                        .build()))
                .positions(List.of(TrafficReplayPosition.builder()
                        .flightId("FUTURE1")
                        .offsetMinutes(10)
                        .latitude(40.0)
                        .longitude(-73.0)
                        .altitudeFeet(0)
                        .groundSpeedKnots(0)
                        .phase("GATE")
                        .sourceRefs(List.of())
                        .build()))
                .build();

        assertEquals(0, adapter.activeFlightCountAtMinute(bundle, 5));
        assertTrue(adapter.positionAtMinute(bundle, "FUTURE1", 5).isEmpty());
        assertEquals(1, adapter.activeFlightCountAtMinute(bundle, 10));
        assertEquals("FUTURE1", adapter.aircraftAtMinute(bundle, 10, List.of()).get(0).getCallsign());
    }

    @Test
    void modelsCoreTrafficFlowManagementPrimitives() {
        TrafficReplayBundle bundle = TrafficReplayBundle.builder()
                .id("tfm-primitives")
                .sourceMode("LOCAL_FIXTURE_REPLAY")
                .flightPlans(List.of(TrafficReplayFlightPlan.builder().flightId("F1").callsign("F1").aircraftClass(AircraftClass.NARROWBODY).sourceRefs(List.of()).build()))
                .positions(List.of(TrafficReplayPosition.builder().flightId("F1").offsetMinutes(5).latitude(1).longitude(2).altitudeFeet(30000).groundSpeedKnots(430).phase("ENROUTE").sourceRefs(List.of()).build()))
                .airportDemand(List.of(TrafficReplayAirportDemand.builder().airportId("KJFK").offsetMinutes(5).departureDemandPerHour(36).departureCapacityPerHour(18).arrivalDemandPerHour(30).arrivalCapacityPerHour(20).averageDelaySeconds(600).sourceRefs(List.of("AIRPORT_DEMAND:KJFK:T5")).build()))
                .sectorDemand(List.of(TrafficReplaySectorDemand.builder().sectorId("ZNY-N90").offsetMinutes(5).activeAircraft(36).baselineCapacity(24).frequencyUtilization(0.9).estimatedHandoffDelaySeconds(180).sourceRefs(List.of("SECTOR_DEMAND:ZNY-N90:T5")).build()))
                .trafficManagementInitiatives(List.of(
                        TrafficManagementInitiative.builder().id("GDP-1").type("GDP").flowProgram(TrafficFlowProgramModel.builder().programId("GDP-1").programType(TrafficManagementInitiativeType.GDP).targetAirport("KJFK").arrivalRatePerHour(20).edctWindowMinutes(5).build()).startOffsetMinutes(0).endOffsetMinutes(30).affectedFlightIds(List.of("F1")).sourceRefs(List.of("TMI:GDP-1")).build(),
                        TrafficManagementInitiative.builder().id("AFP-1").primitiveType(TrafficManagementInitiativeType.AFP).flowProgram(TrafficFlowProgramModel.builder().programId("AFP-1").programType(TrafficManagementInitiativeType.AFP).flowAreaId("FCA-1").acceptanceRatePerHour(24).build()).startOffsetMinutes(0).endOffsetMinutes(30).affectedFlightIds(List.of("F1")).sourceRefs(List.of("TMI:AFP-1")).build(),
                        TrafficManagementInitiative.builder().id("FEA-1").type("FEA").flowArea(FlowEvaluationAreaModel.builder().areaId("FEA-1").areaType(TrafficManagementInitiativeType.FEA).geometryType("LINE").acceptanceRatePerHour(30).build()).startOffsetMinutes(0).endOffsetMinutes(30).sourceRefs(List.of("TMI:FEA-1")).build(),
                        TrafficManagementInitiative.builder().id("FCA-1").type("FCA").flowArea(FlowEvaluationAreaModel.builder().areaId("FCA-1").areaType(TrafficManagementInitiativeType.FCA).geometryType("POLYGON").controlArea(true).acceptanceRatePerHour(24).build()).startOffsetMinutes(0).endOffsetMinutes(30).affectedFlightIds(List.of("F1")).sourceRefs(List.of("TMI:FCA-1")).build(),
                        TrafficManagementInitiative.builder().id("MIT-1").type("MIT").milesInTrail(MilesInTrailRestrictionModel.builder().restrictionId("MIT-1").milesInTrail(20).fixId("MERIT").affectedFlightIds(List.of("F1")).build()).startOffsetMinutes(0).endOffsetMinutes(30).affectedFlightIds(List.of("F1")).sourceRefs(List.of("TMI:MIT-1")).build(),
                        TrafficManagementInitiative.builder().id("RR-1").type("REROUTE_ADVISORY").rerouteAdvisory(RerouteAdvisoryModel.builder().advisoryId("RR-1").routeText("PLAYBOOK EAST").affectedFlightIds(List.of("F1")).build()).startOffsetMinutes(0).endOffsetMinutes(30).affectedFlightIds(List.of("F1")).sourceRefs(List.of("TMI:RR-1")).build(),
                        TrafficManagementInitiative.builder().id("GS-1").type("GS").groundStop(GroundStopModel.builder().groundStopId("GS-1").airportId("KJFK").reason("Weather below minimums").affectedFlightIds(List.of("F1")).build()).startOffsetMinutes(0).endOffsetMinutes(30).affectedFlightIds(List.of("F1")).sourceRefs(List.of("TMI:GS-1")).build(),
                        TrafficManagementInitiative.builder().id("DSP-1").type("DSP").departureMetering(DepartureMeteringModel.builder().meteringId("DSP-1").commonPoint("MERIT").intervalSeconds(120).releaseRatePerHour(30).affectedFlightIds(List.of("F1")).build()).startOffsetMinutes(0).endOffsetMinutes(30).affectedFlightIds(List.of("F1")).sourceRefs(List.of("TMI:DSP-1")).build(),
                        TrafficManagementInitiative.builder().id("AAR-1").type("ARRIVAL_RATE").arrivalRate(ArrivalRateModel.builder().rateId("AAR-1").airportId("KJFK").acceptanceRatePerHour(20).demandRatePerHour(35).build()).startOffsetMinutes(0).endOffsetMinutes(30).affectedFlightIds(List.of("F1")).sourceRefs(List.of("TMI:AAR-1")).build(),
                        TrafficManagementInitiative.builder().id("SC-1").type("SECTOR_CAPACITY").sectorCapacity(SectorCapacityModel.builder().capacityId("SC-1").sectorId("ZNY-N90").baselineCapacity(30).reducedCapacity(20).demandCount(36).capacityState("COMPRESSED").build()).startOffsetMinutes(0).endOffsetMinutes(30).affectedFlightIds(List.of("F1")).sourceRefs(List.of("TMI:SC-1")).build()))
                .build();

        List<TrafficManagementInitiativeType> types = adapter.activeTmisAtMinute(bundle, 5).stream()
                .map(adapter::canonicalTmiType)
                .toList();

        assertTrue(types.contains(TrafficManagementInitiativeType.GDP));
        assertTrue(types.contains(TrafficManagementInitiativeType.AFP));
        assertTrue(types.contains(TrafficManagementInitiativeType.FEA));
        assertTrue(types.contains(TrafficManagementInitiativeType.FCA));
        assertTrue(types.contains(TrafficManagementInitiativeType.MILES_IN_TRAIL));
        assertTrue(types.contains(TrafficManagementInitiativeType.REROUTE_ADVISORY));
        assertTrue(types.contains(TrafficManagementInitiativeType.GROUND_STOP));
        assertTrue(types.contains(TrafficManagementInitiativeType.DEPARTURE_METERING));
        assertTrue(types.contains(TrafficManagementInitiativeType.ARRIVAL_RATE));
        assertTrue(types.contains(TrafficManagementInitiativeType.SECTOR_CAPACITY));
        assertEquals("GROUND_STOP_REVIEW", adapter.aircraftAtMinute(bundle, 5, List.of()).get(0).getRerouteAssignment());
        assertTrue(adapter.recommendationsAtMinute(bundle, 5, bundle.getAirportDemand().get(0), bundle.getSectorDemand().get(0), "BLOCKED").stream()
                .anyMatch(recommendation -> recommendation.getRecommendedType() == TrafficManagementInitiativeType.SECTOR_CAPACITY));
    }

    @Test
    void rejectsMalformedReplayJsonClearly() {
        IllegalArgumentException error = assertThrows(IllegalArgumentException.class, () -> adapter.parseJson("{not-json"));

        assertTrue(error.getMessage().contains("Malformed traffic replay JSON"));
    }

    @Test
    void defaultReplayForScenarioIsFixtureBackedAndComplete() {
        SimulationScenario scenario = SimulationScenario.builder()
                .id("unit")
                .name("Unit")
                .route(List.of(List.of(30.0, -150.0, 24000.0), List.of(31.0, -149.0, 26000.0)))
                .build();

        TrafficReplayBundle bundle = adapter.defaultReplayForScenario(scenario);

        assertEquals("traffic-replay-unit", bundle.getId());
        assertEquals("LOCAL_FIXTURE_REPLAY", bundle.getSourceMode());
        assertFalse(bundle.getFlightPlans().isEmpty());
        assertFalse(bundle.getAirportDemand().isEmpty());
        assertFalse(bundle.getSectorDemand().isEmpty());
        assertTrue(bundle.getAssumptions().get(0).contains("not live SWIM data"));
    }

    private String sampleReplayJson() {
        return """
                {
                  "id":"tfms-replay-jfk-lowvis",
                  "sourceId":"recorded-swim-fixture:jfk-lowvis",
                  "sourceMode":"LOCAL_FIXTURE_REPLAY",
                  "providerFamily":"TFMS_LIKE_RECORDED_REPLAY",
                  "generatedAt":"2026-06-20T12:00:00Z",
                  "timeBasis":"OFFSET_MINUTES",
                  "authorizationMode":"LOCAL_FIXTURE_ONLY",
                  "flightPlans":[
                    {
                      "flightId":"BAW1",
                      "callsign":"SPEEDBIRD1",
                      "aircraftClass":"HEAVY_JET",
                      "operator":"BAW",
                      "origin":"KJFK",
                      "destination":"EGLL",
                      "filedRouteText":"KJFK DCT ACK NAT",
                      "filedRoutePoints":[[30.0,-150.0,24000.0],[30.45,-149.55,25000.0]],
                      "scheduledDepartureTime":"2026-06-20T12:00:00Z",
                      "estimatedDepartureTime":"2026-06-20T12:08:00Z",
                      "estimatedArrivalTime":"2026-06-20T19:00:00Z",
                      "requestedAltitudeFeet":35000,
                      "requestedAltitudeBlock":"FL330-FL370",
                      "filedSpeedKnots":470,
                      "sourceRefs":["TFMS_FLIGHT:BAW1"]
                    },
                    {
                      "flightId":"AAL2",
                      "callsign":"AAL2",
                      "aircraftClass":"NARROWBODY",
                      "operator":"AAL",
                      "origin":"KJFK",
                      "destination":"KORD",
                      "filedRoutePoints":[[30.04,-150.04,22000.0],[30.50,-149.60,23000.0]],
                      "requestedAltitudeFeet":33000,
                      "sourceRefs":["TFMS_FLIGHT:AAL2"]
                    }
                  ],
                  "positions":[
                    {"flightId":"BAW1","offsetMinutes":0,"latitude":30.0,"longitude":-150.0,"altitudeFeet":24000,"groundSpeedKnots":0,"headingDegrees":80,"routeProgress":0.0,"phase":"GATE","sourceRefs":["TRACK:BAW1:T0"]},
                    {"flightId":"BAW1","offsetMinutes":5,"latitude":30.45,"longitude":-149.55,"altitudeFeet":25000,"groundSpeedKnots":430,"headingDegrees":80,"routeProgress":0.5,"phase":"DEPARTURE","sourceRefs":["TRACK:BAW1:T5"]},
                    {"flightId":"AAL2","offsetMinutes":0,"latitude":30.04,"longitude":-150.04,"altitudeFeet":22000,"groundSpeedKnots":0,"headingDegrees":80,"routeProgress":0.0,"phase":"GATE","sourceRefs":["TRACK:AAL2:T0"]},
                    {"flightId":"AAL2","offsetMinutes":5,"latitude":30.50,"longitude":-149.60,"altitudeFeet":23000,"groundSpeedKnots":410,"headingDegrees":80,"routeProgress":0.5,"phase":"DEPARTURE","sourceRefs":["TRACK:AAL2:T5"]}
                  ],
                  "airportDemand":[
                    {"airportId":"KJFK","offsetMinutes":0,"departureDemandPerHour":24,"arrivalDemandPerHour":22,"departureCapacityPerHour":24,"arrivalCapacityPerHour":24,"departureQueueDepth":1,"averageDelaySeconds":30,"runwayConfiguration":"04R/04L","sourceRefs":["AIRPORT_DEMAND:KJFK:T0"]},
                    {"airportId":"KJFK","offsetMinutes":5,"departureDemandPerHour":36,"arrivalDemandPerHour":28,"departureCapacityPerHour":12,"arrivalCapacityPerHour":18,"departureQueueDepth":9,"averageDelaySeconds":420,"runwayConfiguration":"04R/04L","sourceRefs":["AIRPORT_DEMAND:KJFK:T5"]}
                  ],
                  "sectorDemand":[
                    {"sectorId":"ZNY-N90","offsetMinutes":0,"activeAircraft":18,"baselineCapacity":28,"handoffQueueDepth":1,"frequencyUtilization":0.48,"estimatedHandoffDelaySeconds":20,"sourceRefs":["SECTOR_DEMAND:ZNY-N90:T0"]},
                    {"sectorId":"ZNY-N90","offsetMinutes":5,"activeAircraft":34,"baselineCapacity":28,"handoffQueueDepth":8,"frequencyUtilization":0.94,"estimatedHandoffDelaySeconds":165,"sourceRefs":["SECTOR_DEMAND:ZNY-N90:T5"]}
                  ],
                  "trafficManagementInitiatives":[
                    {"id":"TMI-JFK-LVIS","type":"GDP","primitiveType":"GDP","reason":"Low visibility departure compression","constraintId":"KJFK-LVIS","startOffsetMinutes":2,"endOffsetMinutes":30,"expectedDelayMinutes":12,"confidence":0.81,"affectedFlightIds":["BAW1"],"flowProgram":{"programId":"GDP-JFK-LVIS","programType":"GDP","targetAirport":"KJFK","arrivalRatePerHour":18,"edctWindowMinutes":5,"affectedFlightIds":["BAW1"],"sourceRefs":["TMI:TMI-JFK-LVIS"]},"sourceRefs":["TMI:TMI-JFK-LVIS"]}
                  ],
                  "assumptions":["Recorded local fixture; not live SWIM traffic."]
                }
                """;
    }
}
