package org.tash.extensions.simulation;

import org.junit.jupiter.api.Test;
import org.tash.extensions.product.application.AirspaceProductService;
import org.tash.extensions.workflow.InMemoryReservationWorkflowRepository;
import org.tash.extensions.workflow.ReservationWorkflowService;

import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class OperationalSimulationServiceTest {
    private OperationalSimulationService service() {
        return new OperationalSimulationService(
                new AirspaceProductService(new ReservationWorkflowService(new InMemoryReservationWorkflowRepository())));
    }

    @Test
    void exposesCanonicalScenarioLibraryWithLocalFixtureBoundaries() {
        OperationalSimulationService service = service();

        List<SimulationScenario> scenarios = service.scenarios();

        assertEquals(9, scenarios.size());
        assertEquals("low-vis-rvr-smgcs", scenarios.get(0).getId());
        assertTrue(scenarios.stream().anyMatch(scenario -> scenario.getId().equals("sector-capacity-compression")));
        assertTrue(scenarios.stream().allMatch(scenario -> scenario.getCarfAltrv().contains("A. ")));
        assertTrue(scenarios.stream().allMatch(scenario -> scenario.getCarfAltrv().contains("F. FL240-FL280")));
        assertTrue(scenarios.stream().allMatch(scenario -> !scenario.getEvents().isEmpty()));
        assertTrue(scenarios.stream().allMatch(scenario -> scenario.getSensitivityDefaults().containsKey("forecastConfidence")));
    }

    @Test
    void defaultRunPersistsTimelineFeaturesKpisAndArtifactReferences() {
        OperationalSimulationService service = service();

        SimulationRunResult result = service.run(null);

        assertEquals("low-vis-rvr-smgcs", result.getScenarioId());
        assertEquals("DELAY", result.getFinalAction());
        assertEquals("DELAY_OR_CAUTION", result.getExpectedFinalAction());
        assertEquals(6, result.getSteps().size());
        assertEquals(result.getSteps(), service.timeline(result.getId()));
        assertEquals(12, service.features(result.getId()).getFeatures().size());
        assertEquals(300, result.getKpiSummary().getTimeToGuidanceSeconds());
        assertEquals(6, result.getKpiSummary().getMinuteStepCount());
        assertEquals(6, result.getKpiSummary().getAircraftStateUpdateCount());
        assertTrue(result.getKpiSummary().getMaxSurfaceDelaySeconds() > 0);
        assertEquals(1.0, result.getKpiSummary().getSourceRefPreservationRate());
        assertTrue(result.getKpiSummary().isPilotBriefAvailable());
        assertEquals("CLOCK", result.getSteps().get(1).getInjectedEvent().getFamily());
        assertEquals("DELAY", result.getSteps().get(5).getEngineAction());
        assertEquals("ASK_ATC_FOR_PROCEDURE_CONFIRMATION", result.getSteps().get(5).getDynamics().getPilotOperator().getPilotAction());
        assertTrue(result.getSteps().get(5).getDynamics().getAirportSurface().isTerminologyAmbiguity());
        assertTrue(result.getSteps().get(5).getAffectedMissionDeltas().stream()
                .anyMatch(delta -> delta.contains("MONITOR -> DELAY")));
        assertTrue(result.getSteps().get(5).getReplayAuditIds().containsKey("audit"));
        assertNotNull(result.getSteps().get(5).getCoordinationDraft().getRawText());
        assertNotNull(result.getSteps().get(5).getPilotBrief().getPrintableText());
        assertNotNull(result.getWorldState());
        assertEquals(6, result.getWorldState().getTicks().size());
        assertEquals(result.getWorldState(), service.worldState(result.getId()));
        assertNotNull(result.getReplayBundle());
        assertEquals(result.getReplayBundle(), service.replay(result.getId()));
        assertEquals(result.getId(), result.getReplayBundle().getRunId());
        assertTrue(result.getReplayBundle().getReplayHashes().containsKey("result"));
        assertFalse(result.getWorldState().getTicks().get(5).getAircraft().isEmpty());
        assertFalse(result.getWorldState().getTicks().get(5).getBehaviorStates().isEmpty());
    }

    @Test
    void requestTrafficReplayDrivesAircraftAirportAndSectorState() {
        OperationalSimulationService service = service();
        SimulationRunRequest request = new SimulationRunRequest();
        request.setScenarioId("low-vis-rvr-smgcs");
        request.setTrafficReplay(TrafficReplayBundle.builder()
                .id("tfms-like-unit-replay")
                .sourceId("recorded-swim-fixture:unit")
                .sourceMode("LOCAL_FIXTURE_REPLAY")
                .providerFamily("TFMS_LIKE_RECORDED_REPLAY")
                .authorizationMode("LOCAL_FIXTURE_ONLY")
                .flightPlans(List.of(TrafficReplayFlightPlan.builder()
                        .flightId("BAW1")
                        .callsign("SPEEDBIRD1")
                        .aircraftClass(AircraftClass.HEAVY_JET)
                        .origin("KJFK")
                        .destination("EGLL")
                        .filedRoutePoints(List.of(List.of(40.64, -73.78, 0.0), List.of(41.00, -72.70, 15000.0)))
                        .requestedAltitudeBlock("FL330-FL370")
                        .sourceRefs(List.of("TFMS_FLIGHT:BAW1"))
                        .build()))
                .positions(List.of(
                        TrafficReplayPosition.builder().flightId("BAW1").offsetMinutes(0).latitude(40.64).longitude(-73.78).altitudeFeet(0).groundSpeedKnots(0).routeProgress(0).phase("GATE").sourceRefs(List.of("TRACK:BAW1:T0")).build(),
                        TrafficReplayPosition.builder().flightId("BAW1").offsetMinutes(5).latitude(40.70).longitude(-73.60).altitudeFeet(6000).groundSpeedKnots(260).routeProgress(0.2).phase("DEPARTURE").sourceRefs(List.of("TRACK:BAW1:T5")).build()))
                .airportDemand(List.of(TrafficReplayAirportDemand.builder()
                        .airportId("KJFK")
                        .offsetMinutes(5)
                        .departureDemandPerHour(38)
                        .arrivalDemandPerHour(26)
                        .departureCapacityPerHour(10)
                        .arrivalCapacityPerHour(16)
                        .departureQueueDepth(11)
                        .averageDelaySeconds(480)
                        .runwayConfiguration("04R/04L")
                        .sourceRefs(List.of("AIRPORT_DEMAND:KJFK:T5"))
                        .build()))
                .sectorDemand(List.of(TrafficReplaySectorDemand.builder()
                        .sectorId("ZNY-N90")
                        .offsetMinutes(5)
                        .activeAircraft(35)
                        .baselineCapacity(28)
                        .handoffQueueDepth(7)
                        .frequencyUtilization(0.93)
                        .estimatedHandoffDelaySeconds(155)
                        .sourceRefs(List.of("SECTOR_DEMAND:ZNY-N90:T5"))
                        .build()))
                .trafficManagementInitiatives(List.of(TrafficManagementInitiative.builder()
                        .id("TMI-LOWVIS")
                        .type("GDP")
                        .primitiveType(TrafficManagementInitiativeType.GDP)
                        .reason("Low visibility departure compression")
                        .startOffsetMinutes(2)
                        .endOffsetMinutes(20)
                        .expectedDelayMinutes(8)
                        .confidence(0.8)
                        .affectedFlightIds(List.of("BAW1"))
                        .flowProgram(TrafficFlowProgramModel.builder()
                                .programId("GDP-LOWVIS")
                                .programType(TrafficManagementInitiativeType.GDP)
                                .targetAirport("KJFK")
                                .arrivalRatePerHour(16)
                                .edctWindowMinutes(5)
                                .affectedFlightIds(List.of("BAW1"))
                                .sourceRefs(List.of("TMI:TMI-LOWVIS"))
                                .build())
                        .sourceRefs(List.of("TMI:TMI-LOWVIS"))
                        .build()))
                .assumptions(List.of("Recorded local fixture; not live SWIM traffic."))
                .build());

        SimulationRunResult result = service.run(request);
        SimulationStepResult step = result.getSteps().get(5);

        assertEquals("BAW1", step.getDynamics().getAircraft().getAircraftId());
        assertEquals(40.70, step.getDynamics().getAircraft().getLatitude(), 0.001);
        assertEquals(11, step.getDynamics().getAirportSurface().getDepartureQueueDepth());
        assertEquals(480, step.getDynamics().getAirportSurface().getSurfaceDelaySeconds());
        assertEquals("ZNY-N90", step.getDynamics().getSectorWorkload().getSectorId());
        assertEquals(35, step.getDynamics().getSectorWorkload().getActiveAircraft());
        assertEquals(155, step.getDynamics().getSectorWorkload().getEstimatedHandoffDelaySeconds());
        assertEquals("tfms-like-unit-replay", step.getDynamics().getTrafficReplay().getReplaySourceId());
        assertEquals(1, step.getDynamics().getTrafficReplay().getReplayedFlightPlanCount());
        assertEquals(1, step.getDynamics().getTrafficReplay().getActiveTrafficManagementInitiativeCount());
        assertTrue(step.getDynamics().getTrafficReplay().getActiveTmiTypes().contains("GDP"));
        assertTrue(step.getDynamics().getTrafficReplay().getActiveTmiRecommendationCount() >= 1);
        assertTrue(step.getDynamics().getTrafficManagementRecommendations().stream()
                .anyMatch(recommendation -> recommendation.getRecommendedType() == TrafficManagementInitiativeType.GDP));
        assertFalse(step.getDynamics().getTrafficReplay().isLiveSwimNasDataUsed());
        assertEquals("SPEEDBIRD1", result.getWorldState().getTicks().get(5).getAircraft().get(0).getCallsign());
        assertEquals("EDCT_OR_METERING_REVIEW", result.getWorldState().getTicks().get(5).getAircraft().get(0).getRerouteAssignment());
    }

    @Test
    void nationalDemandCapacityConfigFeedsRunReplayWorldStateAndKpis() {
        OperationalSimulationService service = service();
        SimulationRunRequest request = new SimulationRunRequest();
        request.setScenarioId("sector-capacity-compression");
        request.setDurationMinutes(30);
        request.setTickIntervalSeconds(300);
        request.setNationalDemandCapacityConfig(NationalDemandCapacityConfig.builder()
                .id("ops-test")
                .flightCount(500)
                .airportCount(10)
                .sectorCount(12)
                .durationMinutes(60)
                .tickIntervalMinutes(5)
                .randomSeed(99)
                .demandSpikeFactor(1.5)
                .capacityReductionFactor(0.65)
                .build());

        SimulationRunResult result = service.run(request);

        assertNotNull(result.getNationalDemandCapacityReport());
        assertEquals(500, result.getNationalDemandCapacityReport().getFlightCount());
        assertEquals(500, result.getKpiSummary().getNationalFlightCount());
        assertTrue(result.getKpiSummary().getNationalTmiRecommendationCount() > 0);
        assertTrue(result.getKpiSummary().getPeakAirportDemandCapacityRatio() >= 0);
        assertTrue(result.getKpiSummary().getPeakNationalSectorDemandCapacityRatio() >= 0);
        assertNotNull(result.getWorldState().getNationalDemandCapacityReport());
        assertNotNull(result.getWorldState().getTicks().get(0).getNationalDemandCapacity());
        assertNotNull(result.getSteps().get(0).getDynamics().getNationalDemandCapacity());
        assertEquals("LOCAL_SYNTHETIC_NAS_SCALE", result.getSteps().get(0).getDynamics().getTrafficReplay().getSourceMode());
        assertTrue(result.getSteps().get(0).getDynamics().getTrafficReplay().getReplayedFlightPlanCount() >= 500);
    }

    @Test
    void scenarioBundlesValidateImportAndPowerDraftAgentWorkflows() {
        OperationalSimulationService service = service();
        ScenarioBundle bundle = service.scenarioBundle("low-vis-rvr-smgcs");

        ScenarioValidationResult validation = service.validateScenario(bundle);

        assertTrue(validation.isAccepted());
        assertEquals("low-vis-rvr-smgcs", validation.getScenarioId());
        assertNotNull(bundle.getTrafficFlow());
        assertFalse(bundle.getTrafficFlow().getAircraft().isEmpty());
        assertNotNull(bundle.getTrafficReplay());
        assertEquals("LOCAL_FIXTURE_REPLAY", bundle.getTrafficReplay().getSourceMode());
        assertFalse(bundle.getTrafficReplay().getFlightPlans().isEmpty());
        assertNotNull(bundle.getAirportOps());
        assertNotNull(bundle.getWeatherEnsembleConfig());

        SimulationScenario importedScenario = bundle.getScenario().toBuilder()
                .id("imported-low-vis")
                .name("Imported Low Visibility")
                .build();
        ScenarioBundle imported = service.importScenario(bundle.toBuilder()
                .id("imported-low-vis")
                .scenario(importedScenario)
                .build());

        assertEquals("imported-low-vis", imported.getId());
        assertTrue(service.scenarios().stream().anyMatch(scenario -> scenario.getId().equals("imported-low-vis")));

        SimulationAgentRequest request = new SimulationAgentRequest();
        request.setScenarioType("LOW_VISIBILITY_PROCEDURE_AMBIGUITY");
        request.setCount(2);
        request.setFocusAreas(List.of("RVR", "SMGCS"));

        SimulationAgentReport generated = service.generateScenarios(request);
        assertEquals(2, generated.getGeneratedScenarioDrafts().size());
        assertTrue(generated.getPolicyGuards().contains("NO_AUTONOMOUS_IMPORT"));

        SimulationAgentReport redTeam = service.redTeam(new SimulationAgentRequest());
        assertEquals("UNSAFE_GUIDANCE_RED_TEAM", redTeam.getAgentType());
        assertTrue(redTeam.getPolicyGuards().contains("NO_EXTERNAL_SEND"));
    }

    @Test
    void runsEveryCanonicalScenarioWithExpectedActionsAndMapLayers() {
        OperationalSimulationService service = service();

        for (SimulationScenario scenario : service.scenarios()) {
            SimulationRunRequest request = new SimulationRunRequest();
            request.setScenarioId(scenario.getId());
            request.setActor("scenario-test");

            SimulationRunResult result = service.run(request);

            assertTrue(scenario.getExpectedFinalAction().contains(result.getFinalAction()),
                    () -> scenario.getId() + " produced " + result.getFinalAction());
            assertFalse(result.getSteps().isEmpty());
            assertFalse(result.getMissionId().isBlank());
            assertFalse(result.getReservationId().isBlank());
            assertTrue(result.getKpiSummary().getReplayVerificationPassRate() > 0.0);
            assertTrue(service.features(result.getId()).getFeatures().stream()
                    .anyMatch(feature -> "simulation-event".equals(feature.getProperties().get("featureKind"))));
        }
    }

    @Test
    void appliesSensitivityOverridesOnlyWhenRequested() {
        OperationalSimulationService service = service();

        SimulationRunRequest withoutFlag = new SimulationRunRequest();
        withoutFlag.setScenarioId("oceanic-altrv-convection");
        withoutFlag.setSensitivityOverrides(Map.of("communicationDelaySeconds", 90.0));

        SimulationRunRequest withFlag = new SimulationRunRequest();
        withFlag.setScenarioId("oceanic-altrv-convection");
        withFlag.setIncludeSensitivity(true);
        withFlag.setSensitivityOverrides(Map.of("communicationDelaySeconds", 90.0, "routeCorridorBufferNm", 55.0));

        SimulationRunResult defaultSensitivity = service.run(withoutFlag);
        SimulationRunResult overridden = service.run(withFlag);

        assertEquals(30.0, defaultSensitivity.getSensitivity().get("communicationDelaySeconds"));
        assertEquals(90.0, overridden.getSensitivity().get("communicationDelaySeconds"));
        assertEquals(55.0, overridden.getSensitivity().get("routeCorridorBufferNm"));
    }

    @Test
    void campaignCanRunAllScenariosOrASelectedSubset() {
        OperationalSimulationService service = service();

        SimulationCampaignReport all = service.campaign(null);

        assertEquals(9, all.getScenarioCount());
        assertEquals(9, all.getRuns().size());
        assertEquals(all, service.campaignById(all.getId()));
        assertTrue(all.getAggregateKpis().getSourceRefPreservationRate() > 0.0);

        SimulationCampaignRequest subset = new SimulationCampaignRequest();
        subset.setScenarioIds(List.of("pirep-safety-override", "blocked-no-viable-reroute"));
        subset.setActor("subset");
        SimulationCampaignReport report = service.campaign(subset);

        assertEquals(2, report.getScenarioCount());
        assertEquals(2, report.getPassedScenarioCount());
        assertTrue(report.getRuns().stream().anyMatch(run -> run.getFinalAction().equals("BLOCKED")));
    }

    @Test
    void unknownIdsFailClearly() {
        OperationalSimulationService service = service();
        SimulationRunRequest request = new SimulationRunRequest();
        request.setScenarioId("missing-scenario");

        assertThrows(IllegalArgumentException.class, () -> service.run(request));
        assertThrows(IllegalArgumentException.class, () -> service.runById("missing-run"));
        assertThrows(IllegalArgumentException.class, () -> service.timeline("missing-run"));
        assertThrows(IllegalArgumentException.class, () -> service.features("missing-run"));
        assertThrows(IllegalArgumentException.class, () -> service.worldState("missing-run"));
        assertThrows(IllegalArgumentException.class, () -> service.replay("missing-run"));
        assertThrows(IllegalArgumentException.class, () -> service.campaignById("missing-campaign"));
    }

    @Test
    void timelineAndClockHelpersAreDeterministic() {
        SimulationEvent later = SimulationEvent.builder()
                .id("b")
                .offsetMinutes(10)
                .family("WEATHER")
                .label("Later")
                .payload("SIGMET")
                .build();
        SimulationEvent earlier = later.toBuilder().id("a").offsetMinutes(0).label("Earlier").build();
        ZonedDateTime start = ZonedDateTime.of(2026, 6, 20, 12, 0, 0, 0, ZoneOffset.UTC);

        SimulationTimeline timeline = SimulationTimeline.builder()
                .startTime(start)
                .endTime(start.plusMinutes(20))
                .events(List.of(later, earlier))
                .build();
        SimulationClock clock = SimulationClock.builder().startTime(start).build();

        assertEquals(List.of(earlier, later), timeline.orderedEvents());
        assertEquals(start.plusMinutes(7), clock.atOffsetMinutes(7));
    }
}
