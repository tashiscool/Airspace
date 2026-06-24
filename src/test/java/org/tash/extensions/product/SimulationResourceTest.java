package org.tash.extensions.product;

import io.quarkus.test.junit.QuarkusTest;
import org.junit.jupiter.api.Test;

import static io.restassured.RestAssured.given;
import static org.hamcrest.Matchers.equalTo;
import static org.hamcrest.Matchers.greaterThanOrEqualTo;
import static org.hamcrest.Matchers.hasItem;
import static org.hamcrest.Matchers.notNullValue;

@QuarkusTest
class SimulationResourceTest {
    @Test
    void scenariosExposeCanonicalAerospaceCapabilityStories() {
        given()
                .when()
                .get("/api/simulations/scenarios")
                .then()
                .statusCode(200)
                .body("id", hasItem("low-vis-rvr-smgcs"))
                .body("id", hasItem("oceanic-altrv-convection"))
                .body("id", hasItem("pirep-safety-override"))
                .body("id", hasItem("blocked-no-viable-reroute"))
                .body("find { it.id == 'low-vis-rvr-smgcs' }.capabilityStory", equalTo("Low Visibility Procedure Ambiguity"))
                .body("find { it.id == 'oceanic-altrv-convection' }.events.size()", greaterThanOrEqualTo(2));
    }

    @Test
    void singleSimulationRunProducesTimelineKpisFeaturesAndBriefArtifacts() {
        String runId = given()
                .contentType("application/json")
                .body("{\"scenarioId\":\"low-vis-rvr-smgcs\",\"actor\":\"tester\",\"includeSensitivity\":true,\"sensitivityOverrides\":{\"communicationDelaySeconds\":45}}")
                .when()
                .post("/api/simulations/run")
                .then()
                .statusCode(200)
                .body("scenarioId", equalTo("low-vis-rvr-smgcs"))
                .body("finalAction", equalTo("DELAY"))
                .body("expectedFinalAction", equalTo("DELAY_OR_CAUTION"))
                .body("steps.size()", equalTo(6))
                .body("steps[1].injectedEvent.family", equalTo("CLOCK"))
                .body("steps[5].engineAction", equalTo("DELAY"))
                .body("steps[5].sourceRefs", hasItem("NOTAM:evt-lowvis-2"))
                .body("steps[5].dynamics.aircraft.performancePhase", equalTo("DEPARTURE_SURFACE_REVIEW"))
                .body("steps[5].dynamics.airportSurface.terminologyAmbiguity", equalTo(true))
                .body("steps[5].dynamics.sectorWorkload.estimatedHandoffDelaySeconds", greaterThanOrEqualTo(30))
                .body("steps[5].dynamics.pilotOperator.pilotAction", equalTo("ASK_ATC_FOR_PROCEDURE_CONFIRMATION"))
                .body("steps[5].dynamics.weatherEvolution.ensembleMemberCount", greaterThanOrEqualTo(8))
                .body("steps[5].dynamics.trafficReplay.liveSwimNasDataUsed", equalTo(false))
                .body("steps[5].dynamics.trafficReplay.sourceMode", equalTo("LOCAL_FIXTURE_REPLAY"))
                .body("steps[5].dynamics.trafficReplay.replayedPositionCount", greaterThanOrEqualTo(3))
                .body("steps[5].dynamics.trafficReplay.activeTmiTypes", hasItem("REROUTE_ADVISORY"))
                .body("steps[5].dynamics.trafficReplay.activeTmiRecommendationCount", greaterThanOrEqualTo(1))
                .body("steps[5].dynamics.trafficManagementRecommendations.find { it.recommendedType == 'REROUTE_ADVISORY' }.action", equalTo("REVIEW_REROUTE_ADVISORY"))
                .body("steps[5].coordinationDraft.rawText", notNullValue())
                .body("steps[5].pilotBrief.printableText", notNullValue())
                .body("worldState.ticks.size()", equalTo(6))
                .body("worldState.ticks[5].aircraft.size()", greaterThanOrEqualTo(3))
                .body("replayBundle.replayHashes.result", notNullValue())
                .body("kpiSummary.timeToGuidanceSeconds", equalTo(300))
                .body("kpiSummary.minuteStepCount", equalTo(6))
                .body("kpiSummary.maxSurfaceDelaySeconds", greaterThanOrEqualTo(400))
                .body("kpiSummary.sourceRefPreservationRate", equalTo(1.0F))
                .body("kpiSummary.pilotBriefAvailable", equalTo(true))
                .extract()
                .path("id");

        given()
                .when()
                .get("/api/simulations/runs/" + runId + "/timeline")
                .then()
                .statusCode(200)
                .body("size()", equalTo(6))
                .body("[5].routeImpactDeltas.size()", greaterThanOrEqualTo(1));

        given()
                .when()
                .get("/api/simulations/runs/" + runId + "/features")
                .then()
                .statusCode(200)
                .body("type", equalTo("FeatureCollection"))
                .body("features.size()", greaterThanOrEqualTo(12))
                .body("features.find { it.properties.sourceFamily == 'NOTAM' }.properties.displayLayer", equalTo("notams"));

        given()
                .when()
                .get("/api/simulations/runs/" + runId + "/world-state")
                .then()
                .statusCode(200)
                .body("runId", equalTo(runId))
                .body("ticks.size()", equalTo(6))
                .body("ticks[5].behaviorStates.find { it.actor == 'PILOT' }.nextAction", equalTo("ASK_ATC_FOR_PROCEDURE_CONFIRMATION"));

        given()
                .when()
                .get("/api/simulations/runs/" + runId + "/replay")
                .then()
                .statusCode(200)
                .body("runId", equalTo(runId))
                .body("replayHashes.result", notNullValue())
                .body("steps.size()", equalTo(6));
    }

    @Test
    void scenarioBundlesAndSimulationAgentsAreHumanReviewReady() {
        given()
                .when()
                .get("/api/simulations/scenarios/low-vis-rvr-smgcs/bundle")
                .then()
                .statusCode(200)
                .body("scenario.id", equalTo("low-vis-rvr-smgcs"))
                .body("trafficFlow.aircraft.size()", greaterThanOrEqualTo(3))
                .body("trafficReplay.sourceMode", equalTo("LOCAL_FIXTURE_REPLAY"))
                .body("trafficReplay.flightPlans.size()", greaterThanOrEqualTo(3))
                .body("airportOps.runwayStates[0].airportId", equalTo("KJFK"))
                .body("weatherEnsembleConfig.memberCount", greaterThanOrEqualTo(8));

        String bundle = """
                {
                  "id":"api-import-low-vis",
                  "name":"API Imported Low Visibility",
                  "scenario":{
                    "id":"api-import-low-vis",
                    "name":"API Imported Low Visibility",
                    "capabilityStory":"Low Visibility Procedure Ambiguity",
                    "narrative":"Imported fixture for authoring tests.",
                    "missionNumber":"SIM-API-IMPORT",
                    "carfAltrv":"A. SIM-API-IMPORT\\nF. FL240-FL280",
                    "expectedFinalAction":"DELAY_OR_CAUTION",
                    "route":[[30.0,-150.0,24000.0],[31.0,-149.0,26000.0]],
                    "events":[{"id":"evt-api-1","offsetMinutes":0,"family":"METAR","payload":"METAR KJFK 201200Z 1/8SM R04R/1000FT FG","expectedAction":"DELAY","sourceRefs":["METAR:evt-api-1"]}],
                    "expectedSourceFamilies":["WEATHER"],
                    "sensitivityDefaults":{"forecastConfidence":0.75}
                  },
                  "kpiGates":[],
                  "expectedSummary":{}
                }
                """;

        given()
                .contentType("application/json")
                .body(bundle)
                .when()
                .post("/api/simulations/scenarios/validate")
                .then()
                .statusCode(200)
                .body("accepted", equalTo(true))
                .body("scenarioId", equalTo("api-import-low-vis"));

        given()
                .contentType("application/json")
                .body(bundle)
                .when()
                .post("/api/simulations/scenarios/import")
                .then()
                .statusCode(200)
                .body("scenario.id", equalTo("api-import-low-vis"));

        given()
                .contentType("application/json")
                .body("""
                        {
                          "id":"api-traffic-replay",
                          "sourceMode":"LOCAL_FIXTURE_REPLAY",
                          "flightPlans":[{"flightId":"BAW1","callsign":"SPEEDBIRD1","aircraftClass":"HEAVY_JET","origin":"KJFK","destination":"EGLL"}],
                          "positions":[{"flightId":"BAW1","offsetMinutes":0,"latitude":40.64,"longitude":-73.78,"altitudeFeet":0,"groundSpeedKnots":0}],
                          "airportDemand":[{"airportId":"KJFK","offsetMinutes":0,"departureDemandPerHour":24,"arrivalDemandPerHour":22,"departureCapacityPerHour":20,"arrivalCapacityPerHour":20}],
                          "sectorDemand":[{"sectorId":"ZNY-N90","offsetMinutes":0,"activeAircraft":18,"baselineCapacity":28}]
                        }
                        """)
                .when()
                .post("/api/simulations/traffic-replay/validate")
                .then()
                .statusCode(200)
                .body("accepted", equalTo(true))
                .body("flightPlanCount", equalTo(1))
                .body("positionCount", equalTo(1))
                .body("diagnostics[0]", equalTo("Traffic replay bundle is structurally complete for local simulation."));

        given()
                .contentType("application/json")
                .body("{\"id\":\"api-national\",\"flightCount\":400,\"airportCount\":8,\"sectorCount\":10,\"durationMinutes\":60,\"tickIntervalMinutes\":10,\"randomSeed\":5}")
                .when()
                .post("/api/simulations/national-demand/preview")
                .then()
                .statusCode(200)
                .body("id", equalTo("national-demand-api-national"))
                .body("flightCount", equalTo(400))
                .body("airportCount", equalTo(8))
                .body("sectorCount", equalTo(10))
                .body("trafficReplay.flightPlans.size()", equalTo(400))
                .body("snapshots.size()", greaterThanOrEqualTo(6))
                .body("totalTmiRecommendationCount", greaterThanOrEqualTo(1));

        given()
                .contentType("application/json")
                .body("{\"scenarioType\":\"LOW_VISIBILITY_PROCEDURE_AMBIGUITY\",\"count\":2,\"focusAreas\":[\"RVR\",\"SMGCS\"]}")
                .when()
                .post("/api/simulations/agents/generate-scenarios")
                .then()
                .statusCode(200)
                .body("agentType", equalTo("SCENARIO_GENERATION"))
                .body("generatedScenarioDrafts.size()", equalTo(2))
                .body("policyGuards", hasItem("NO_AUTONOMOUS_IMPORT"));

        given()
                .contentType("application/json")
                .body("{\"focusAreas\":[\"false-clear\",\"source-refs\"]}")
                .when()
                .post("/api/simulations/agents/red-team")
                .then()
                .statusCode(200)
                .body("agentType", equalTo("UNSAFE_GUIDANCE_RED_TEAM"))
                .body("policyGuards", hasItem("NO_EXTERNAL_SEND"));
    }

    @Test
    void historicalReplayCorpusLoadsRunsAndReportsCalibrationReadiness() {
        given()
                .when()
                .get("/api/simulations/historical-replay/days")
                .then()
                .statusCode(200)
                .body("id", hasItem("public-like-jfk-lowvis-opsnet-bts-awc"))
                .body("find { it.id == 'public-like-jfk-lowvis-opsnet-bts-awc' }.sourceMode", equalTo("PUBLIC_HISTORICAL_LIKE"))
                .body("find { it.id == 'public-like-jfk-lowvis-opsnet-bts-awc' }.authorizationMode", equalTo("PUBLIC_HISTORICAL_LIKE"))
                .body("find { it.id == 'public-like-jfk-lowvis-opsnet-bts-awc' }.publicSourceRefs", hasItem("FAA_OPSNET:public-finalized-monthly-delay-data"))
                .body("find { it.id == 'public-like-jfk-lowvis-opsnet-bts-awc' }.expectedOutcomes.size()", greaterThanOrEqualTo(2));

        given()
                .when()
                .get("/api/simulations/historical-replay/days/public-like-jfk-lowvis-opsnet-bts-awc")
                .then()
                .statusCode(200)
                .body("trafficReplay.sourceMode", equalTo("PUBLIC_HISTORICAL_LIKE"))
                .body("trafficReplay.providerFamily", equalTo("HISTORICAL_REPLAY_CORPUS"))
                .body("dataQualityWarnings", hasItem("No authorized operational historical feed was used."));

        String runId = given()
                .contentType("application/json")
                .body("{\"dayId\":\"public-like-jfk-lowvis-opsnet-bts-awc\",\"runSimulation\":true,\"actor\":\"historical-test\"}")
                .when()
                .post("/api/simulations/historical-replay/load")
                .then()
                .statusCode(200)
                .body("accepted", equalTo(true))
                .body("ranSimulation", equalTo(true))
                .body("sourceMode", equalTo("PUBLIC_HISTORICAL_LIKE"))
                .body("flightPlanCount", greaterThanOrEqualTo(3))
                .body("expectedOutcomeCount", greaterThanOrEqualTo(2))
                .body("warnings", hasItem("Corpus day is not authorized operational evidence; use for local replay/calibration readiness only."))
                .body("calibrationReport.calibrationVersion", equalTo("historical-replay-corpus-v1"))
                .extract()
                .path("runId");

        given()
                .when()
                .get("/api/simulations/runs/" + runId)
                .then()
                .statusCode(200)
                .body("worldState.ticks[0].aircraft.size()", greaterThanOrEqualTo(3))
                .body("worldState.ticks[0].aircraft[0].impactedSourceRefs", hasItem("FAA_OPSNET:public-finalized-monthly-delay-data"))
                .body("worldState.ticks[0].aircraft[0].impactedSourceRefs", hasItem("BTS_TRANSTATS:on-time-schedule-delay-proxy"));

        given()
                .contentType("application/json")
                .body("{}")
                .when()
                .post("/api/simulations/historical-replay/calibrate")
                .then()
                .statusCode(200)
                .body("corpusDayCount", greaterThanOrEqualTo(3))
                .body("expectedOutcomeCount", greaterThanOrEqualTo(6))
                .body("sourceModes", hasItem("PUBLIC_HISTORICAL_LIKE"))
                .body("sourceModes", hasItem("SYNTHETIC"))
                .body("sourceRefPreservationRate", equalTo(1.0F))
                .body("uncalibratedCoefficients", hasItem("No authorized operational historical dataset loaded."));
    }

    @Test
    void campaignAggregatesExpectedActionsAcrossScenarioSet() {
        String campaignId = given()
                .contentType("application/json")
                .body("{\"scenarioIds\":[\"pirep-safety-override\",\"blocked-no-viable-reroute\",\"viable-reroute-residual-risk\"],\"actor\":\"campaign\"}")
                .when()
                .post("/api/simulations/campaign")
                .then()
                .statusCode(200)
                .body("scenarioCount", equalTo(3))
                .body("passedScenarioCount", equalTo(3))
                .body("runs.find { it.scenarioId == 'pirep-safety-override' }.finalAction", equalTo("CAUTION"))
                .body("runs.find { it.scenarioId == 'blocked-no-viable-reroute' }.finalAction", equalTo("BLOCKED"))
                .body("aggregateKpis.replayVerificationPassRate", equalTo(1.0F))
                .extract()
                .path("id");

        given()
                .when()
                .get("/api/simulations/campaigns/" + campaignId + "/report")
                .then()
                .statusCode(200)
                .body("scenarioCount", equalTo(3))
                .body("diagnostics[0]", equalTo("Campaign uses local deterministic scenarios and fixture-backed scoring."));

        given()
                .when()
                .get("/api/simulations/campaigns/" + campaignId + "/dossier")
                .then()
                .statusCode(200)
                .body("campaignId", equalTo(campaignId))
                .body("markdown", notNullValue())
                .body("nonCertificationWarnings", hasItem("Not an FAA-qualified FSTD."));
    }
}
