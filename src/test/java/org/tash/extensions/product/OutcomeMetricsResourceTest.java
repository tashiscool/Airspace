package org.tash.extensions.product;

import io.quarkus.test.junit.QuarkusTest;
import org.junit.jupiter.api.Test;

import static io.restassured.RestAssured.given;
import static org.hamcrest.Matchers.containsString;
import static org.hamcrest.Matchers.equalTo;
import static org.hamcrest.Matchers.greaterThanOrEqualTo;
import static org.hamcrest.Matchers.hasItem;
import static org.hamcrest.Matchers.notNullValue;

@QuarkusTest
class OutcomeMetricsResourceTest {
    @Test
    void defaultOutcomeMetricsExposeTfmBoardBenefitReadout() {
        given()
                .when()
                .get("/api/outcomes/metrics")
                .then()
                .statusCode(200)
                .body("id", notNullValue())
                .body("scope", equalTo("TFM_BOARD"))
                .body("sourceMode", equalTo("LOCAL_SYNTHETIC_NAS_SCALE"))
                .body("baselineDelayMinutes", greaterThanOrEqualTo(0.0F))
                .body("delayMinutesSaved", greaterThanOrEqualTo(0.0F))
                .body("metrics.id", hasItem("delay-minutes-saved"))
                .body("metrics.id", hasItem("fuel-impact"))
                .body("metrics.id", hasItem("reroute-miles"))
                .body("metrics.id", hasItem("sector-overload-avoided"))
                .body("metrics.id", hasItem("false-clear"))
                .body("metrics.id", hasItem("false-block"))
                .body("metrics.id", hasItem("source-ref-completeness"))
                .body("metrics.id", hasItem("operator-time-to-decision"))
                .body("assumptions", hasItem(containsString("not authoritative NAS post-event measurement")))
                .body("sourceRefs.size()", greaterThanOrEqualTo(1));
    }

    @Test
    void runBackedOutcomeMetricsIncludeSimulationSafetyAndRerouteKpis() {
        given()
                .contentType("application/json")
                .body("""
                        {
                          "runSimulation": true,
                          "scenarioId": "oceanic-altrv-convection",
                          "includeTfmBoard": true,
                          "demandCapacityConfig": {
                            "id": "outcomes-api",
                            "flightCount": 500,
                            "airportCount": 8,
                            "sectorCount": 12,
                            "durationMinutes": 60,
                            "tickIntervalMinutes": 10,
                            "randomSeed": 7,
                            "demandSpikeFactor": 1.5,
                            "capacityReductionFactor": 0.66
                          }
                        }
                        """)
                .when()
                .post("/api/outcomes/metrics")
                .then()
                .statusCode(200)
                .body("scope", equalTo("RUN"))
                .body("scenarioId", equalTo("oceanic-altrv-convection"))
                .body("runId", notNullValue())
                .body("baselineDelayMinutes", greaterThanOrEqualTo(0.0F))
                .body("mitigatedDelayMinutes", greaterThanOrEqualTo(0.0F))
                .body("delayMinutesSaved", greaterThanOrEqualTo(0.0F))
                .body("sourceRefCompletenessRate", greaterThanOrEqualTo(0.0F))
                .body("operatorTimeToDecisionSeconds", greaterThanOrEqualTo(0))
                .body("routeAlternativeCount", greaterThanOrEqualTo(1))
                .body("proposedTmiCount", greaterThanOrEqualTo(1))
                .body("metrics.find { it.id == 'source-ref-completeness' }.status", notNullValue())
                .body("diagnostics[0]", containsString("simulation run"))
                .body("sourceRefs.size()", greaterThanOrEqualTo(1));
    }
}
