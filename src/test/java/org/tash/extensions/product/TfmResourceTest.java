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
class TfmResourceTest {
    @Test
    void tfmBoardExposesCommandCenterDemandCapacityAndHumanReview() {
        given()
                .contentType("application/json")
                .body("""
                        {
                          "demandCapacityConfig": {
                            "id": "tfm-api",
                            "flightCount": 600,
                            "airportCount": 8,
                            "sectorCount": 12,
                            "durationMinutes": 90,
                            "tickIntervalMinutes": 10,
                            "randomSeed": 42,
                            "demandSpikeFactor": 1.6,
                            "capacityReductionFactor": 0.62
                          },
                          "maxAirportRows": 6,
                          "maxSectorRows": 6,
                          "maxConstraintRows": 10,
                          "maxProposalRows": 10
                        }
                        """)
                .when()
                .post("/api/tfm/board")
                .then()
                .statusCode(200)
                .body("id", equalTo("tfm-board-national-demand-tfm-api"))
                .body("boardMode", equalTo("LOCAL_COMMAND_CENTER_PREVIEW"))
                .body("sourceMode", equalTo("LOCAL_SYNTHETIC_NAS_SCALE"))
                .body("authorizationMode", equalTo("LOCAL_FIXTURE_ONLY"))
                .body("selectedSnapshot", notNullValue())
                .body("airportDemand.size()", greaterThanOrEqualTo(1))
                .body("airportDemand[0].airportId", notNullValue())
                .body("airportDemand[0].demandCapacityRatio", greaterThanOrEqualTo(0.0F))
                .body("sectorLoad.size()", greaterThanOrEqualTo(1))
                .body("sectorLoad[0].sectorId", notNullValue())
                .body("activeConstraints.size()", greaterThanOrEqualTo(1))
                .body("proposedTmis.size()", greaterThanOrEqualTo(1))
                .body("proposedTmis[0].requiresHumanApproval", equalTo(true))
                .body("impactTotals.flightCount", equalTo(600))
                .body("impactTotals.humanApprovalRequired", equalTo(true))
                .body("impactTotals.proposedTmiCount", greaterThanOrEqualTo(1))
                .body("impactTotals.commonOperatingPictureStatus", equalTo("SHARED_REVIEW_READY"))
                .body("humanFactorsNotes", hasItem(containsString("least restrictive TMI")))
                .body("assumptions", hasItem(containsString("not connected to live FMDS")))
                .body("sourceRefs.size()", greaterThanOrEqualTo(1));
    }

    @Test
    void defaultTfmBoardIsAvailableForWorkbenchStartup() {
        given()
                .when()
                .get("/api/tfm/board")
                .then()
                .statusCode(200)
                .body("boardMode", equalTo("LOCAL_COMMAND_CENTER_PREVIEW"))
                .body("impactTotals.flightCount", greaterThanOrEqualTo(1))
                .body("airportDemand.size()", greaterThanOrEqualTo(1))
                .body("sectorLoad.size()", greaterThanOrEqualTo(1));
    }
}
