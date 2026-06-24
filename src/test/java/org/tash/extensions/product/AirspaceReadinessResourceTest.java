package org.tash.extensions.product;

import io.quarkus.test.junit.QuarkusTest;
import org.junit.jupiter.api.Test;

import static io.restassured.RestAssured.given;
import static org.hamcrest.Matchers.containsInAnyOrder;
import static org.hamcrest.Matchers.containsString;
import static org.hamcrest.Matchers.equalTo;
import static org.hamcrest.Matchers.greaterThanOrEqualTo;
import static org.hamcrest.Matchers.hasItem;
import static org.hamcrest.Matchers.notNullValue;

@QuarkusTest
class AirspaceReadinessResourceTest {
    @Test
    void gapRegistryAndReleaseGatesExposeHonestProductionReadiness() {
        given()
                .when()
                .get("/api/gaps")
                .then()
                .statusCode(200)
                .body("id", hasItem("live-feeds"))
                .body("id", hasItem("calibration"))
                .body("id", hasItem("safety-claims"))
                .body("find { it.id == 'safety-claims' }.certificationClaimAllowed", equalTo(false))
                .body("find { it.id == 'live-feeds' }.externallyBlocked", equalTo(true))
                .body("find { it.id == 'notam' }.summary", containsString("distinct from CARF/ALTRV"));

        given()
                .when()
                .get("/api/gaps/release-gates")
                .then()
                .statusCode(200)
                .body("id", containsInAnyOrder("prototype-ready", "public-review-ready", "integration-ready", "operational-evaluation-ready"))
                .body("find { it.id == 'integration-ready' }.passed", equalTo(true))
                .body("find { it.id == 'operational-evaluation-ready' }.passed", equalTo(false))
                .body("find { it.id == 'operational-evaluation-ready' }.blockingGapIds", hasItem("calibration"))
                .body("find { it.id == 'operational-evaluation-ready' }.excludedClaims", hasItem("certified cockpit/dispatch system"));
    }

    @Test
    void providersExposeModesFreshnessAndDisabledAuthoritativeAdapters() {
        given()
                .when()
                .get("/api/providers/status")
                .then()
                .statusCode(200)
                .body("find { it.id == 'awc-public-weather' }.sourceMode", equalTo("PUBLIC_API"))
                .body("find { it.id == 'awc-public-weather' }.egressPolicy", equalTo("CONFIG_GATED"))
                .body("find { it.id == 'faa-swim' }.enabled", equalTo(false))
                .body("find { it.id == 'faa-swim' }.credentialRequirement", containsString("REQUIRED"))
                .body("find { it.id == 'faa-fns-nms' }.authoritative", equalTo(true))
                .body("find { it.id == 'nadin-wmscr-kvm' }.liveOperationalUseAllowed", equalTo(false));

        given()
                .contentType("application/json")
                .body("{\"products\":[\"metar\"],\"hoursBeforeNow\":1,\"maxResults\":5}")
                .when()
                .post("/api/providers/weather/poll")
                .then()
                .statusCode(200)
                .body("id", equalTo("awc-public-weather"))
                .body("enabled", equalTo(false))
                .body("freshness.status", equalTo("DISABLED"))
                .body("diagnostics.size()", greaterThanOrEqualTo(1));
    }

    @Test
    void calibrationSafetyAndCoordinationSurfacesStayHumanReviewed() {
        String runId = given()
                .contentType("application/json")
                .body("{\"datasetId\":\"fixture-low-vis-weather\",\"includeSyntheticScale\":true,\"actor\":\"tester\"}")
                .when()
                .post("/api/calibration/run")
                .then()
                .statusCode(200)
                .body("accepted", equalTo(true))
                .body("datasetId", equalTo("fixture-low-vis-weather"))
                .body("routeImpactReport.calibrationVersion", equalTo("fixture-calibration-2026-06-12"))
                .body("routeImpactReport.uncalibratedCoefficientCount", greaterThanOrEqualTo(1))
                .body("diagnostics[0]", containsString("local fixtures"))
                .extract()
                .path("id");

        given()
                .when()
                .get("/api/calibration/reports")
                .then()
                .statusCode(200)
                .body("find { it.id == '" + runId + "' }.routeImpactReport.datasetId", equalTo("fixture-low-vis-weather"));

        given()
                .when()
                .get("/api/safety/dossier")
                .then()
                .statusCode(200)
                .body("certificationClaimAllowed", equalTo(false))
                .body("summary", containsString("does not replace certified FAA"))
                .body("rejectedOverclaims", hasItem("automatic official message transmission"))
                .body("humanReviewCheckpoints", hasItem("Coordination drafts require operator approval."));

        given()
                .contentType("application/json")
                .body("{\"actor\":\"supervisor\",\"note\":\"reviewed\"}")
                .when()
                .post("/api/coordination/draft-123/approve")
                .then()
                .statusCode(200)
                .body("draftId", equalTo("draft-123"))
                .body("humanApproved", equalTo(true))
                .body("externalSendPerformed", equalTo(false));

        given()
                .contentType("application/json")
                .body("{\"actor\":\"supervisor\",\"deliveryChannel\":\"PHONE\",\"externalReceiptId\":\"ops-log-77\"}")
                .when()
                .post("/api/coordination/draft-123/mark-delivered")
                .then()
                .statusCode(200)
                .body("status", equalTo("DELIVERED_BY_OPERATOR"))
                .body("externalReceiptId", equalTo("ops-log-77"))
                .body("externalSendPerformed", equalTo(false));
    }

    @Test
    void collaborativeDecisionWorkflowExposesCopProposalApprovalsAndReceipts() {
        given()
                .when()
                .get("/api/collaboration/common-operating-picture")
                .then()
                .statusCode(200)
                .body("id", equalTo("cop-local-airspace"))
                .body("sourceMode", equalTo("LOCAL_FIXTURE_AND_PROVIDER_STATUS"))
                .body("participants.find { it.participantId == 'FAA-ATCSCC' }.canApprove", equalTo(true))
                .body("diagnostics", hasItem("No official FAA CDM, SWIM, NADIN, WMSCR, or KVM state was synchronized."));

        String proposalId = given()
                .contentType("application/json")
                .body("""
                        {
                          "missionId":"mission-cdm",
                          "reservationId":"reservation-cdm",
                          "hazardOrDecisionId":"decision-cdm",
                          "proposalType":"REROUTE_COORDINATION",
                          "recommendedAction":"REROUTE",
                          "summary":"Reroute around severe convection.",
                          "rationale":"Route impact crossed stakeholder boundaries.",
                          "actor":"planner",
                          "role":"PLANNER",
                          "sourceRefs":["WEATHER:SIGMET-CDM","TMI:AFP-CDM"],
                          "recipientParticipantIds":["FAA-ATCSCC","AIRLINE-DISPATCH"]
                        }
                        """)
                .when()
                .post("/api/collaboration/proposals")
                .then()
                .statusCode(200)
                .body("state", equalTo("PROPOSED"))
                .body("humanApprovalRequired", equalTo(true))
                .body("sourceRefs", hasItem("WEATHER:SIGMET-CDM"))
                .body("recipientParticipantIds", hasItem("AIRLINE-DISPATCH"))
                .extract()
                .path("id");

        given()
                .contentType("application/json")
                .body("{\"actor\":\"dispatcher\",\"role\":\"AIRLINE_OPERATOR\",\"note\":\"Accept reroute if ATCSCC approves.\"}")
                .when()
                .post("/api/collaboration/proposals/" + proposalId + "/comment")
                .then()
                .statusCode(200)
                .body("comments.size()", equalTo(1))
                .body("comments[0].role", equalTo("AIRLINE_OPERATOR"));

        given()
                .contentType("application/json")
                .body("{\"actor\":\"dispatcher\",\"role\":\"AIRLINE_OPERATOR\",\"note\":\"Operator accepts the proposed route option.\"}")
                .when()
                .post("/api/collaboration/proposals/" + proposalId + "/accept")
                .then()
                .statusCode(200)
                .body("state", equalTo("ACCEPTED"))
                .body("approvals[0].humanApproved", equalTo(false));

        given()
                .contentType("application/json")
                .body("{\"actor\":\"tmu-supervisor\",\"role\":\"TRAFFIC_MANAGER\",\"note\":\"Approved for local coordination.\"}")
                .when()
                .post("/api/collaboration/proposals/" + proposalId + "/approve")
                .then()
                .statusCode(200)
                .body("state", equalTo("APPROVED_LOCAL"))
                .body("approvals.find { it.status == 'APPROVED_LOCAL' }.humanApproved", equalTo(true));

        given()
                .contentType("application/json")
                .body("{\"actor\":\"tmu-supervisor\",\"deliveryChannel\":\"TELECON\",\"externalReceiptId\":\"telecon-log-42\",\"recipient\":\"FAA-ATCSCC\"}")
                .when()
                .post("/api/collaboration/proposals/" + proposalId + "/deliver")
                .then()
                .statusCode(200)
                .body("state", equalTo("DELIVERED_BY_OPERATOR"))
                .body("deliveryReceipts[0].externalReceiptId", equalTo("telecon-log-42"))
                .body("deliveryReceipts[0].externalSendPerformed", equalTo(false));

        given()
                .when()
                .get("/api/collaboration/common-operating-picture")
                .then()
                .statusCode(200)
                .body("proposals.find { it.id == '" + proposalId + "' }.state", equalTo("DELIVERED_BY_OPERATOR"))
                .body("deliveredReceiptCount", greaterThanOrEqualTo(1));
    }
}
