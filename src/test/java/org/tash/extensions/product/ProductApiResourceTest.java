package org.tash.extensions.product;

import io.quarkus.test.junit.QuarkusTest;
import io.restassured.response.Response;
import org.junit.jupiter.api.Test;

import java.util.Map;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static io.restassured.RestAssured.given;
import static org.hamcrest.Matchers.equalTo;
import static org.hamcrest.Matchers.greaterThanOrEqualTo;
import static org.hamcrest.Matchers.notNullValue;

@QuarkusTest
class ProductApiResourceTest {
    @Test
    void authMissionMessageFeedAndDecisionEndpointsWorkTogether() {
        String token = given()
                .contentType("application/json")
                .body("{\"username\":\"planner\",\"password\":\"planner\"}")
                .when()
                .post("/api/auth/login")
                .then()
                .statusCode(200)
                .body("accepted", equalTo(true))
                .body("user.roles.size()", greaterThanOrEqualTo(1))
                .extract()
                .path("token");

        given()
                .header("Authorization", "Bearer " + token)
                .when()
                .get("/api/auth/me")
                .then()
                .statusCode(200)
                .body("username", equalTo("planner"));

        String missionId = given()
                .contentType("application/json")
                .body("{\"missionNumber\":\"M-100\",\"title\":\"Oceanic test\",\"actor\":\"planner\"}")
                .when()
                .post("/api/missions")
                .then()
                .statusCode(200)
                .body("id", notNullValue())
                .body("missionNumber", equalTo("M-100"))
                .extract()
                .path("id");

        given()
                .contentType("application/json")
                .body("{\"actor\":\"planner\"}")
                .when()
                .post("/api/missions/" + missionId + "/lock")
                .then()
                .statusCode(200)
                .body("lockedBy", equalTo("planner"));

        String reservationId = given()
                .contentType("application/json")
                .body("{\"actor\":\"planner\",\"rawText\":" + json(validMessage()) + "}")
                .when()
                .post("/api/missions/" + missionId + "/reservations")
                .then()
                .statusCode(200)
                .body("accepted", equalTo(true))
                .extract()
                .path("record.id");

        given()
                .contentType("application/json")
                .body("{\"actor\":\"planner\"}")
                .when()
                .post("/api/reservations/" + reservationId + "/parse")
                .then()
                .statusCode(200)
                .body("accepted", equalTo(true));

        given()
                .contentType("application/json")
                .body("{\"actor\":\"planner\"}")
                .when()
                .post("/api/reservations/" + reservationId + "/deconflict")
                .then()
                .statusCode(200)
                .body("accepted", equalTo(true));

        given()
                .when()
                .get("/api/missions/" + missionId)
                .then()
                .statusCode(200)
                .body("reservations.find { it.id == '" + reservationId + "' }.state", equalTo("VALIDATED"));

        String supplementId = given()
                .contentType("application/json")
                .body("{\"kind\":\"NOTAM\",\"status\":\"DRAFT\",\"title\":\"Domestic NOTAM\",\"text\":\"RWY CLSD\",\"actor\":\"planner\"}")
                .when()
                .post("/api/reservations/" + reservationId + "/supplements")
                .then()
                .statusCode(200)
                .body("reservationId", equalTo(reservationId))
                .body("kind", equalTo("NOTAM"))
                .extract()
                .path("id");

        given()
                .contentType("application/json")
                .body("{\"status\":\"APPROVED\",\"actor\":\"supervisor\",\"note\":\"approved for ops\"}")
                .when()
                .post("/api/reservations/" + reservationId + "/supplements/" + supplementId + "/transition")
                .then()
                .statusCode(200)
                .body("status", equalTo("APPROVED"));

        given()
                .when()
                .get("/api/reservations/" + reservationId + "/supplements")
                .then()
                .statusCode(200)
                .body("find { it.kind == 'NOTAM' }.title", equalTo("Domestic NOTAM"));

        String messageId = given()
                .contentType("application/json")
                .body("{\"missionId\":\"" + missionId + "\",\"family\":\"USNS\",\"direction\":\"OUTBOUND\",\"subject\":\"Coordination\",\"rawText\":\"SVC RQ\",\"actor\":\"planner\"}")
                .when()
                .post("/api/messages/send")
                .then()
                .statusCode(200)
                .body("status", equalTo("QUEUED"))
                .extract()
                .path("id");

        given()
                .when()
                .get("/api/messages/" + messageId)
                .then()
                .statusCode(200)
                .body("rawText", equalTo("SVC RQ"))
                .body("missionId", equalTo(missionId));

        given()
                .contentType("application/json")
                .body("{\"rawText\":\"ACK SVC RQ\",\"actor\":\"planner\"}")
                .when()
                .post("/api/messages/" + messageId + "/reply")
                .then()
                .statusCode(200)
                .body("subject", equalTo("RE: Coordination"));

        given()
                .contentType("application/json")
                .body("{\"actor\":\"planner\"}")
                .when()
                .post("/api/messages/" + messageId + "/forward")
                .then()
                .statusCode(200)
                .body("subject", equalTo("FWD: Coordination"));

        String feedArtifactId = given()
                .contentType("application/json")
                .body("{\"sourceId\":\"test\",\"type\":\"WEATHER\",\"rawPayload\":\"METAR KJFK 200000Z 18012KT 1/2SM TSRA BKN004\"}")
                .when()
                .post("/api/feed/ingest")
                .then()
                .statusCode(200)
                .body("results.size()", equalTo(1))
                .extract()
                .path("results[0].envelope.id");

        given()
                .when()
                .get("/api/feed/artifacts/" + feedArtifactId)
                .then()
                .statusCode(200)
                .body("rawPayload", equalTo("METAR KJFK 200000Z 18012KT 1/2SM TSRA BKN004"))
                .body("sourceId", equalTo("test"));

        given()
                .when()
                .get("/api/feed/artifacts/" + feedArtifactId + "/transactions")
                .then()
                .statusCode(200)
                .body("size()", greaterThanOrEqualTo(1))
                .body("[0].type", notNullValue());

        given()
                .queryParam("q", "Oceanic")
                .when()
                .get("/api/search")
                .then()
                .statusCode(200)
                .body("find { it.type == 'mission' }.route", equalTo("/missions/" + missionId));

        given()
                .queryParam("q", "METAR")
                .when()
                .get("/api/search")
                .then()
                .statusCode(200)
                .body("find { it.type == 'feed' }.route", equalTo("/feed/" + feedArtifactId));

        String decisionId = given()
                .contentType("application/json")
                .body("{\"decisionTime\":\"2026-05-20T00:00:00Z\",\"route\":[[30,-150.5,24000],[30,-148.5,24000]]}")
                .when()
                .post("/api/decisions/evaluate")
                .then()
                .statusCode(200)
                .body("id", notNullValue())
                .body("action", notNullValue())
                .body("resultJson", notNullValue())
                .body("auditJson", notNullValue())
                .body("replayJson", notNullValue())
                .extract()
                .path("id");

        given()
                .when()
                .get("/api/decisions/" + decisionId + "/features")
                .then()
                .statusCode(200)
                .body("type", equalTo("FeatureCollection"));

        Response replayResponse = given()
                .when()
                .post("/api/decisions/" + decisionId + "/replay");
        assertEquals(200, replayResponse.statusCode(), replayResponse.asString());
        assertTrue(replayResponse.jsonPath().getBoolean("accepted"), replayResponse.asString());

        given().when().get("/api/reference/navaids").then().statusCode(200).body("JFK.latitude", notNullValue());
        given().when().get("/api/reference/points").then().statusCode(200).body("find { it.identifier == 'JFK' }.pointType", equalTo("NAVAID"));
        given()
                .contentType("application/json")
                .body("{\"identifier\":\"LOCALFIX\",\"pointType\":\"FIX\",\"latitude\":38.5,\"longitude\":-77.1,\"altitudeFeet\":0,\"source\":\"test\"}")
                .when()
                .post("/api/reference/points")
                .then()
                .statusCode(200)
                .body("identifier", equalTo("LOCALFIX"))
                .body("pointType", equalTo("FIX"));
        given()
                .contentType("application/json")
                .body("{\"payload\":\"type,identifier,latitude,longitude,altitudeFeet,source,version\\nFIX,APIFIX,39.0,-76.0,0,api,v1\",\"actor\":\"admin\"}")
                .when()
                .post("/api/reference/import/preview")
                .then()
                .statusCode(200)
                .body("accepted", equalTo(true))
                .body("parsedCount", equalTo(1));
        given()
                .contentType("application/json")
                .body("{\"payload\":\"type,identifier,latitude,longitude,altitudeFeet,source,version\\nFIX,APIFIX,39.0,-76.0,0,api,v1\",\"actor\":\"admin\",\"apply\":true}")
                .when()
                .post("/api/reference/import/apply")
                .then()
                .statusCode(200)
                .body("accepted", equalTo(true))
                .body("appliedCount", equalTo(1));
        given().when().get("/api/config").then().statusCode(200).body("mapStack", equalTo("OpenLayers"));
        Map<String, Number> metrics = given().when().get("/api/metrics").then().statusCode(200).extract().as(Map.class);
        assertTrue(metrics.get("product.missions").doubleValue() >= 1.0);
        given().when().get("/api/history").then().statusCode(200).body("size()", greaterThanOrEqualTo(1));
    }

    @Test
    void unknownOperationalResourceReturnsOperatorFriendlyDiagnostic() {
        given()
                .when()
                .get("/api/messages/00000000-0000-0000-0000-000000000000")
                .then()
                .statusCode(404)
                .body("accepted", equalTo(false))
                .body("diagnostics[0]", equalTo("Unknown message: 00000000-0000-0000-0000-000000000000"));
    }

    private String validMessage() {
        return "A. TEST02 KZNY\n"
                + "B. 1F22/I\n"
                + "C. KZNY\n"
                + "D. FL240B260 3000N 15000W 0000 3000N 15100W 0100\n"
                + "E. TEST02\n"
                + "F. ETD TEST02 021200 MAR 2010 AVANA 021300\n"
                + "G. TAS: 300 KTAS\n";
    }

    private String json(String text) {
        return "\"" + text.replace("\\", "\\\\").replace("\"", "\\\"").replace("\n", "\\n") + "\"";
    }
}
