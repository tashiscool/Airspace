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
    void weatherPatternEndpointsExposeOfflineSafeLiveStatusPatternsEventsAndRouteSampling() {
        String artifactId = given()
                .contentType("application/json")
                .body("{\"sourceId\":\"awc-mock\",\"type\":\"WEATHER\",\"rawPayload\":\"SIGMET CONV SEV VALID 201200/201800 3000N15000W 3000N14900W 3100N14900W FL240-260 CONF 90\"}")
                .when()
                .post("/api/feed/ingest")
                .then()
                .statusCode(200)
                .extract()
                .path("results[0].envelope.id");

        given()
                .when()
                .get("/api/weather/live/status")
                .then()
                .statusCode(200)
                .body("enabled", equalTo(false))
                .body("baseUrl", notNullValue());

        given()
                .contentType("application/json")
                .body("{\"products\":[\"metar\"],\"hoursBeforeNow\":1,\"maxResults\":5}")
                .when()
                .post("/api/weather/live/poll")
                .then()
                .statusCode(200)
                .body("accepted", equalTo(false))
                .body("diagnostics[0]", org.hamcrest.Matchers.containsString("disabled"));

        given()
                .when()
                .get("/api/weather/patterns")
                .then()
                .statusCode(200)
                .body("size()", greaterThanOrEqualTo(1))
                .body("find { it.id != null }.sourceRefs.size()", greaterThanOrEqualTo(1));

        given()
                .when()
                .get("/api/weather/events")
                .then()
                .statusCode(200)
                .body("size()", greaterThanOrEqualTo(1))
                .body("find { it.eventType == 'CONVECTION' }.productCount", greaterThanOrEqualTo(1));

        given()
                .when()
                .get("/api/weather/patterns/features")
                .then()
                .statusCode(200)
                .body("type", equalTo("FeatureCollection"))
                .body("features.find { it.properties.patternType == 'CONVECTION' }.properties.sourceRefs.size()", greaterThanOrEqualTo(1));

        given()
                .contentType("application/json")
                .body("{\"route\":[[30,-150.2,25000],[30.5,-148.8,25000]],\"lowerAltitudeFeet\":24000,\"upperAltitudeFeet\":26000,\"corridorNauticalMiles\":50}")
                .when()
                .post("/api/weather/route-sample")
                .then()
                .statusCode(200)
                .body("size()", greaterThanOrEqualTo(1))
                .body("[0].geometryOverlap", equalTo(true))
                .body("[0].sourceRefs.size()", greaterThanOrEqualTo(1));

        given()
                .when()
                .get("/api/feed/artifacts/" + artifactId)
                .then()
                .statusCode(200)
                .body("sourceId", equalTo("awc-mock"));
    }

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

        given()
                .contentType("application/json")
                .body("{\"missionId\":\"" + missionId + "\",\"family\":\"SIGMET\",\"direction\":\"INBOUND\",\"subject\":\"SIGMET ECHO\",\"rawText\":\"SIGMET CONV SEV 3000N15000W 3000N14900W 3100N14900W FL240-260 CONF 90\",\"actor\":\"planner\"}")
                .when()
                .post("/api/messages/send")
                .then()
                .statusCode(200)
                .body("family", equalTo("SIGMET"));

        given()
                .when()
                .get("/api/missions/" + missionId + "/weather-verdict")
                .then()
                .statusCode(200)
                .body("missionId", equalTo(missionId))
                .body("sourceCount", greaterThanOrEqualTo(1))
                .body("sources[0].family", equalTo("SIGMET"));

        given()
                .when()
                .get("/api/missions/" + missionId + "/weather-changes?limit=5")
                .then()
                .statusCode(200)
                .body("size()", greaterThanOrEqualTo(1))
                .body("[0].family", equalTo("SIGMET"));

        given()
                .when()
                .get("/api/weather/affected-missions?limit=5")
                .then()
                .statusCode(200)
                .body("size()", greaterThanOrEqualTo(1))
                .body("[0].missionId", equalTo(missionId))
                .body("[0].ageSeconds", greaterThanOrEqualTo(0))
                .body("[0].guidanceLatencySeconds", greaterThanOrEqualTo(0));

        given()
                .when()
                .get("/api/missions/" + missionId + "/route-impact?reservationId=" + reservationId)
                .then()
                .statusCode(200)
                .body("missionId", equalTo(missionId))
                .body("action", notNullValue())
                .body("sourceRefs.size()", greaterThanOrEqualTo(1));

        given()
                .contentType("application/json")
                .body("{\"recencyMinutes\":60,\"corridorNauticalMiles\":50}")
                .when()
                .post("/api/missions/" + missionId + "/pireps/relevant")
                .then()
                .statusCode(200)
                .body("missionId", equalTo(missionId));

        given()
                .contentType("application/json")
                .body("{\"reservationId\":\"" + reservationId + "\",\"actor\":\"planner\"}")
                .when()
                .post("/api/missions/" + missionId + "/coordinate-weather")
                .then()
                .statusCode(200)
                .body("family", equalTo("USNS"))
                .body("rawText", org.hamcrest.Matchers.containsString("WEATHER COORDINATION"));

        given()
                .when()
                .get("/api/missions/" + missionId + "/pilot-brief")
                .then()
                .statusCode(200)
                .body("missionId", equalTo(missionId))
                .body("printableText", org.hamcrest.Matchers.containsString("AIRSPACE PILOT BRIEF"));

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

        String notamFeedArtifactId = given()
                .contentType("application/json")
                .body("{\"sourceId\":\"notam-api\",\"type\":\"USNS\",\"rawPayload\":"
                        + json(envelope("!DCA LDN RWY 10/28 CLSD 1012211200-1012211300\\n"
                        .replace("\\n", "\n") + "(A0001/26 NOTAMN Q) /QRTCA/IV/BO/W/000/180/3000N15000W005 A) KZNY B) 2601011200 C) PERM E) TEST)\\n"
                        .replace("\\n", "\n") + "(A0002/26 NOTAMC A) KZNY E) CANCEL TEST)\\n"
                        .replace("\\n", "\n") + "(SVC RQ DOM RQN KJFK HIST)\\n"
                        .replace("\\n", "\n") + "(SVC TBL ROUTING UPDATE)\\n"
                        .replace("\\n", "\n") + "GENOT RWA TEST ADMIN MESSAGE")) + "}")
                .when()
                .post("/api/feed/ingest")
                .then()
                .statusCode(200)
                .body("results.size()", equalTo(1))
                .extract()
                .path("results[0].envelope.id");

        given()
                .when()
                .get("/api/feed/artifacts/" + notamFeedArtifactId + "/transactions")
                .then()
                .statusCode(200)
                .body("find { it.notamType == 'NOTAMN' }.notamQCode", equalTo("QRTCA"))
                .body("find { it.notamType == 'NOTAMN' }.notamHasGeometry", equalTo(true))
                .body("find { it.notamType == 'NOTAMN' }.notamPermanentEnd", equalTo(true))
                .body("find { it.type == 'DOMESTIC' }.domesticNotamKeyword", equalTo("RWY"))
                .body("find { it.type == 'DOMESTIC' }.domesticNotamReducerRuleId", equalTo("DOM2.SURFACE.CLOSED"))
                .body("find { it.type == 'DOMESTIC' }.domesticNotamQ23", equalTo("LC"))
                .body("find { it.notamType == 'NOTAMC' }.notamHasGeometry", equalTo(false))
                .body("find { it.notamType == 'NOTAMC' }.warnings[0]", org.hamcrest.Matchers.containsString("No Q field"))
                .body("find { it.type == 'SERVICE_REQUEST' }.serviceCommandOperation", equalTo("RQN"))
                .body("find { it.type == 'SERVICE_REQUEST' }.serviceCommandLocation", equalTo("KJFK"))
                .body("find { it.type == 'SERVICE_TABLE' }.serviceCommandOperation", equalTo("UPDATE"))
                .body("find { it.type == 'GENOT' }.familySemantic", equalTo("genot-admin-message"))
                .body("find { it.type == 'GENOT' }.familyGenotSeries", equalTo("RWA"));

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

        given()
                .queryParam("q", "DOM2.SURFACE.CLOSED")
                .when()
                .get("/api/search")
                .then()
                .statusCode(200)
                .body("find { it.type == 'feed-transaction' }.route", equalTo("/feed/" + notamFeedArtifactId));

        given()
                .queryParam("q", "QRTCA")
                .when()
                .get("/api/search")
                .then()
                .statusCode(200)
                .body("find { it.type == 'feed-transaction' }.snippet", org.hamcrest.Matchers.containsString("GEOMETRY"));

        given()
                .queryParam("q", "RQN")
                .when()
                .get("/api/search")
                .then()
                .statusCode(200)
                .body("find { it.type == 'feed-transaction' }.snippet", org.hamcrest.Matchers.containsString("REQUEST"));

        given()
                .queryParam("q", "genot-admin-message")
                .when()
                .get("/api/search")
                .then()
                .statusCode(200)
                .body("find { it.type == 'feed-transaction' }.snippet", org.hamcrest.Matchers.containsString("GENOT"));

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

        String agentRunId = given()
                .contentType("application/json")
                .body("{\"agentType\":\"ALL\",\"missionId\":\"" + missionId + "\",\"reservationId\":\"" + reservationId + "\",\"decisionId\":\"" + decisionId + "\",\"actor\":\"planner\"}")
                .when()
                .post("/api/agents/run")
                .then()
                .statusCode(200)
                .body("accepted", equalTo(true))
                .body("operatingLoop.find { it.stage == 'OBSERVE' }.status", equalTo("COMPLETE"))
                .body("auditEnvelope.inputHash", notNullValue())
                .body("findings.size()", greaterThanOrEqualTo(1))
                .body("recommendations.find { it.humanApprovalRequired == true }.id", notNullValue())
                .extract()
                .path("id");

        given()
                .when()
                .get("/api/agents/runs/" + agentRunId)
                .then()
                .statusCode(200)
                .body("id", equalTo(agentRunId))
                .body("reasoningEnvelope.draftHash", notNullValue());

        given()
                .when()
                .get("/api/agents/status")
                .then()
                .statusCode(200)
                .body("mode", notNullValue())
                .body("runCount", greaterThanOrEqualTo(1));

        Map<String, Number> agentMetrics = given()
                .when()
                .get("/api/agents/metrics")
                .then()
                .statusCode(200)
                .extract()
                .as(Map.class);
        assertTrue(agentMetrics.get("agentic.runs").doubleValue() >= 1.0, agentMetrics.toString());

        given()
                .contentType("application/json")
                .body("{\"missionId\":\"" + missionId + "\",\"actor\":\"planner\"}")
                .when()
                .post("/api/agents/coordination-draft")
                .then()
                .statusCode(200)
                .body("accepted", equalTo(true))
                .body("summary", org.hamcrest.Matchers.containsString("send is blocked"));

        given()
                .contentType("application/json")
                .body("{\"previousDecisionId\":\"" + decisionId + "\",\"decisionId\":\"" + decisionId + "\",\"missionId\":\"" + missionId + "\"}")
                .when()
                .post("/api/agents/delta")
                .then()
                .statusCode(200);

        given()
                .contentType("application/json")
                .body("{\"scenarioType\":\"VIABLE_REROUTE\",\"missionNumber\":\"API-NXGEN\",\"includeMalformedInputs\":true}")
                .when()
                .post("/api/agents/scenario/generate")
                .then()
                .statusCode(200)
                .body("scenarioType", equalTo("VIABLE_REROUTE"))
                .body("missionNumber", equalTo("API-NXGEN"))
                .body("expectedSummary.malformedRetained", equalTo("true"));

        String agentTaskId = given()
                .when()
                .get("/api/agents/tasks?limit=5&routeContains=" + missionId)
                .then()
                .statusCode(200)
                .body("size()", greaterThanOrEqualTo(1))
                .extract()
                .path("[0].id");

        given()
                .contentType("application/json")
                .body("{\"status\":\"ACKNOWLEDGED\",\"actor\":\"planner\",\"note\":\"reviewed from API test\"}")
                .when()
                .post("/api/agents/tasks/" + agentTaskId + "/transition")
                .then()
                .statusCode(200)
                .body("status", equalTo("ACKNOWLEDGED"));

        given()
                .when()
                .get("/api/agents/tasks/" + agentTaskId)
                .then()
                .statusCode(200)
                .body("id", equalTo(agentTaskId))
                .body("status", equalTo("ACKNOWLEDGED"));

        given()
                .when()
                .get("/api/agents/runs?limit=5&missionId=" + missionId + "&agentType=ALL&accepted=true")
                .then()
                .statusCode(200)
                .body("size()", greaterThanOrEqualTo(1))
                .body("[0].missionId", equalTo(missionId));

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
        given().when().get("/api/config").then().statusCode(200)
                .body("mapStack", equalTo("OpenLayers"))
                .body("agenticStore", notNullValue());
        Map<String, Number> metrics = given().when().get("/api/metrics").then().statusCode(200).extract().as(Map.class);
        assertTrue(metrics.get("product.missions").doubleValue() >= 1.0);
        assertTrue(metrics.get("agentic.runs").doubleValue() >= 1.0);
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
        StringBuilder escaped = new StringBuilder("\"");
        for (int i = 0; i < text.length(); i++) {
            char ch = text.charAt(i);
            switch (ch) {
                case '\\':
                    escaped.append("\\\\");
                    break;
                case '"':
                    escaped.append("\\\"");
                    break;
                case '\n':
                    escaped.append("\\n");
                    break;
                case '\r':
                    escaped.append("\\r");
                    break;
                case '\t':
                    escaped.append("\\t");
                    break;
                default:
                    if (ch < 0x20) {
                        escaped.append(String.format("\\u%04x", (int) ch));
                    } else {
                        escaped.append(ch);
                    }
                    break;
            }
        }
        return escaped.append("\"").toString();
    }

    private String envelope(String body) {
        return "01GGNC07GP\n"
                + "CNS000 300334\n"
                + "GG KDZZNAXX\n"
                + "300334 KGPS\n"
                + org.tash.extensions.messaging.MessageControlCharacters.STX + body
                + org.tash.extensions.messaging.MessageControlCharacters.VT
                + org.tash.extensions.messaging.MessageControlCharacters.ETX;
    }
}
