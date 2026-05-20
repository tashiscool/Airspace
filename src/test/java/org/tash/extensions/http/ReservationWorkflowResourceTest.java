package org.tash.extensions.http;

import io.quarkus.test.junit.QuarkusTest;
import org.junit.jupiter.api.Test;

import static io.restassured.RestAssured.given;
import static org.hamcrest.Matchers.equalTo;
import static org.hamcrest.Matchers.notNullValue;

@QuarkusTest
class ReservationWorkflowResourceTest {
    @Test
    void analyzeEndpointReturnsAcceptedCarfResult() {
        given()
                .contentType("application/json")
                .body("{\"rawText\":" + json(validMessage()) + "}")
                .when()
                .post("/api/carf/analyze")
                .then()
                .statusCode(200)
                .body("accepted", equalTo(true))
                .body("reservations.size()", equalTo(1));
    }

    @Test
    void reservationWorkflowEndpointsCreateValidateAndExportFeatures() {
        String id = given()
                .contentType("application/json")
                .body("{\"actor\":\"planner\",\"rawText\":" + json(validMessage()) + "}")
                .when()
                .post("/api/reservations/drafts")
                .then()
                .statusCode(200)
                .body("accepted", equalTo(true))
                .body("record.id", notNullValue())
                .extract()
                .path("record.id");

        given()
                .contentType("application/json")
                .body("{\"actor\":\"planner\"}")
                .when()
                .post("/api/reservations/" + id + "/validate")
                .then()
                .statusCode(200)
                .body("accepted", equalTo(true))
                .body("record.state", equalTo("VALIDATED"));

        given()
                .when()
                .get("/api/reservations/" + id + "/features")
                .then()
                .statusCode(200)
                .body("type", equalTo("FeatureCollection"))
                .body("features.size()", equalTo(1));
    }

    @Test
    void visualizeEndpointReturnsGenericGeoJsonFeatureCollection() {
        given()
                .contentType("application/json")
                .body("{}")
                .when()
                .post("/api/visualize")
                .then()
                .statusCode(200)
                .body("type", equalTo("FeatureCollection"))
                .body("features.size()", equalTo(0));
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
