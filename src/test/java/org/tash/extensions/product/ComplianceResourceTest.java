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
class ComplianceResourceTest {
    @Test
    void policyAndManifestAreTransparentAndDoNotEnableCloneTimeCollection() {
        given()
                .when()
                .get("/api/compliance/policy")
                .then()
                .statusCode(200)
                .body("licenseSpdx", equalTo("AGPL-3.0-or-later"))
                .body("hiddenTelemetryEnabled", equalTo(false))
                .body("cloneTimeCollectionEnabled", equalTo(false))
                .body("collectorMode", equalTo("EXPLICIT_OPT_IN_ATTESTATION"))
                .body("obligations", hasItem(containsString("Publish")))
                .body("prohibitedCollection", hasItem(containsString("No hidden telemetry")));

        given()
                .when()
                .get("/api/compliance/manifest")
                .then()
                .statusCode(200)
                .body("projectName", equalTo("Airspace"))
                .body("sourceDisclosureRequired", equalTo(true))
                .body("operationalConfigDisclosureRequired", equalTo(true))
                .body("secretRedactionRequired", equalTo(true))
                .body("hiddenTelemetryEnabled", equalTo(false))
                .body("cloneTimeCollectionEnabled", equalTo(false))
                .body("redactionRules", hasItem(containsString("Never publish secrets")))
                .body("publicRegistryUrl", notNullValue());
    }

    @Test
    void attestationCollectorRequiresPublicSourceConfigAndAcknowledgement() {
        given()
                .contentType("application/json")
                .body("""
                        {
                          "organization":"Example Ops Lab",
                          "purpose":"Internal evaluation",
                          "useType":"EVALUATION",
                          "sourceCodeUrl":"https://example.invalid/airspace-fork",
                          "operationalConfigUrl":"https://example.invalid/airspace-config",
                          "publicDisclosureUrl":"https://example.invalid/disclosure",
                          "acknowledgesAgpl":true,
                          "acknowledgesPublicUseAndAiPolicy":true,
                          "sourcePublished":true,
                          "operationalConfigPublished":true,
                          "secretsRedacted":true,
                          "actor":"compliance-officer"
                        }
                        """)
                .when()
                .post("/api/compliance/attestations")
                .then()
                .statusCode(200)
                .body("accepted", equalTo(true))
                .body("status", equalTo("ACCEPTED_PUBLIC_DISCLOSURE"))
                .body("sourceCodeUrl", equalTo("https://example.invalid/airspace-fork"))
                .body("operationalConfigUrl", equalTo("https://example.invalid/airspace-config"))
                .body("attestationHash", notNullValue());

        given()
                .contentType("application/json")
                .body("""
                        {
                          "organization":"Closed Clone",
                          "purpose":"Private deployment",
                          "useType":"DEPLOYMENT",
                          "acknowledgesAgpl":false,
                          "sourcePublished":false,
                          "operationalConfigPublished":false,
                          "secretsRedacted":false
                        }
                        """)
                .when()
                .post("/api/compliance/attestations")
                .then()
                .statusCode(200)
                .body("accepted", equalTo(false))
                .body("status", equalTo("INCOMPLETE_DISCLOSURE"))
                .body("diagnostics", hasItem(containsString("source code URL")))
                .body("diagnostics", hasItem(containsString("operational configuration")));

        given()
                .when()
                .get("/api/compliance/attestations")
                .then()
                .statusCode(200)
                .body("size()", greaterThanOrEqualTo(2));
    }
}
