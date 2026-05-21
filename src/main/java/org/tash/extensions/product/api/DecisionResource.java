package org.tash.extensions.product.api;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import jakarta.inject.Inject;
import jakarta.ws.rs.Consumes;
import jakarta.ws.rs.GET;
import jakarta.ws.rs.POST;
import jakarta.ws.rs.Path;
import jakarta.ws.rs.PathParam;
import jakarta.ws.rs.Produces;
import jakarta.ws.rs.core.MediaType;
import org.tash.extensions.engine.HmacAuditSigner;
import org.tash.extensions.engine.OperationalDecisionReplayBundle;
import org.tash.extensions.engine.OperationalDecisionResult;
import org.tash.extensions.engine.ReplayVerificationResult;
import org.tash.extensions.product.application.AirspaceProductService;
import org.tash.extensions.product.dto.ProductDtos;
import org.tash.extensions.weather.decision.RouteBlockagePrediction;
import org.tash.extensions.weather.decision.RouteWeatherAdvisory;
import org.tash.extensions.weather.decision.WeatherDecisionAction;
import org.tash.extensions.weather.decision.WeatherDecisionSeverity;
import org.tash.extensions.weather.decision.WeatherRecommendedAction;
import org.tash.extensions.visualization.AirspaceFeatureCollection;
import org.tash.extensions.visualization.AirspaceVisualizationService;
import org.tash.extensions.visualization.VisualizationRequest;

import java.util.ArrayList;
import java.util.List;

@Path("/api/decisions")
@Consumes(MediaType.APPLICATION_JSON)
@Produces(MediaType.APPLICATION_JSON)
public class DecisionResource {
    private static final ObjectMapper JSON = new ObjectMapper();

    @Inject
    AirspaceProductService productService;
    @Inject
    AirspaceVisualizationService visualizationService;

    @POST
    @Path("/evaluate")
    public ProductDtos.DecisionSummary evaluate(ProductDtos.DecisionEvaluateRequest request) {
        return productService.evaluateDecision(request);
    }

    @GET
    @Path("/{id}")
    public ProductDtos.DecisionSummary get(@PathParam("id") String id) {
        return productService.decision(id);
    }

    @POST
    @Path("/{id}/replay")
    @Consumes(MediaType.WILDCARD)
    public ReplayVerificationResult replay(@PathParam("id") String id) {
        ProductDtos.DecisionSummary decision = productService.decision(id);
        if (decision.getResult() != null && decision.getResult().getReplayBundle() != null) {
            return new org.tash.extensions.engine.OperationalDecisionEngine()
                    .replay(decision.getResult().getReplayBundle());
        }
        try {
            OperationalDecisionReplayBundle bundle = productService.decisionReplayBundle(id);
            ReplayVerificationResult typed = new org.tash.extensions.engine.OperationalDecisionEngine().replay(bundle);
            java.util.List<String> warnings = new java.util.ArrayList<>(typed.getWarnings());
            warnings.add("Verified typed persisted replay bundle");
            return ReplayVerificationResult.builder()
                    .accepted(typed.isAccepted())
                    .errors(new java.util.ArrayList<>(typed.getErrors()))
                    .warnings(warnings)
                    .result(typed.getResult())
                    .build();
        } catch (Exception ignored) {
            // Fall back to envelope-only verification for older or partially persisted decisions.
        }
        return verifyPersistedReplay(decision);
    }

    @GET
    @Path("/{id}/features")
    public AirspaceFeatureCollection features(@PathParam("id") String id) {
        ProductDtos.DecisionSummary decision = productService.decision(id);
        OperationalDecisionResult result = decision.getResult();
        if (result == null) {
            result = productService.decisionResult(id);
        }
        VisualizationRequest request = new VisualizationRequest();
        request.setReservations(result.getReservations());
        request.setConflicts(result.getConflicts());
        request.setWeatherProducts(result.getWeatherProducts());
        request.setWeatherAdvisories(weatherAdvisories(result));
        AirspaceFeatureCollection collection = visualizationService.combined(request);
        collection.getFeatures().addAll(avoidanceFeatures(result));
        return collection;
    }

    private List<RouteWeatherAdvisory> weatherAdvisories(OperationalDecisionResult result) {
        List<RouteWeatherAdvisory> advisories = new ArrayList<>();
        for (RouteBlockagePrediction prediction : result.getRouteImpacts()) {
            advisories.add(RouteWeatherAdvisory.builder()
                    .action(result.getAction() == null ? WeatherDecisionAction.MONITOR : result.getAction())
                    .recommendedAction(prediction.getRecommendedAction() == null ? WeatherRecommendedAction.MONITOR : prediction.getRecommendedAction())
                    .severity(prediction.getSeverity() == null ? WeatherDecisionSeverity.INFO : prediction.getSeverity())
                    .rationale(prediction.getRationale())
                    .confidence(prediction.getConfidence())
                    .primaryHazardId(prediction.getPrimaryHazardId())
                    .intersections(prediction.getIntersections())
                    .blockagePredictions(java.util.Collections.singletonList(prediction))
                    .build());
        }
        return advisories;
    }

    private List<org.tash.extensions.visualization.AirspaceFeature> avoidanceFeatures(OperationalDecisionResult result) {
        List<org.tash.extensions.visualization.AirspaceFeature> features = new ArrayList<>();
        result.getAvoidanceCandidates().forEach(candidate -> {
            java.util.Map<String, Object> properties = new java.util.LinkedHashMap<>();
            properties.put("featureKind", "route-avoidance-candidate");
            properties.put("displayLayer", "route-impacts");
            properties.put("constraintType", "ROUTE_AVOIDANCE");
            properties.put("sourceFamily", "WEATHER_ROUTE_IMPACT");
            properties.put("operationalRole", "suggested-deviation-corridor");
            properties.put("label", candidate.getId());
            properties.put("rationale", candidate.getRationale());
            properties.put("routeCost", candidate.getCost());
            properties.put("avoidedConstraintIds", candidate.getAvoidedConstraintIds());
            properties.put("remainingConstraintIds", candidate.getRemainingConstraintIds());
            features.add(org.tash.extensions.visualization.AirspaceFeature.builder()
                    .id("avoidance-" + candidate.getId())
                    .geometry(lineGeometry(candidate.getPoints()))
                    .properties(properties)
                    .build());
        });
        return features;
    }

    private org.tash.extensions.visualization.AirspaceGeometry lineGeometry(List<org.tash.data.GeoCoordinate> points) {
        org.tash.extensions.visualization.AirspaceGeometry geometry = new org.tash.extensions.visualization.AirspaceGeometry();
        geometry.setType("LineString");
        java.util.List<double[]> coordinates = new java.util.ArrayList<>();
        for (org.tash.data.GeoCoordinate point : points) {
            coordinates.add(new double[]{point.getLongitude(), point.getLatitude(), point.getAltitude()});
        }
        geometry.setCoordinates(coordinates);
        return geometry;
    }

    private ReplayVerificationResult verifyPersistedReplay(ProductDtos.DecisionSummary decision) {
        java.util.List<String> errors = new java.util.ArrayList<>();
        java.util.List<String> warnings = new java.util.ArrayList<>();
        if (decision.getReplayJson() == null || decision.getAuditJson() == null) {
            errors.add("Decision replay bundle is not loaded in this runtime and persisted replay JSON is missing");
            return ReplayVerificationResult.builder().accepted(false).errors(errors).warnings(warnings).build();
        }
        try {
            JsonNode replay = JSON.readTree(decision.getReplayJson());
            JsonNode audit = JSON.readTree(decision.getAuditJson());
            String expectedResultHash = text(replay, "expectedResultHash");
            String auditResultHash = text(audit, "resultHash");
            String requestHash = text(audit, "requestHash");
            String ruleCatalogVersion = text(audit, "ruleCatalogVersion");
            String signature = text(audit, "signature");
            if (blank(expectedResultHash)) {
                errors.add("Persisted replay bundle is missing expectedResultHash");
            } else if (!expectedResultHash.equals(auditResultHash)) {
                errors.add("Persisted replay result hash does not match audit result hash");
            }
            if (blank(requestHash) || blank(auditResultHash) || blank(ruleCatalogVersion) || blank(signature)) {
                errors.add("Persisted audit envelope is missing request hash, result hash, rule catalog version, or signature");
            } else if (!new HmacAuditSigner("local-test-key", "airspace-local-audit-secret")
                    .verify(requestHash + ":" + auditResultHash + ":" + ruleCatalogVersion, signature)) {
                errors.add("Persisted audit signature verification failed");
            }
            warnings.add("Verified persisted replay envelope from stored JSON; typed in-memory decision result was not retained");
            return ReplayVerificationResult.builder()
                    .accepted(errors.isEmpty())
                    .errors(errors)
                    .warnings(warnings)
                    .build();
        } catch (Exception e) {
            errors.add("Unable to parse persisted replay JSON: " + e.getMessage());
            return ReplayVerificationResult.builder().accepted(false).errors(errors).warnings(warnings).build();
        }
    }

    private static String text(JsonNode node, String field) {
        JsonNode value = node == null ? null : node.get(field);
        return value == null || value.isNull() ? null : value.asText();
    }

    private static boolean blank(String value) {
        return value == null || value.trim().isEmpty();
    }
}
