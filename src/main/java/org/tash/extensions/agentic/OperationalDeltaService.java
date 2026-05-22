package org.tash.extensions.agentic;

import jakarta.enterprise.context.ApplicationScoped;
import jakarta.inject.Inject;
import org.tash.extensions.product.application.AirspaceProductService;
import org.tash.extensions.product.dto.ProductDtos;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;

@ApplicationScoped
public class OperationalDeltaService {
    @Inject
    AirspaceProductService productService;

    public List<AgentOperationalDelta> compare(String previousDecisionId, String currentDecisionId, String missionId) {
        List<AgentOperationalDelta> deltas = new ArrayList<>();
        ProductDtos.DecisionSummary previous = decision(previousDecisionId);
        ProductDtos.DecisionSummary current = decision(currentDecisionId);
        if (previous != null && current != null) {
            addDecisionDeltas(previous, current, deltas);
        }
        if (current != null && current.getRouteImpact() != null) {
            addRouteImpactDeltas(previous == null ? null : previous.getRouteImpact(), current.getRouteImpact(), deltas);
        }
        if (deltas.isEmpty() && missionId != null && productService != null) {
            for (ProductDtos.WeatherSourceSummary source : productService.weatherChanges(missionId, null, 8)) {
                AgentSourceCitation citation = AgentSupport.citation(source.getFamily(), source.getId(), source.getLabel(), "/weather");
                deltas.add(AgentOperationalDelta.builder()
                        .id(AgentSupport.id("delta", "source:" + missionId + ":" + source.getFamily() + ":" + source.getId()))
                        .changeType("SOURCE_OBSERVED")
                        .sourceFamily(source.getFamily())
                        .sourceId(source.getId())
                        .previousValue("not in previous brief window")
                        .currentValue(AgentSupport.value(source.getSeverity(), AgentSupport.value(source.getLabel(), source.getFamily())))
                        .severity(AgentSupport.value(source.getSeverity(), "INFO"))
                        .observedAt(source.getObservedAt())
                        .citations(java.util.Collections.singletonList(citation))
                        .build());
            }
        }
        return deltas;
    }

    private void addDecisionDeltas(ProductDtos.DecisionSummary previous,
                                   ProductDtos.DecisionSummary current,
                                   List<AgentOperationalDelta> deltas) {
        List<AgentSourceCitation> citations = current.getRouteImpact() == null
                ? java.util.Collections.singletonList(AgentSupport.citation("DECISION", current.getId(), current.getAction(), "/decisions/" + current.getId()))
                : AgentSupport.citations(current.getRouteImpact());
        if (!Objects.equals(previous.getAction(), current.getAction())) {
            deltas.add(delta("ACTION_CHANGED", "DECISION", current.getId(), previous.getAction(), current.getAction(), severity(current), citations));
        }
        if (!Objects.equals(previous.getRecommendedAction(), current.getRecommendedAction())) {
            deltas.add(delta("RECOMMENDATION_CHANGED", "DECISION", current.getId(), previous.getRecommendedAction(), current.getRecommendedAction(), severity(current), citations));
        }
        if (Math.abs(previous.getConfidence() - current.getConfidence()) >= 0.05) {
            deltas.add(delta("CONFIDENCE_CHANGED", "DECISION", current.getId(),
                    percent(previous.getConfidence()), percent(current.getConfidence()), severity(current), citations));
        }
    }

    private void addRouteImpactDeltas(ProductDtos.RouteImpactSummary previous,
                                      ProductDtos.RouteImpactSummary current,
                                      List<AgentOperationalDelta> deltas) {
        List<AgentSourceCitation> citations = AgentSupport.citations(current);
        Set<String> previousSources = previous == null ? java.util.Collections.emptySet() : new LinkedHashSet<>(previous.getSourceRefs());
        for (String sourceRef : current.getSourceRefs()) {
            if (!previousSources.contains(sourceRef)) {
                String family = sourceRef.contains(":") ? sourceRef.substring(0, sourceRef.indexOf(':')) : "SOURCE";
                deltas.add(delta("SOURCE_ADDED", family, sourceRef, "absent", "present", severity(current), citations));
            }
        }
        if (previous != null) {
            if (previous.getBlockingConstraintCount() != current.getBlockingConstraintCount()) {
                deltas.add(delta("BLOCKING_CONSTRAINT_COUNT_CHANGED", "ROUTE_IMPACT", current.getMissionId(),
                        String.valueOf(previous.getBlockingConstraintCount()), String.valueOf(current.getBlockingConstraintCount()), severity(current), citations));
            }
            if (previous.getCandidateComparisons().size() != current.getCandidateComparisons().size()) {
                deltas.add(delta("ROUTE_CANDIDATE_COUNT_CHANGED", "ROUTE_IMPACT", current.getMissionId(),
                        String.valueOf(previous.getCandidateComparisons().size()), String.valueOf(current.getCandidateComparisons().size()), severity(current), citations));
            }
        } else if (!current.getCandidateComparisons().isEmpty()) {
            deltas.add(delta("ROUTE_CANDIDATES_AVAILABLE", "ROUTE_IMPACT", current.getMissionId(),
                    "none", String.valueOf(current.getCandidateComparisons().size()), severity(current), citations));
        }
    }

    private ProductDtos.DecisionSummary decision(String id) {
        if (id == null || id.trim().isEmpty() || productService == null) {
            return null;
        }
        try {
            return productService.decision(id);
        } catch (Exception ignored) {
            return null;
        }
    }

    private AgentOperationalDelta delta(String type,
                                        String family,
                                        String sourceId,
                                        String previous,
                                        String current,
                                        String severity,
                                        List<AgentSourceCitation> citations) {
        return AgentOperationalDelta.builder()
                .id(AgentSupport.id("delta", type + ":" + sourceId + ":" + previous + ":" + current))
                .changeType(type)
                .sourceFamily(family)
                .sourceId(sourceId)
                .previousValue(AgentSupport.value(previous, "none"))
                .currentValue(AgentSupport.value(current, "none"))
                .severity(severity)
                .citations(citations)
                .build();
    }

    private String severity(ProductDtos.DecisionSummary decision) {
        return decision == null ? "INFO" : severity(decision.getRouteImpact());
    }

    private String severity(ProductDtos.RouteImpactSummary impact) {
        if (impact == null) {
            return "INFO";
        }
        String action = AgentSupport.value(impact.getAction(), impact.getRecommendedAction()).toUpperCase(java.util.Locale.US);
        if (action.contains("BLOCK")) {
            return "HIGH";
        }
        if (action.contains("REROUTE") || action.contains("AVOID")) {
            return "MEDIUM";
        }
        return "INFO";
    }

    private String percent(double value) {
        return Math.round(value * 100.0) + "%";
    }
}
