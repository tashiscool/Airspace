package org.tash.extensions.agentic.stability;

import jakarta.enterprise.context.ApplicationScoped;
import jakarta.inject.Inject;
import org.tash.extensions.agentic.AgentRunRequest;
import org.tash.extensions.agentic.AgentRunResult;
import org.tash.extensions.agentic.AgenticOperationsService;

import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

@ApplicationScoped
public class AgentStabilityHarness {
    @Inject
    AgenticOperationsService agenticOperationsService;
    @Inject
    AgentRunComparator comparator;

    public AgentStabilityHarness() {
    }

    public AgentStabilityHarness(AgenticOperationsService agenticOperationsService, AgentRunComparator comparator) {
        this.agenticOperationsService = agenticOperationsService;
        this.comparator = comparator;
    }

    public AgentStabilityResult evaluate(AgentStabilityRequest request) {
        AgentStabilityRequest safe = request == null ? new AgentStabilityRequest() : request;
        int iterations = iterations(safe.getIterations());
        double minAgreement = value(safe.getMinAgreement(), 1.0);
        double minCitationJaccard = value(safe.getMinCitationJaccard(), 0.95);
        double maxCv = value(safe.getMaxCountCoefficientOfVariation(), 0.0);
        ZonedDateTime started = ZonedDateTime.now(ZoneOffset.UTC);
        List<AgentRunResult> runs = new ArrayList<>();
        List<String> runIds = new ArrayList<>();
        AgentRunRequest runRequest = safe.getAgentRunRequest() == null ? new AgentRunRequest() : safe.getAgentRunRequest();
        for (int i = 0; i < iterations; i++) {
            AgentRunResult run = service().run(copy(runRequest));
            runs.add(run);
            runIds.add(run.getId());
        }
        List<AgentStabilityMetric> metrics = compare().compare(runs, minAgreement, minCitationJaccard, maxCv);
        boolean accepted = metrics.stream().allMatch(AgentStabilityMetric::isAccepted);
        List<String> diagnostics = new ArrayList<>();
        if (!accepted) {
            for (AgentStabilityMetric metric : metrics) {
                if (!metric.isAccepted()) {
                    diagnostics.add(metric.getName() + " failed: " + metric.getValue() + " vs threshold " + metric.getThreshold());
                }
            }
        }
        ZonedDateTime completed = ZonedDateTime.now(ZoneOffset.UTC);
        return AgentStabilityResult.builder()
                .id("agent-stability-" + java.util.UUID.nameUUIDFromBytes(String.join(":", runIds).getBytes(java.nio.charset.StandardCharsets.UTF_8)))
                .accepted(accepted)
                .iterations(iterations)
                .startedAt(started)
                .completedAt(completed)
                .runIds(runIds)
                .metrics(metrics)
                .diagnostics(diagnostics)
                .build();
    }

    private AgentRunRequest copy(AgentRunRequest source) {
        AgentRunRequest copy = new AgentRunRequest();
        copy.setAgentType(source.getAgentType());
        copy.setMissionId(source.getMissionId());
        copy.setReservationId(source.getReservationId());
        copy.setDecisionId(source.getDecisionId());
        copy.setPreviousDecisionId(source.getPreviousDecisionId());
        copy.setHazardOrDecisionId(source.getHazardOrDecisionId());
        copy.setQuestion(source.getQuestion());
        copy.setActor(source.getActor());
        copy.setFeedArtifactId(source.getFeedArtifactId());
        copy.setReferenceType(source.getReferenceType());
        copy.setReferenceIdentifier(source.getReferenceIdentifier());
        copy.setPolicy(source.getPolicy());
        copy.setToolCalls(source.getToolCalls() == null ? new ArrayList<>() : new ArrayList<>(source.getToolCalls()));
        return copy;
    }

    private int iterations(Integer requested) {
        if (requested == null) {
            return 3;
        }
        return Math.max(2, Math.min(10, requested));
    }

    private double value(Double requested, double fallback) {
        return requested == null ? fallback : requested;
    }

    private AgenticOperationsService service() {
        if (agenticOperationsService == null) {
            agenticOperationsService = new AgenticOperationsService();
        }
        return agenticOperationsService;
    }

    private AgentRunComparator compare() {
        if (comparator == null) {
            comparator = new AgentRunComparator();
        }
        return comparator;
    }
}
