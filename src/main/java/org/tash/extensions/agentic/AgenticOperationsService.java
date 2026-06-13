package org.tash.extensions.agentic;

import jakarta.enterprise.context.ApplicationScoped;
import jakarta.enterprise.inject.Instance;
import jakarta.inject.Inject;

import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

@ApplicationScoped
public class AgenticOperationsService {
    @Inject
    WeatherImpactWatchAgent weatherImpactWatchAgent;
    @Inject
    MissionRiskAnalystAgent missionRiskAnalystAgent;
    @Inject
    RerouteAnalystAgent rerouteAnalystAgent;
    @Inject
    CoordinationDraftAgent coordinationDraftAgent;
    @Inject
    PilotBriefAgent pilotBriefAgent;
    @Inject
    DataIntegrityAgent dataIntegrityAgent;
    @Inject
    ReplayAuditAgent replayAuditAgent;
    @Inject
    AgentCitationValidator citationValidator;
    @Inject
    AgentPolicyEnforcer policyEnforcer;
    @Inject
    AgentEvaluationService evaluationService;
    @Inject
    AgentAssessmentBuilder assessmentBuilder;
    @Inject
    AgentReasoningContextBuilder reasoningContextBuilder;
    @Inject
    AgentAuditService auditService;
    @Inject
    AgentRunStore runStore;
    @Inject
    Instance<LlmReasoningProvider> llmReasoningProviders;

    public AgentRunResult run(AgentRunRequest request) {
        AgentRunRequest safe = request == null ? new AgentRunRequest() : request;
        AgentPolicy policy = safe.getPolicy() == null ? AgentPolicy.builder().build() : safe.getPolicy();
        String type = normalize(safe.getAgentType());
        AgentRunResult result;
        switch (type) {
            case "WEATHER_IMPACT":
                result = weatherImpactWatchAgent.evaluate(safe);
                break;
            case "MISSION_RISK":
                result = missionRiskAnalystAgent.evaluate(safe);
                break;
            case "REROUTE_ANALYSIS":
                result = rerouteAnalystAgent.evaluate(safe);
                break;
            case "COORDINATION_DRAFT":
                result = coordinationDraftAgent.createDraft(safe);
                break;
            case "PILOT_BRIEF":
                result = pilotBriefAgent.generate(safe);
                break;
            case "DATA_INTEGRITY":
                result = dataIntegrityAgent.scan(safe);
                break;
            case "REPLAY_AUDIT":
                result = replayAuditAgent.explain(safe);
                break;
            case "ALL":
            default:
                result = runAll(safe);
                break;
        }
        result = mergeRequestToolCalls(safe, result);
        result = result.toBuilder()
                .reasoningEnvelope(reasoningContextBuilder == null
                        ? new AgentReasoningContextBuilder().build(safe, result, policy)
                        : reasoningContextBuilder.build(safe, result, policy))
                .build();
        if (llmReasoningProviders != null && !llmReasoningProviders.isUnsatisfied()) {
            AgentRunResult reasoned = llmReasoningProviders.get().reason(safe, result);
            result = reasoned.getReasoningEnvelope() == null
                    ? reasoned.toBuilder().reasoningEnvelope(result.getReasoningEnvelope()).build()
                    : reasoned;
        }
        return finalizeResult(safe, result, policy);
    }

    public AgentRunResult weatherImpact(AgentRunRequest request) {
        return run(withType(request, "WEATHER_IMPACT"));
    }

    public AgentRunResult missionRisk(AgentRunRequest request) {
        return run(withType(request, "MISSION_RISK"));
    }

    public AgentRunResult rerouteAnalysis(AgentRunRequest request) {
        return run(withType(request, "REROUTE_ANALYSIS"));
    }

    public AgentRunResult coordinationDraft(AgentRunRequest request) {
        return run(withType(request, "COORDINATION_DRAFT"));
    }

    public AgentRunResult pilotBrief(AgentRunRequest request) {
        return run(withType(request, "PILOT_BRIEF"));
    }

    public AgentRunResult dataIntegrity(AgentRunRequest request) {
        return run(withType(request, "DATA_INTEGRITY"));
    }

    public AgentRunResult replayAudit(AgentRunRequest request) {
        return run(withType(request, "REPLAY_AUDIT"));
    }

    public List<AgentRunResult> runs(Integer limit) {
        return store().runs(limit);
    }

    public List<AgentRunResult> runs(AgentHistoryQuery query) {
        return store().runs(query);
    }

    public AgentRunResult runById(String id) {
        return store().findRun(id).orElseThrow(() -> new IllegalArgumentException("Unknown agent run: " + id));
    }

    public List<AgentTask> tasks(String status, Integer limit) {
        return store().tasks(status, limit);
    }

    public List<AgentTask> tasks(AgentHistoryQuery query) {
        return store().tasks(query);
    }

    public AgentTask taskById(String id) {
        return store().findTask(id).orElseThrow(() -> new IllegalArgumentException("Unknown agent task: " + id));
    }

    public AgentTask transitionTask(String id, AgentTaskTransitionRequest request) {
        return store().transitionTask(id, request);
    }

    public AgentStoreStatus status() {
        return store().status();
    }

    public Map<String, Double> metrics() {
        Map<String, Double> metrics = new LinkedHashMap<>();
        AgentStoreStatus status = status();
        metrics.put("agentic.runs", (double) status.getRunCount());
        metrics.put("agentic.tasks", (double) status.getTaskCount());
        metrics.put("agentic.store.durable", status.isDurable() ? 1.0 : 0.0);
        for (AgentRunResult run : runs(250)) {
            metrics.put("agentic.runs." + normalizeMetricPart(run.getAgentType()),
                    metrics.getOrDefault("agentic.runs." + normalizeMetricPart(run.getAgentType()), 0.0) + 1.0);
            metrics.put(run.isAccepted() ? "agentic.runs.accepted" : "agentic.runs.rejected",
                    metrics.getOrDefault(run.isAccepted() ? "agentic.runs.accepted" : "agentic.runs.rejected", 0.0) + 1.0);
            if (run.getEvaluation() != null) {
                metrics.put("agentic.uncitedClaims",
                        metrics.getOrDefault("agentic.uncitedClaims", 0.0) + run.getEvaluation().getUncitedClaimCount());
                metrics.put("agentic.policyViolations",
                        metrics.getOrDefault("agentic.policyViolations", 0.0) + run.getEvaluation().getPolicyViolationCount());
            }
        }
        for (AgentTask task : tasks((String) null, 250)) {
            String statusKey = "agentic.tasks.status." + normalizeMetricPart(task.getStatus());
            String priorityKey = "agentic.tasks.priority." + normalizeMetricPart(task.getPriority());
            metrics.put(statusKey, metrics.getOrDefault(statusKey, 0.0) + 1.0);
            metrics.put(priorityKey, metrics.getOrDefault(priorityKey, 0.0) + 1.0);
        }
        return metrics;
    }

    private AgentRunResult runAll(AgentRunRequest request) {
        List<AgentRunResult> results = new ArrayList<>();
        results.add(weatherImpactWatchAgent.evaluate(request));
        results.add(missionRiskAnalystAgent.evaluate(request));
        results.add(rerouteAnalystAgent.evaluate(request));
        results.add(coordinationDraftAgent.createDraft(request));
        results.add(pilotBriefAgent.generate(request));
        results.add(dataIntegrityAgent.scan(request));
        results.add(replayAuditAgent.explain(request));
        List<AgentFinding> findings = new ArrayList<>();
        List<AgentRecommendation> recommendations = new ArrayList<>();
        List<AgentTask> tasks = new ArrayList<>();
        List<AgentToolCall> tools = new ArrayList<>();
        List<AgentSourceCitation> citations = new ArrayList<>();
        List<AgentOperationalDelta> deltas = new ArrayList<>();
        List<String> diagnostics = new ArrayList<>();
        double confidence = 0.0;
        for (AgentRunResult result : results) {
            findings.addAll(result.getFindings());
            recommendations.addAll(result.getRecommendations());
            tasks.addAll(result.getTasks());
            tools.addAll(result.getToolCalls());
            citations.addAll(result.getCitations());
            deltas.addAll(result.getDeltas());
            diagnostics.addAll(result.getDiagnostics());
            confidence += result.getConfidence();
        }
        return AgentRunResult.builder()
                .id(AgentSupport.id("agent", "all:" + findings.size() + ":" + recommendations.size() + ":" + tasks.size()))
                .agentType("ALL")
                .summary("Agentic operations review produced " + findings.size() + " finding(s), "
                        + recommendations.size() + " recommendation(s), and " + tasks.size() + " task(s).")
                .confidence(results.isEmpty() ? 0.0 : confidence / results.size())
                .accepted(true)
                .generatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .findings(findings)
                .recommendations(recommendations)
                .tasks(tasks)
                .toolCalls(tools)
                .citations(citations)
                .deltas(deltas)
                .diagnostics(diagnostics)
                .build();
    }

    private AgentRunResult finalizeResult(AgentRunRequest request, AgentRunResult result, AgentPolicy policy) {
        AgentAssessmentBuilder safeAssessmentBuilder = assessmentBuilder == null ? new AgentAssessmentBuilder() : assessmentBuilder;
        result = safeAssessmentBuilder.normalize(result);
        AgentEvaluationSummary evaluation = evaluationService == null
                ? fallbackEvaluation(result, policy)
                : evaluationService.evaluate(result, policy);
        List<String> diagnostics = new ArrayList<>(result.getDiagnostics());
        diagnostics.addAll(evaluation.getWarnings());
        diagnostics.addAll(evaluation.getErrors());
        boolean accepted = result.isAccepted() && evaluation.isAccepted();
        AgentRunResult withDiagnostics = result.toBuilder()
                .accepted(accepted)
                .diagnostics(diagnostics)
                .evaluation(evaluation)
                .operatingLoop(result.getOperatingLoop().isEmpty() ? operatingLoop(result, diagnostics) : result.getOperatingLoop())
                .build();
        AgentAuditEnvelope audit = auditService.record(request, withDiagnostics, policy);
        AgentRunResult finalized = withDiagnostics.toBuilder()
                .auditEnvelope(audit)
                .missionId(request == null ? null : request.getMissionId())
                .reservationId(request == null ? null : request.getReservationId())
                .decisionId(request == null ? null : request.getDecisionId())
                .generatedAt(withDiagnostics.getGeneratedAt() == null ? ZonedDateTime.now(ZoneOffset.UTC) : withDiagnostics.getGeneratedAt())
                .build();
        store().saveRun(finalized);
        for (AgentTask task : finalized.getTasks()) {
            store().saveTask(task);
        }
        return finalized;
    }

    private AgentRunResult mergeRequestToolCalls(AgentRunRequest request, AgentRunResult result) {
        if (request == null || request.getToolCalls() == null || request.getToolCalls().isEmpty()) {
            return result;
        }
        List<AgentToolCall> calls = new ArrayList<>(result.getToolCalls());
        calls.addAll(request.getToolCalls());
        List<AgentSourceCitation> citations = new ArrayList<>(result.getCitations());
        for (AgentToolCall call : request.getToolCalls()) {
            if (call.getSourceRefs() != null) {
                citations.addAll(call.getSourceRefs());
            }
        }
        List<String> diagnostics = new ArrayList<>(result.getDiagnostics());
        diagnostics.add("Attached " + request.getToolCalls().size() + " curated MCP evidence receipt(s) to agent run.");
        return result.toBuilder()
                .toolCalls(calls)
                .citations(citations.isEmpty() ? result.getCitations() : citations)
                .diagnostics(diagnostics)
                .build();
    }

    private AgentRunRequest withType(AgentRunRequest request, String type) {
        AgentRunRequest safe = request == null ? new AgentRunRequest() : request;
        safe.setAgentType(type);
        return safe;
    }

    private AgentEvaluationSummary fallbackEvaluation(AgentRunResult result, AgentPolicy policy) {
        List<String> errors = new ArrayList<>();
        errors.addAll(citationValidator == null ? Collections.emptyList() : citationValidator.validate(result, policy));
        errors.addAll(policyEnforcer == null ? Collections.emptyList() : policyEnforcer.validate(result, policy));
        int totalClaims = result.getFindings().size() + result.getRecommendations().size() + result.getTasks().size();
        int uncited = (int) errors.stream().filter(value -> value.toLowerCase(Locale.US).contains("citation")).count();
        return AgentEvaluationSummary.builder()
                .accepted(errors.isEmpty())
                .findingCount(result.getFindings().size())
                .recommendationCount(result.getRecommendations().size())
                .taskCount(result.getTasks().size())
                .deltaCount(result.getDeltas().size())
                .citedClaimCount(Math.max(0, totalClaims - uncited))
                .uncitedClaimCount(uncited)
                .policyViolationCount((int) errors.stream().filter(value -> value.toLowerCase(Locale.US).contains("policy")).count())
                .citationCoverage(totalClaims == 0 ? 1.0 : (double) Math.max(0, totalClaims - uncited) / (double) totalClaims)
                .stabilityAccepted(true)
                .errors(errors)
                .build();
    }

    private String normalize(String type) {
        if (type == null || type.trim().isEmpty()) {
            return "ALL";
        }
        return type.trim().replace('-', '_').replace(' ', '_').toUpperCase(Locale.US);
    }

    private String normalizeMetricPart(String value) {
        if (value == null || value.trim().isEmpty()) {
            return "unknown";
        }
        return value.trim().replace('-', '_').replace(' ', '_').toLowerCase(Locale.US);
    }

    private List<AgentOperatingLoopStep> operatingLoop(AgentRunResult result, List<String> diagnostics) {
        List<AgentSourceCitation> citations = result.getCitations() == null || result.getCitations().isEmpty()
                ? Collections.singletonList(AgentSupport.citation("ENGINE", "agentic", "Agentic operations engine", "/decisions/latest"))
                : result.getCitations();
        List<AgentOperatingLoopStep> steps = new ArrayList<>();
        steps.add(loop("OBSERVE", "COMPLETE", "Operational messages, mission context, weather/PIREP/NOTAM sources, decisions, and route artifacts were collected for agent review.", citations));
        steps.add(loop("NORMALIZE", "COMPLETE", "Inputs were consumed as typed product/engine artifacts with retained source references.", citations));
        steps.add(loop("FUSE", "COMPLETE", "Agent analysis is grounded in fused mission verdict, route impact, decision trace, or replay outputs.", citations));
        steps.add(loop("PREDICT", "COMPLETE", result.getFindings().size() + " finding(s) and " + result.getRecommendations().size() + " recommendation(s) summarize predicted operational impact.", citations));
        steps.add(loop("GUIDE", result.getRecommendations().isEmpty() ? "NOT_APPLICABLE" : "COMPLETE", result.getRecommendations().isEmpty() ? "No agent recommendation was required." : "Recommendations remain advisory and human-approved.", citations));
        steps.add(loop("COORDINATE", result.getTasks().isEmpty() ? "NOT_APPLICABLE" : "READY", result.getTasks().isEmpty() ? "No coordination task was opened." : result.getTasks().size() + " review/coordination task(s) are available.", citations));
        steps.add(loop("EXPLAIN", "COMPLETE", "Agent output includes cited rationale and can answer trace/replay questions from retained evidence.", citations));
        steps.add(loop("AUDIT", diagnostics.isEmpty() ? "COMPLETE" : "WARNING", diagnostics.isEmpty() ? "Citation validation accepted and audit hashes were generated." : "Audit retained diagnostics: " + diagnostics, citations));
        return steps;
    }

    private AgentOperatingLoopStep loop(String stage, String status, String summary, List<AgentSourceCitation> citations) {
        return AgentOperatingLoopStep.builder()
                .stage(stage)
                .status(status)
                .summary(summary)
                .citations(citations)
                .build();
    }

    private AgentRunStore store() {
        if (runStore == null) {
            runStore = new InMemoryAgentRunStore();
        }
        return runStore;
    }
}
