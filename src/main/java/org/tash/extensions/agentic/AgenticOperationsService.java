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
    AirspaceSafetyLabAgent safetyLabAgent;
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
        long startedNanos = System.nanoTime();
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
            case "SAFETY_LAB_ALL":
            case "UNSAFE_GUIDANCE_RED_TEAM":
            case "OUTCOME_METRICS_AUDITOR":
            case "SCENARIO_GENERATION":
            case "TMI_RECOMMENDATION_AUDITOR":
            case "BRIEF_DELTA_AGENT":
            case "REPLAY_INTEGRITY_AGENT":
            case "HISTORICAL_CALIBRATION_CURATOR":
            case "NATIONAL_DEMAND_STRESS_AGENT":
            case "COLLABORATIVE_DECISION_FACILITATOR":
            case "COORDINATION_DRAFT_AGENT":
            case "PROVIDER_FRESHNESS_WATCHER":
                result = safetyLab().run(safe);
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
        return finalizeResult(safe, result, policy, startedNanos);
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

    public AgentRunResult safetyLab(AgentRunRequest request) {
        return run(withType(request, "SAFETY_LAB_ALL"));
    }

    public List<AgentWorkloadDefinition> workloads() {
        List<AgentWorkloadDefinition> definitions = new ArrayList<>();
        definitions.add(workload("WEATHER_IMPACT", "Weather Impact Watch", "WEATHER", "Weather/PIREP route impact", "Manual or context refresh"));
        definitions.add(workload("MISSION_RISK", "Mission Risk Analyst", "MISSION", "Mission verdict and affected mission review", "Manual or mission selection"));
        definitions.add(workload("REROUTE_ANALYSIS", "Reroute Analyst", "ROUTE", "Route candidate and residual-risk review", "Manual or route blockage"));
        definitions.add(workload("COORDINATION_DRAFT", "Coordination Draft", "COORDINATION", "Human-approved coordination drafts", "Manual"));
        definitions.add(workload("PILOT_BRIEF", "Pilot Brief", "BRIEF", "Pilot/controller handoff draft", "Manual or brief request"));
        definitions.add(workload("DATA_INTEGRITY", "Data Integrity", "DATA_QUALITY", "NOTAM/weather/PIREP/ALTRV parser and contradiction review", "Manual or ingest"));
        definitions.add(workload("REPLAY_AUDIT", "Replay Audit", "AUDIT", "Decision replay and trace Q&A", "Manual or decision replay"));
        definitions.addAll(safetyLab().workloads());
        return definitions;
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

    private AgentRunResult finalizeResult(AgentRunRequest request, AgentRunResult result, AgentPolicy policy, long startedNanos) {
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
        AgentRunResult audited = withDiagnostics.toBuilder()
                .auditEnvelope(audit)
                .missionId(request == null ? null : request.getMissionId())
                .reservationId(request == null ? null : request.getReservationId())
                .decisionId(request == null ? null : request.getDecisionId())
                .generatedAt(withDiagnostics.getGeneratedAt() == null ? ZonedDateTime.now(ZoneOffset.UTC) : withDiagnostics.getGeneratedAt())
                .build();
        AgentRunResult finalized = audited.toBuilder()
                .costEstimate(costEstimate(withDiagnostics))
                .executionTimeMs(Math.max(0L, (System.nanoTime() - startedNanos) / 1_000_000L))
                .policyGuardDetails(policyGuardDetails(withDiagnostics))
                .replayRefs(replayRefs(request, audited))
                .approvalRequirements(approvalRequirements(withDiagnostics))
                .build();
        store().saveRun(finalized);
        for (AgentTask task : finalized.getTasks()) {
            store().saveTask(task);
        }
        return finalized;
    }

    private double costEstimate(AgentRunResult result) {
        if (result == null) {
            return 0.0;
        }
        if (result.getCostBudget() != null) {
            return result.getCostBudget().getEstimatedCostUsd();
        }
        return result.getCostEstimate();
    }

    private List<AgentPolicyGuard> policyGuardDetails(AgentRunResult result) {
        if (result == null) {
            return Collections.emptyList();
        }
        if (result.getPolicyGuardDetails() != null && !result.getPolicyGuardDetails().isEmpty()) {
            return result.getPolicyGuardDetails();
        }
        List<AgentPolicyGuard> guards = new ArrayList<>();
        for (String guard : result.getPolicyGuards() == null ? Collections.<String>emptyList() : result.getPolicyGuards()) {
            guards.add(AgentPolicyGuard.builder()
                    .id(guard)
                    .label(guardLabel(guard))
                    .description(guardDescription(guard))
                    .enforced(true)
                    .build());
        }
        return guards;
    }

    private String guardLabel(String guard) {
        if (guard == null || guard.trim().isEmpty()) {
            return "Policy guard";
        }
        String normalized = guard.trim().replace('_', ' ').toLowerCase(Locale.US);
        return normalized.substring(0, 1).toUpperCase(Locale.US) + normalized.substring(1);
    }

    private String guardDescription(String guard) {
        String normalized = guard == null ? "" : guard.trim().toUpperCase(Locale.US);
        switch (normalized) {
            case "ADVISORY_ONLY":
                return "Agent output is advisory analysis, draft, or review material.";
            case "NO_EXTERNAL_SEND":
                return "Agent output may not send external coordination traffic.";
            case "NO_OFFICIAL_MUTATION":
                return "Agent output may not mutate official mission, reservation, TMI, or message state.";
            case "HUMAN_APPROVAL_REQUIRED":
                return "Human review is required before operational use.";
            case "CITED_EVIDENCE_REQUIRED":
                return "Claims must carry source references or produce missing-evidence findings.";
            case "LOCAL_OR_REPLAY_FIRST":
                return "Workload runs against local, fixture, or replay evidence unless explicitly configured.";
            default:
                return "Configured agent policy guard.";
        }
    }

    private List<AgentReplayReference> replayRefs(AgentRunRequest request, AgentRunResult result) {
        List<AgentReplayReference> refs = new ArrayList<>();
        if (result != null && result.getReplayRefs() != null && !result.getReplayRefs().isEmpty()) {
            refs.addAll(result.getReplayRefs());
        }
        if (request != null) {
            addReplayRef(refs, request.getRunId(), "SIMULATION_RUN", "/simulation/runs/" + request.getRunId() + "/replay", "Simulation run replay");
            addReplayRef(refs, request.getCampaignId(), "SIMULATION_CAMPAIGN", "/simulation/campaigns/" + request.getCampaignId() + "/report", "Simulation campaign report");
            addReplayRef(refs, request.getDecisionId(), "DECISION", "/decisions/" + request.getDecisionId(), "Decision replay/audit");
        }
        if (result != null && result.getAuditEnvelope() != null && result.getAuditEnvelope().getOutputHash() != null) {
            addReplayRef(refs, result.getAuditEnvelope().getId(), "AGENT_AUDIT", "/config", "Agent audit envelope",
                    result.getAuditEnvelope().getOutputHash());
        }
        return refs;
    }

    private void addReplayRef(List<AgentReplayReference> refs, String id, String type, String route, String label) {
        addReplayRef(refs, id, type, route, label, null);
    }

    private void addReplayRef(List<AgentReplayReference> refs, String id, String type, String route, String label, String hash) {
        if (id == null || id.trim().isEmpty()) {
            return;
        }
        for (AgentReplayReference ref : refs) {
            if (id.equals(ref.getId()) && type.equals(ref.getType())) {
                return;
            }
        }
        refs.add(AgentReplayReference.builder()
                .id(id)
                .type(type)
                .route(route)
                .label(label)
                .hash(hash)
                .build());
    }

    private List<AgentApprovalRequirement> approvalRequirements(AgentRunResult result) {
        if (result == null) {
            return Collections.emptyList();
        }
        if (result.getApprovalRequirements() != null && !result.getApprovalRequirements().isEmpty()) {
            return result.getApprovalRequirements();
        }
        List<AgentApprovalRequirement> requirements = new ArrayList<>();
        for (AgentRecommendation recommendation : result.getRecommendations() == null ? Collections.<AgentRecommendation>emptyList() : result.getRecommendations()) {
            HumanReviewMode mode = recommendation.getHumanReviewMode();
            if (recommendation.isHumanApprovalRequired() || mode == HumanReviewMode.PUSH_APPROVAL || mode == HumanReviewMode.PULL_CLARIFICATION) {
                requirements.add(AgentApprovalRequirement.builder()
                        .id(AgentSupport.id("approval", recommendation.getId()))
                        .mode(mode == null ? HumanReviewMode.PUSH_APPROVAL : mode)
                        .reason(recommendation.getHumanReviewReason())
                        .route(null)
                        .required(true)
                        .citations(recommendation.getCitations())
                        .build());
            }
        }
        for (AgentTask task : result.getTasks() == null ? Collections.<AgentTask>emptyList() : result.getTasks()) {
            HumanReviewMode mode = task.getHumanReviewMode();
            if (mode == HumanReviewMode.PUSH_APPROVAL || mode == HumanReviewMode.PULL_CLARIFICATION) {
                requirements.add(AgentApprovalRequirement.builder()
                        .id(AgentSupport.id("approval", task.getId()))
                        .mode(mode)
                        .reason(task.getHumanReviewReason())
                        .route(task.getRoute())
                        .required(true)
                        .citations(task.getCitations())
                        .build());
            }
        }
        if (requirements.isEmpty() && result.isHumanApprovalRequired()) {
            requirements.add(AgentApprovalRequirement.builder()
                    .id(AgentSupport.id("approval", result.getId()))
                    .mode(HumanReviewMode.REVIEW_ONLY)
                    .reason("Agent result is advisory and requires human review before operational use.")
                    .required(true)
                    .citations(result.getCitations())
                    .build());
        }
        return requirements;
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

    private AgentWorkloadDefinition workload(String id, String label, String category, String gapCoverage, String trigger) {
        return AgentWorkloadDefinition.builder()
                .id(id)
                .label(label)
                .category(category)
                .gapCoverage(gapCoverage)
                .description(label + " produces cited advisory findings through the deterministic Airspace agentic layer.")
                .defaultTrigger(trigger)
                .humanApprovalRequired(true)
                .externalSendAllowed(false)
                .policyGuards(List.of("ADVISORY_ONLY", "NO_EXTERNAL_SEND", "NO_OFFICIAL_MUTATION", "CITED_EVIDENCE_REQUIRED"))
                .build();
    }

    private AirspaceSafetyLabAgent safetyLab() {
        if (safetyLabAgent == null) {
            safetyLabAgent = new AirspaceSafetyLabAgent();
        }
        return safetyLabAgent;
    }
}
