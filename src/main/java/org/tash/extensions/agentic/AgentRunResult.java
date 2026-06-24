package org.tash.extensions.agentic;

import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;
import lombok.AllArgsConstructor;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder(toBuilder = true)
public class AgentRunResult {
    private String id;
    private String agentType;
    private String missionId;
    private String reservationId;
    private String decisionId;
    private String summary;
    private double confidence;
    private boolean accepted;
    private boolean humanApprovalRequired;
    private boolean externalSendPerformed;
    private boolean officialStateMutationPerformed;
    private ZonedDateTime generatedAt;
    private AgentCostBudget costBudget;
    private AgentAuditEnvelope auditEnvelope;
    private AgentEvaluationSummary evaluation;
    private AgentReasoningEnvelope reasoningEnvelope;
    private AgentTraceAnswer traceAnswer;
    @Builder.Default
    private List<AgentFinding> findings = new ArrayList<>();
    @Builder.Default
    private List<AgentRecommendation> recommendations = new ArrayList<>();
    @Builder.Default
    private List<AgentTask> tasks = new ArrayList<>();
    @Builder.Default
    private List<AgentToolCall> toolCalls = new ArrayList<>();
    @Builder.Default
    private List<AgentAssessment> assessments = new ArrayList<>();
    @Builder.Default
    private List<AgentSourceCitation> citations = new ArrayList<>();
    @Builder.Default
    private List<AgentOperationalDelta> deltas = new ArrayList<>();
    @Builder.Default
    private List<AgentOperatingLoopStep> operatingLoop = new ArrayList<>();
    @Builder.Default
    private List<AgentEvidenceReceipt> evidenceReceipts = new ArrayList<>();
    @Builder.Default
    private List<String> policyGuards = new ArrayList<>();
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
}
