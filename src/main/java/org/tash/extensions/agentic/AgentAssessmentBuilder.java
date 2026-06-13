package org.tash.extensions.agentic;

import jakarta.enterprise.context.ApplicationScoped;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

@ApplicationScoped
public class AgentAssessmentBuilder {
    public AgentRunResult normalize(AgentRunResult result) {
        if (result == null) {
            return null;
        }
        List<AgentRecommendation> recommendations = new ArrayList<>();
        for (AgentRecommendation recommendation : result.getRecommendations()) {
            recommendations.add(normalize(recommendation));
        }
        List<AgentTask> tasks = new ArrayList<>();
        for (AgentTask task : result.getTasks()) {
            tasks.add(normalize(task));
        }
        AgentRunResult normalized = result.toBuilder()
                .recommendations(recommendations)
                .tasks(tasks)
                .build();
        if (normalized.getAssessments() != null && !normalized.getAssessments().isEmpty()) {
            return normalized;
        }
        return normalized.toBuilder()
                .assessments(assessments(normalized))
                .build();
    }

    public AgentRecommendation normalize(AgentRecommendation recommendation) {
        if (recommendation == null) {
            return null;
        }
        HumanReviewMode mode = recommendation.getHumanReviewMode();
        if (mode == null) {
            mode = inferRecommendationMode(recommendation);
        }
        boolean approvalRequired = recommendation.isHumanApprovalRequired() || mode == HumanReviewMode.PUSH_APPROVAL;
        return recommendation.toBuilder()
                .humanReviewMode(mode)
                .humanReviewReason(reviewReason(mode, recommendation.getAction(), recommendation.getSummary()))
                .humanApprovalRequired(approvalRequired)
                .build();
    }

    public AgentTask normalize(AgentTask task) {
        if (task == null) {
            return null;
        }
        HumanReviewMode mode = task.getHumanReviewMode();
        if (mode == null) {
            mode = inferTaskMode(task);
        }
        return task.toBuilder()
                .humanReviewMode(mode)
                .humanReviewReason(reviewReason(mode, task.getTitle(), task.getRationale()))
                .build();
    }

    public List<AgentAssessment> assessments(AgentRunResult result) {
        if (result == null) {
            return Collections.emptyList();
        }
        List<AgentAssessment> assessments = new ArrayList<>();
        for (AgentFinding finding : result.getFindings()) {
            assessments.add(fromFinding(result, finding));
        }
        for (AgentRecommendation recommendation : result.getRecommendations()) {
            assessments.add(fromRecommendation(result, recommendation));
        }
        for (AgentTask task : result.getTasks()) {
            assessments.add(fromTask(result, task));
        }
        return assessments;
    }

    private AgentAssessment fromFinding(AgentRunResult result, AgentFinding finding) {
        HumanReviewMode mode = inferFindingMode(finding);
        return AgentAssessment.builder()
                .id("assessment-" + value(finding.getId(), "finding"))
                .schemaVersion("agent-assessment-v1")
                .agentType(result.getAgentType())
                .claim(finding.getMessage())
                .verdict(value(finding.getSeverity(), finding.getCategory()))
                .confidence(finding.getConfidence())
                .evidence(evidenceLabels(finding.getCitations()))
                .counterEvidence(counterEvidence(finding.getMessage()))
                .uncertainty(uncertainty(finding.getConfidence(), finding.getCitations()))
                .requiredHumanAction(mode == HumanReviewMode.PULL_CLARIFICATION
                        ? "Clarify ambiguous or missing evidence before relying on this finding."
                        : "Review cited evidence before operational use.")
                .humanReviewMode(mode)
                .citations(finding.getCitations())
                .build();
    }

    private AgentAssessment fromRecommendation(AgentRunResult result, AgentRecommendation recommendation) {
        AgentRecommendation normalized = normalize(recommendation);
        return AgentAssessment.builder()
                .id("assessment-" + value(normalized.getId(), "recommendation"))
                .schemaVersion("agent-assessment-v1")
                .agentType(result.getAgentType())
                .claim(value(normalized.getSummary(), normalized.getAction()))
                .verdict(value(normalized.getAction(), "RECOMMENDATION"))
                .confidence(normalized.getConfidence())
                .evidence(evidenceLabels(normalized.getCitations()))
                .counterEvidence(counterEvidence(normalized.getRationale()))
                .uncertainty(uncertainty(normalized.getConfidence(), normalized.getCitations()))
                .requiredHumanAction(actionFor(normalized.getHumanReviewMode()))
                .humanReviewMode(normalized.getHumanReviewMode())
                .citations(normalized.getCitations())
                .build();
    }

    private AgentAssessment fromTask(AgentRunResult result, AgentTask task) {
        AgentTask normalized = normalize(task);
        return AgentAssessment.builder()
                .id("assessment-" + value(normalized.getId(), "task"))
                .schemaVersion("agent-assessment-v1")
                .agentType(result.getAgentType())
                .claim(value(normalized.getTitle(), normalized.getRationale()))
                .verdict(value(normalized.getPriority(), "TASK"))
                .confidence(0.75)
                .evidence(evidenceLabels(normalized.getCitations()))
                .counterEvidence(counterEvidence(normalized.getRationale()))
                .uncertainty(uncertainty(0.75, normalized.getCitations()))
                .requiredHumanAction(actionFor(normalized.getHumanReviewMode()))
                .humanReviewMode(normalized.getHumanReviewMode())
                .citations(normalized.getCitations())
                .build();
    }

    private HumanReviewMode inferFindingMode(AgentFinding finding) {
        String text = combined(finding == null ? null : finding.getCategory(), finding == null ? null : finding.getMessage());
        if (text.contains("MISSING") || text.contains("AMBIGUOUS") || text.contains("UNSUPPORTED")
                || text.contains("MALFORMED") || text.contains("CONFLICT") || text.contains("CONTRADICT")) {
            return HumanReviewMode.PULL_CLARIFICATION;
        }
        return HumanReviewMode.REVIEW_ONLY;
    }

    private HumanReviewMode inferRecommendationMode(AgentRecommendation recommendation) {
        if (recommendation.isHumanApprovalRequired()) {
            return HumanReviewMode.PUSH_APPROVAL;
        }
        String text = combined(recommendation.getAction(), recommendation.getSummary(), recommendation.getRationale());
        if (text.contains("SEND") || text.contains("TRANSMIT") || text.contains("APPROVE")
                || text.contains("SUBMIT") || text.contains("CANCEL") || text.contains("COMPLETE")
                || text.contains("DRAFT") || text.contains("COORDINATION")) {
            return HumanReviewMode.PUSH_APPROVAL;
        }
        if (text.contains("MISSING") || text.contains("AMBIGUOUS") || text.contains("UNSUPPORTED")
                || text.contains("MALFORMED") || text.contains("CONFLICT") || text.contains("CONTRADICT")) {
            return HumanReviewMode.PULL_CLARIFICATION;
        }
        return HumanReviewMode.REVIEW_ONLY;
    }

    private HumanReviewMode inferTaskMode(AgentTask task) {
        String text = combined(task.getTitle(), task.getRationale(), task.getAssignedRole(), task.getRoute());
        if (text.contains("SEND") || text.contains("TRANSMIT") || text.contains("APPROVE")
                || text.contains("SUBMIT") || text.contains("CANCEL") || text.contains("COMPLETE")
                || text.contains("DRAFT") || text.contains("COORDINATION")) {
            return HumanReviewMode.PUSH_APPROVAL;
        }
        if (text.contains("MISSING") || text.contains("AMBIGUOUS") || text.contains("UNSUPPORTED")
                || text.contains("MALFORMED") || text.contains("CONFLICT") || text.contains("CONTRADICT")) {
            return HumanReviewMode.PULL_CLARIFICATION;
        }
        return HumanReviewMode.REVIEW_ONLY;
    }

    private String reviewReason(HumanReviewMode mode, String primary, String secondary) {
        switch (mode == null ? HumanReviewMode.REVIEW_ONLY : mode) {
            case NONE:
                return "No human action required.";
            case PULL_CLARIFICATION:
                return "Agent needs missing, ambiguous, or conflicting evidence clarified before reliance.";
            case PUSH_APPROVAL:
                return "Agent output is draft-only and requires human approval before official operational action.";
            case REVIEW_ONLY:
            default:
                return "Agent output is advisory and should be reviewed with cited evidence.";
        }
    }

    private String actionFor(HumanReviewMode mode) {
        switch (mode == null ? HumanReviewMode.REVIEW_ONLY : mode) {
            case NONE:
                return "No human action required.";
            case PULL_CLARIFICATION:
                return "Clarify scope or evidence.";
            case PUSH_APPROVAL:
                return "Approve, edit, or reject the draft before official use.";
            case REVIEW_ONLY:
            default:
                return "Review cited evidence.";
        }
    }

    private List<String> evidenceLabels(List<AgentSourceCitation> citations) {
        List<String> labels = new ArrayList<>();
        for (AgentSourceCitation citation : citations == null ? Collections.<AgentSourceCitation>emptyList() : citations) {
            labels.add(value(citation.getSourceFamily(), "SRC") + ":" + value(citation.getSourceId(), citation.getLabel()));
        }
        return labels;
    }

    private List<String> counterEvidence(String text) {
        String normalized = combined(text);
        if (normalized.contains("CONTRADICT") || normalized.contains("CONFLICT")) {
            return Collections.singletonList("Conflicting evidence is present and requires operator review.");
        }
        if (normalized.contains("MISSING") || normalized.contains("AMBIGUOUS") || normalized.contains("UNSUPPORTED")) {
            return Collections.singletonList("Evidence is incomplete or ambiguous.");
        }
        return Collections.emptyList();
    }

    private String uncertainty(double confidence, List<AgentSourceCitation> citations) {
        if (citations == null || citations.isEmpty()) {
            return "High uncertainty: no citations are attached.";
        }
        if (confidence > 0.0 && confidence < 0.7) {
            return "Elevated uncertainty: confidence below 70%.";
        }
        return "Uncertainty bounded by cited local Airspace artifacts; not an autonomous operational clearance.";
    }

    private String combined(String... values) {
        StringBuilder builder = new StringBuilder();
        for (String value : values) {
            if (value != null) {
                builder.append(value).append(' ');
            }
        }
        return builder.toString().toUpperCase(Locale.US);
    }

    private String value(String primary, String fallback) {
        return primary == null || primary.trim().isEmpty() ? fallback : primary;
    }
}
