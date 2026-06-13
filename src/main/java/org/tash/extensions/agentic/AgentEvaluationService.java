package org.tash.extensions.agentic;

import jakarta.enterprise.context.ApplicationScoped;
import jakarta.inject.Inject;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

@ApplicationScoped
public class AgentEvaluationService {
    @Inject
    AgentCitationValidator citationValidator;
    @Inject
    AgentPolicyEnforcer policyEnforcer;
    @Inject
    AgentAssessmentBuilder assessmentBuilder;

    public AgentEvaluationSummary evaluate(AgentRunResult result, AgentPolicy policy) {
        if (result == null) {
            return AgentEvaluationSummary.builder()
                    .accepted(false)
                    .errors(java.util.Collections.singletonList("Agent result is missing"))
                    .build();
        }
        AgentRunResult normalized = assessmentBuilder == null ? new AgentAssessmentBuilder().normalize(result) : assessmentBuilder.normalize(result);
        List<String> citationDiagnostics = citationValidator == null
                ? java.util.Collections.emptyList()
                : citationValidator.validate(normalized, policy);
        List<String> policyDiagnostics = policyEnforcer == null
                ? java.util.Collections.emptyList()
                : policyEnforcer.validate(normalized, policy);
        List<String> assessmentDiagnostics = validateAssessments(normalized);
        int totalClaims = normalized.getFindings().size() + normalized.getRecommendations().size() + normalized.getTasks().size();
        int uncited = citationDiagnostics.size();
        int cited = Math.max(0, totalClaims - uncited);
        Map<String, Integer> families = sourceFamilyCounts(normalized);
        List<String> warnings = new ArrayList<>();
        if (families.isEmpty()) {
            warnings.add("No source families were represented in agent citations");
        }
        List<String> errors = new ArrayList<>();
        errors.addAll(citationDiagnostics);
        errors.addAll(policyDiagnostics);
        errors.addAll(assessmentDiagnostics);
        return AgentEvaluationSummary.builder()
                .accepted(errors.isEmpty())
                .findingCount(normalized.getFindings().size())
                .recommendationCount(normalized.getRecommendations().size())
                .taskCount(normalized.getTasks().size())
                .deltaCount(normalized.getDeltas().size())
                .citedClaimCount(cited)
                .uncitedClaimCount(uncited)
                .policyViolationCount(policyDiagnostics.size())
                .citationCoverage(totalClaims == 0 ? 1.0 : (double) cited / (double) totalClaims)
                .stabilityAccepted(true)
                .sourceFamilyCounts(families)
                .warnings(warnings)
                .errors(errors)
                .build();
    }

    private List<String> validateAssessments(AgentRunResult result) {
        List<String> diagnostics = new ArrayList<>();
        if (result.getAssessments() == null || result.getAssessments().isEmpty()) {
            if (!result.getFindings().isEmpty() || !result.getRecommendations().isEmpty() || !result.getTasks().isEmpty()) {
                diagnostics.add("Agent assessments are missing fixed-shape claim/verdict envelopes");
            }
            return diagnostics;
        }
        for (AgentAssessment assessment : result.getAssessments()) {
            String id = assessment.getId() == null ? "unknown" : assessment.getId();
            if (assessment.getClaim() == null || assessment.getClaim().trim().isEmpty()) {
                diagnostics.add("Agent assessment lacks claim: " + id);
            }
            if (assessment.getVerdict() == null || assessment.getVerdict().trim().isEmpty()) {
                diagnostics.add("Agent assessment lacks verdict: " + id);
            }
            if (assessment.getUncertainty() == null || assessment.getUncertainty().trim().isEmpty()) {
                diagnostics.add("Agent assessment lacks uncertainty: " + id);
            }
            if (assessment.getHumanReviewMode() == null) {
                diagnostics.add("Agent assessment lacks human-review mode: " + id);
            }
            if (assessment.getCitations() == null || assessment.getCitations().isEmpty()) {
                diagnostics.add("Agent assessment lacks citation: " + id);
            }
        }
        return diagnostics;
    }

    private Map<String, Integer> sourceFamilyCounts(AgentRunResult result) {
        Map<String, Integer> counts = new LinkedHashMap<>();
        for (AgentSourceCitation citation : result.getCitations()) {
            add(counts, citation);
        }
        for (AgentFinding finding : result.getFindings()) {
            finding.getCitations().forEach(citation -> add(counts, citation));
        }
        for (AgentRecommendation recommendation : result.getRecommendations()) {
            recommendation.getCitations().forEach(citation -> add(counts, citation));
        }
        for (AgentTask task : result.getTasks()) {
            task.getCitations().forEach(citation -> add(counts, citation));
        }
        for (AgentOperationalDelta delta : result.getDeltas()) {
            delta.getCitations().forEach(citation -> add(counts, citation));
        }
        return counts;
    }

    private void add(Map<String, Integer> counts, AgentSourceCitation citation) {
        if (citation == null || citation.getSourceFamily() == null || citation.getSourceFamily().trim().isEmpty()) {
            return;
        }
        String family = citation.getSourceFamily().trim().toUpperCase(Locale.US);
        counts.put(family, counts.getOrDefault(family, 0) + 1);
    }
}
