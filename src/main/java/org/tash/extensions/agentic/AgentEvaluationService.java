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

    public AgentEvaluationSummary evaluate(AgentRunResult result, AgentPolicy policy) {
        if (result == null) {
            return AgentEvaluationSummary.builder()
                    .accepted(false)
                    .errors(java.util.Collections.singletonList("Agent result is missing"))
                    .build();
        }
        List<String> citationDiagnostics = citationValidator == null
                ? java.util.Collections.emptyList()
                : citationValidator.validate(result, policy);
        List<String> policyDiagnostics = policyEnforcer == null
                ? java.util.Collections.emptyList()
                : policyEnforcer.validate(result, policy);
        int totalClaims = result.getFindings().size() + result.getRecommendations().size() + result.getTasks().size();
        int uncited = citationDiagnostics.size();
        int cited = Math.max(0, totalClaims - uncited);
        Map<String, Integer> families = sourceFamilyCounts(result);
        List<String> warnings = new ArrayList<>();
        if (families.isEmpty()) {
            warnings.add("No source families were represented in agent citations");
        }
        List<String> errors = new ArrayList<>();
        errors.addAll(citationDiagnostics);
        errors.addAll(policyDiagnostics);
        return AgentEvaluationSummary.builder()
                .accepted(errors.isEmpty())
                .findingCount(result.getFindings().size())
                .recommendationCount(result.getRecommendations().size())
                .taskCount(result.getTasks().size())
                .deltaCount(result.getDeltas().size())
                .citedClaimCount(cited)
                .uncitedClaimCount(uncited)
                .policyViolationCount(policyDiagnostics.size())
                .citationCoverage(totalClaims == 0 ? 1.0 : (double) cited / (double) totalClaims)
                .sourceFamilyCounts(families)
                .warnings(warnings)
                .errors(errors)
                .build();
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
