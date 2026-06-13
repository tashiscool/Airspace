package org.tash.extensions.agentic.stability;

import jakarta.enterprise.context.ApplicationScoped;
import org.tash.extensions.agentic.AgentFinding;
import org.tash.extensions.agentic.AgentRecommendation;
import org.tash.extensions.agentic.AgentRunResult;
import org.tash.extensions.agentic.AgentSourceCitation;
import org.tash.extensions.agentic.AgentTask;
import org.tash.extensions.agentic.AgentToolCall;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;

@ApplicationScoped
public class AgentRunComparator {
    public List<AgentStabilityMetric> compare(List<AgentRunResult> runs,
                                              double minAgreement,
                                              double minCitationJaccard,
                                              double maxCountCoefficientOfVariation) {
        List<AgentRunResult> safeRuns = runs == null ? Collections.emptyList() : runs;
        List<AgentStabilityMetric> metrics = new ArrayList<>();
        metrics.add(agreement("accepted_agreement", "Accepted/verdict agreement", acceptedValues(safeRuns), minAgreement));
        metrics.add(countCv("finding_count_wobble", "Finding count wobble", counts(safeRuns, "finding"), maxCountCoefficientOfVariation));
        metrics.add(countCv("recommendation_count_wobble", "Recommendation count wobble", counts(safeRuns, "recommendation"), maxCountCoefficientOfVariation));
        metrics.add(countCv("task_count_wobble", "Task count wobble", counts(safeRuns, "task"), maxCountCoefficientOfVariation));
        metrics.add(jaccard("cited_source_jaccard", "Cited-source Jaccard", citationSets(safeRuns), minCitationJaccard));
        metrics.add(jaccard("finding_category_agreement", "Finding category agreement", findingCategorySets(safeRuns), minAgreement));
        metrics.add(jaccard("recommendation_action_agreement", "Recommendation action agreement", recommendationActionSets(safeRuns), minAgreement));
        metrics.add(jaccard("task_route_priority_agreement", "Task route/priority agreement", taskRoutePrioritySets(safeRuns), minAgreement));
        metrics.add(jaccard("tool_receipt_hash_agreement", "Tool receipt hash agreement", toolHashSets(safeRuns), minAgreement));
        return metrics;
    }

    private AgentStabilityMetric agreement(String id, String name, List<String> values, double threshold) {
        if (values.isEmpty()) {
            return metric(id, name, 1.0, threshold, true, "No runs to compare.");
        }
        String first = values.get(0);
        long matching = values.stream().filter(first::equals).count();
        double value = (double) matching / (double) values.size();
        return metric(id, name, value, threshold, value >= threshold,
                matching + "/" + values.size() + " runs matched the first non-volatile value.");
    }

    private AgentStabilityMetric countCv(String id, String name, List<Integer> values, double threshold) {
        double cv = coefficientOfVariation(values);
        return metric(id, name, cv, threshold, cv <= threshold,
                "Counts=" + values + "; lower is more stable.");
    }

    private AgentStabilityMetric jaccard(String id, String name, List<Set<String>> sets, double threshold) {
        double value = averagePairwiseJaccard(sets);
        return metric(id, name, value, threshold, value >= threshold,
                "Compared " + sets.size() + " non-volatile set(s).");
    }

    private AgentStabilityMetric metric(String id, String name, double value, double threshold, boolean accepted, String details) {
        return AgentStabilityMetric.builder()
                .id(id)
                .name(name)
                .value(value)
                .threshold(threshold)
                .accepted(accepted)
                .details(details)
                .build();
    }

    private List<String> acceptedValues(List<AgentRunResult> runs) {
        List<String> values = new ArrayList<>();
        for (AgentRunResult run : runs) {
            values.add(String.valueOf(run.isAccepted()));
        }
        return values;
    }

    private List<Integer> counts(List<AgentRunResult> runs, String type) {
        List<Integer> values = new ArrayList<>();
        for (AgentRunResult run : runs) {
            switch (type) {
                case "finding":
                    values.add(run.getFindings().size());
                    break;
                case "recommendation":
                    values.add(run.getRecommendations().size());
                    break;
                case "task":
                    values.add(run.getTasks().size());
                    break;
                default:
                    values.add(0);
            }
        }
        return values;
    }

    private List<Set<String>> citationSets(List<AgentRunResult> runs) {
        List<Set<String>> values = new ArrayList<>();
        for (AgentRunResult run : runs) {
            Set<String> citations = new LinkedHashSet<>();
            collect(citations, run.getCitations());
            run.getFindings().forEach(finding -> collect(citations, finding.getCitations()));
            run.getRecommendations().forEach(recommendation -> collect(citations, recommendation.getCitations()));
            run.getTasks().forEach(task -> collect(citations, task.getCitations()));
            run.getDeltas().forEach(delta -> collect(citations, delta.getCitations()));
            values.add(citations);
        }
        return values;
    }

    private List<Set<String>> findingCategorySets(List<AgentRunResult> runs) {
        List<Set<String>> values = new ArrayList<>();
        for (AgentRunResult run : runs) {
            Set<String> set = new LinkedHashSet<>();
            for (AgentFinding finding : run.getFindings()) {
                set.add(normalize(finding.getCategory()) + ":" + normalize(finding.getSeverity()));
            }
            values.add(set);
        }
        return values;
    }

    private List<Set<String>> recommendationActionSets(List<AgentRunResult> runs) {
        List<Set<String>> values = new ArrayList<>();
        for (AgentRunResult run : runs) {
            Set<String> set = new LinkedHashSet<>();
            for (AgentRecommendation recommendation : run.getRecommendations()) {
                set.add(normalize(recommendation.getAction()) + ":" + normalize(String.valueOf(recommendation.getHumanReviewMode())));
            }
            values.add(set);
        }
        return values;
    }

    private List<Set<String>> taskRoutePrioritySets(List<AgentRunResult> runs) {
        List<Set<String>> values = new ArrayList<>();
        for (AgentRunResult run : runs) {
            Set<String> set = new LinkedHashSet<>();
            for (AgentTask task : run.getTasks()) {
                set.add(normalize(task.getRoute()) + ":" + normalize(task.getPriority()) + ":" + normalize(String.valueOf(task.getHumanReviewMode())));
            }
            values.add(set);
        }
        return values;
    }

    private List<Set<String>> toolHashSets(List<AgentRunResult> runs) {
        List<Set<String>> values = new ArrayList<>();
        for (AgentRunResult run : runs) {
            Set<String> set = new LinkedHashSet<>();
            for (AgentToolCall call : run.getToolCalls()) {
                set.add(normalize(call.getToolName()) + ":" + normalize(call.getInputHash()) + ":" + normalize(call.getOutputHash()));
            }
            values.add(set);
        }
        return values;
    }

    private void collect(Set<String> values, List<AgentSourceCitation> citations) {
        for (AgentSourceCitation citation : citations == null ? Collections.<AgentSourceCitation>emptyList() : citations) {
            values.add(normalize(citation.getSourceFamily()) + ":" + normalize(citation.getSourceId()));
        }
    }

    private double coefficientOfVariation(List<Integer> values) {
        if (values == null || values.isEmpty()) {
            return 0.0;
        }
        double mean = values.stream().mapToDouble(Integer::doubleValue).average().orElse(0.0);
        if (mean == 0.0) {
            return values.stream().allMatch(value -> value == 0) ? 0.0 : 1.0;
        }
        double variance = values.stream()
                .mapToDouble(value -> Math.pow(value - mean, 2.0))
                .average()
                .orElse(0.0);
        return Math.sqrt(variance) / mean;
    }

    private double averagePairwiseJaccard(List<Set<String>> sets) {
        if (sets == null || sets.size() < 2) {
            return 1.0;
        }
        double total = 0.0;
        int pairs = 0;
        for (int i = 0; i < sets.size(); i++) {
            for (int j = i + 1; j < sets.size(); j++) {
                total += jaccard(sets.get(i), sets.get(j));
                pairs++;
            }
        }
        return pairs == 0 ? 1.0 : total / pairs;
    }

    private double jaccard(Set<String> first, Set<String> second) {
        if ((first == null || first.isEmpty()) && (second == null || second.isEmpty())) {
            return 1.0;
        }
        Set<String> union = new LinkedHashSet<>();
        if (first != null) {
            union.addAll(first);
        }
        if (second != null) {
            union.addAll(second);
        }
        Set<String> intersection = new LinkedHashSet<>();
        if (first != null) {
            intersection.addAll(first);
        }
        if (second == null) {
            intersection.clear();
        } else {
            intersection.retainAll(second);
        }
        return union.isEmpty() ? 1.0 : (double) intersection.size() / (double) union.size();
    }

    private String normalize(String value) {
        return value == null ? "none" : value.trim().toUpperCase(Locale.US);
    }
}
