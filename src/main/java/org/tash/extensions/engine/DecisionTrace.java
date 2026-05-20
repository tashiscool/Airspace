package org.tash.extensions.engine;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class DecisionTrace {
    private final List<DecisionTraceStep> steps = new ArrayList<>();

    public void add(String stage, String message, ZonedDateTime timestamp, DecisionSourceRef source) {
        List<DecisionSourceRef> sources = new ArrayList<>();
        if (source != null) {
            sources.add(source);
        }
        steps.add(DecisionTraceStep.builder()
                .stage(stage)
                .message(message)
                .timestamp(timestamp)
                .sources(sources)
                .build());
    }

    public void addDetailed(String stage,
                            String message,
                            ZonedDateTime timestamp,
                            List<DecisionSourceRef> sources,
                            List<DecisionSourceSpan> sourceSpans,
                            List<String> warningIds) {
        steps.add(DecisionTraceStep.builder()
                .stage(stage)
                .message(message)
                .timestamp(timestamp)
                .sources(sources == null ? Collections.emptyList() : sources)
                .sourceSpans(sourceSpans == null ? Collections.emptyList() : sourceSpans)
                .warningIds(warningIds == null ? Collections.emptyList() : warningIds)
                .build());
    }

    public void addRule(String stage,
                        String message,
                        ZonedDateTime timestamp,
                        DecisionRule rule,
                        String confidenceMath,
                        List<DecisionThreshold> thresholds,
                        List<String> warningIds,
                        List<DecisionSourceRef> sources) {
        steps.add(DecisionTraceStep.builder()
                .stage(stage)
                .message(message)
                .timestamp(timestamp)
                .rule(rule)
                .confidenceMath(confidenceMath)
                .thresholds(thresholds == null ? Collections.emptyList() : thresholds)
                .warningIds(warningIds == null ? Collections.emptyList() : warningIds)
                .sources(sources == null ? Collections.emptyList() : sources)
                .build());
    }

    public List<DecisionTraceStep> getSteps() {
        return Collections.unmodifiableList(steps);
    }
}
