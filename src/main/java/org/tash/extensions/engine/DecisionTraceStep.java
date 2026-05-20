package org.tash.extensions.engine;

import lombok.Builder;
import lombok.Data;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class DecisionTraceStep {
    private final String stage;
    private final String message;
    private final ZonedDateTime timestamp;
    private final DecisionRule rule;
    private final String confidenceMath;
    @Builder.Default
    private final List<DecisionSourceRef> sources = new ArrayList<>();
    @Builder.Default
    private final List<DecisionThreshold> thresholds = new ArrayList<>();
    @Builder.Default
    private final List<String> warningIds = new ArrayList<>();
    @Builder.Default
    private final List<DecisionSourceSpan> sourceSpans = new ArrayList<>();

    public List<DecisionSourceRef> getSources() {
        return Collections.unmodifiableList(sources);
    }

    public List<DecisionThreshold> getThresholds() {
        return Collections.unmodifiableList(thresholds);
    }

    public List<String> getWarningIds() {
        return Collections.unmodifiableList(warningIds);
    }

    public List<DecisionSourceSpan> getSourceSpans() {
        return Collections.unmodifiableList(sourceSpans);
    }
}
