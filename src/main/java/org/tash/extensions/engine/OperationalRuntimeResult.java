package org.tash.extensions.engine;

import lombok.Builder;
import lombok.Data;
import org.tash.extensions.feed.OperationalFeedBatchResult;

import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@Data
@Builder
public class OperationalRuntimeResult {
    @Builder.Default
    private final List<OperationalFeedBatchResult> feedBatchResults = new ArrayList<>();
    private final OperationalDecisionResult decisionResult;
    @Builder.Default
    private final Map<String, String> persistedArtifactIds = new LinkedHashMap<>();
    @Builder.Default
    private final Map<String, Double> metricsSummary = new LinkedHashMap<>();
    private final boolean accepted;

    public List<OperationalFeedBatchResult> getFeedBatchResults() {
        return Collections.unmodifiableList(feedBatchResults == null ? Collections.emptyList() : feedBatchResults);
    }
}
