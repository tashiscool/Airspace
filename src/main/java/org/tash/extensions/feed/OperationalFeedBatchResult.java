package org.tash.extensions.feed;

import lombok.Builder;
import lombok.Data;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class OperationalFeedBatchResult {
    private final String sourceId;
    private final ZonedDateTime receivedAt;
    @Builder.Default
    private final List<OperationalFeedIngestResult> results = new ArrayList<>();
    @Builder.Default
    private final List<String> diagnostics = new ArrayList<>();

    public int acceptedCount() {
        return (int) getResults().stream().filter(OperationalFeedIngestResult::isAccepted).count();
    }

    public int rejectedCount() {
        return getResults().size() - acceptedCount();
    }

    public int classifiedOnlyCount() {
        return (int) getResults().stream()
                .filter(result -> result.getWeatherProductResult() != null && result.getWeatherProductResult().isClassifiedOnly())
                .count();
    }

    public List<OperationalFeedIngestResult> getResults() {
        return Collections.unmodifiableList(results == null ? Collections.emptyList() : results);
    }

    public List<String> getDiagnostics() {
        return Collections.unmodifiableList(diagnostics == null ? Collections.emptyList() : diagnostics);
    }
}
