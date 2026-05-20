package org.tash.extensions.ops;

import org.tash.extensions.feed.OperationalFeedBatchResult;

import java.util.LinkedHashMap;
import java.util.Map;

public final class FeedIngestMetrics {
    public static final String MESSAGES_RECEIVED = "feed.messages.received";
    public static final String MESSAGES_ACCEPTED = "feed.messages.accepted";
    public static final String MESSAGES_REJECTED = "feed.messages.rejected";
    public static final String MESSAGES_CLASSIFIED_ONLY = "feed.messages.classified_only";

    private FeedIngestMetrics() {
    }

    public static void recordBatch(OperationalMetricSink sink, OperationalFeedBatchResult result) {
        if (sink == null || result == null) {
            return;
        }
        Map<String, String> tags = new LinkedHashMap<>();
        tags.put("sourceId", result.getSourceId() == null ? "unknown" : result.getSourceId());
        sink.record(metric(MESSAGES_RECEIVED, result.getResults().size(), tags));
        sink.record(metric(MESSAGES_ACCEPTED, result.acceptedCount(), tags));
        sink.record(metric(MESSAGES_REJECTED, result.rejectedCount(), tags));
        sink.record(metric(MESSAGES_CLASSIFIED_ONLY, result.classifiedOnlyCount(), tags));
    }

    private static OperationalMetric metric(String name, double value, Map<String, String> tags) {
        return OperationalMetric.builder()
                .name(name)
                .value(value)
                .unit("count")
                .tags(tags)
                .timestamp(java.time.ZonedDateTime.now(java.time.ZoneOffset.UTC))
                .build();
    }
}
