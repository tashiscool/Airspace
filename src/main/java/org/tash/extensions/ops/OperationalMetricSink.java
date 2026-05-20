package org.tash.extensions.ops;

import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.Collections;
import java.util.Map;

public interface OperationalMetricSink {
    void record(OperationalMetric metric);

    default void increment(String name, Map<String, String> tags) {
        record(OperationalMetric.builder()
                .name(name)
                .value(1.0)
                .unit("count")
                .timestamp(ZonedDateTime.now(ZoneOffset.UTC))
                .tags(tags == null ? Collections.emptyMap() : tags)
                .build());
    }

    default void duration(String name, long millis, Map<String, String> tags) {
        record(OperationalMetric.builder()
                .name(name)
                .value(millis)
                .unit("milliseconds")
                .timestamp(ZonedDateTime.now(ZoneOffset.UTC))
                .tags(tags == null ? Collections.emptyMap() : tags)
                .build());
    }
}
