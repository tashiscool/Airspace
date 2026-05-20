package org.tash.extensions.ops;

import lombok.Builder;
import lombok.Data;

import java.time.ZonedDateTime;
import java.util.LinkedHashMap;
import java.util.Map;

@Data
@Builder
public class OperationalMetric {
    private final String name;
    private final double value;
    private final String unit;
    private final ZonedDateTime timestamp;
    @Builder.Default
    private final Map<String, String> tags = new LinkedHashMap<>();
}
