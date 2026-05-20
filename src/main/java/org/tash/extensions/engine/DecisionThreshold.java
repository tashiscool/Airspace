package org.tash.extensions.engine;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class DecisionThreshold {
    private final String name;
    private final String description;
    private final String comparator;
    private final double value;
    private final Double observedValue;
    private final String units;
}
