package org.tash.extensions.engine;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class DecisionSourceRef {
    private final String type;
    private final String id;
    private final String description;
}
