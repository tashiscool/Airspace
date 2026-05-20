package org.tash.extensions.engine;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class DecisionSourceSpan {
    private final String sourceId;
    private final String field;
    private final int startOffset;
    private final int endOffset;
    private final String text;
}
