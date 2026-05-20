package org.tash.extensions.carf.altrv;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class AltrvToken {
    private AltrvTokenType type;
    private String text;
    private int offset;
    private int endOffset;
    private AltrvSourceSpan sourceSpan;
}
