package org.tash.extensions.carf.altrv;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class AltrvSourceSpan {
    private int startOffset;
    private int endOffset;
    private String text;

    public boolean contains(int offset) {
        return offset >= startOffset && offset <= endOffset;
    }
}
