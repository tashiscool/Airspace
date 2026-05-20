package org.tash.extensions.carf.altrv;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class AltrvExit {
    private String type;
    private String rawText;
    private AltrvSourceSpan sourceSpan;
}
