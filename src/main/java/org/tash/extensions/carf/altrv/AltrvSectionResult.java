package org.tash.extensions.carf.altrv;

import lombok.Builder;
import lombok.Data;

import java.util.List;

@Data
@Builder
public class AltrvSectionResult {
    private String name;
    private String rawText;
    private AltrvSourceSpan sourceSpan;
    private boolean accepted;
    private List<AltrvDiagnostic> diagnostics;
}
