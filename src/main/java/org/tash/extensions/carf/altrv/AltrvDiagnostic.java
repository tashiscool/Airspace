package org.tash.extensions.carf.altrv;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class AltrvDiagnostic {
    private AltrvDiagnosticSeverity severity;
    private String message;
    private String sectionName;
    private String routeId;
    private String eventId;
    private AltrvSourceSpan sourceSpan;
}
