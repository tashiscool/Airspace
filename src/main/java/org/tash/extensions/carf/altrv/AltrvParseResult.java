package org.tash.extensions.carf.altrv;

import lombok.Builder;
import lombok.Data;

import java.util.List;
import java.util.Collections;

@Data
@Builder
public class AltrvParseResult {
    private boolean accepted;
    private AltrvMessage message;
    private List<String> diagnostics;
    private List<AltrvSectionResult> sectionResults;
    private List<AltrvDiagnostic> typedDiagnostics;
    private List<AltrvSourceSpan> sourceSpans;

    public boolean isAccepted() {
        return accepted;
    }

    public List<String> getErrors() {
        if (typedDiagnostics == null) {
            return diagnostics == null ? Collections.emptyList() : diagnostics;
        }
        java.util.ArrayList<String> errors = new java.util.ArrayList<>();
        for (AltrvDiagnostic diagnostic : typedDiagnostics) {
            if (diagnostic.getSeverity() == AltrvDiagnosticSeverity.ERROR) {
                errors.add(diagnostic.getMessage());
            }
        }
        return errors;
    }

    public List<String> getWarnings() {
        if (typedDiagnostics == null) {
            return Collections.emptyList();
        }
        java.util.ArrayList<String> warnings = new java.util.ArrayList<>();
        for (AltrvDiagnostic diagnostic : typedDiagnostics) {
            if (diagnostic.getSeverity() == AltrvDiagnosticSeverity.WARNING) {
                warnings.add(diagnostic.getMessage());
            }
        }
        return warnings;
    }
}
