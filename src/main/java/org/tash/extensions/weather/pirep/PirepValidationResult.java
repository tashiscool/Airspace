package org.tash.extensions.weather.pirep;

import lombok.Builder;
import lombok.Data;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class PirepValidationResult {
    private final boolean accepted;
    @Builder.Default
    private final List<String> errors = new ArrayList<>();
    @Builder.Default
    private final List<String> warnings = new ArrayList<>();
    @Builder.Default
    private final List<PirepDiagnostic> diagnostics = new ArrayList<>();

    public List<String> getErrors() {
        return Collections.unmodifiableList(errors);
    }

    public List<String> getWarnings() {
        return Collections.unmodifiableList(warnings);
    }

    public List<PirepDiagnostic> getDiagnostics() {
        return Collections.unmodifiableList(diagnostics);
    }

    public boolean hasDiagnostic(PirepDiagnosticType type) {
        return diagnostics.stream().anyMatch(d -> d.getType() == type);
    }
}
