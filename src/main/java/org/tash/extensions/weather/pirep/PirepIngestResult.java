package org.tash.extensions.weather.pirep;

import lombok.Builder;
import lombok.Data;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class PirepIngestResult {
    private final boolean accepted;
    private final PirepReport report;
    private final PirepDisseminationStatus disseminationStatus;
    @Builder.Default
    private final List<PirepDiagnostic> diagnostics = new ArrayList<>();

    public List<PirepDiagnostic> getDiagnostics() {
        return Collections.unmodifiableList(diagnostics);
    }

    public boolean hasDiagnostic(PirepDiagnosticType type) {
        return diagnostics.stream().anyMatch(d -> d.getType() == type);
    }
}
