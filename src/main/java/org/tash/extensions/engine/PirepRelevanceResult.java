package org.tash.extensions.engine;

import lombok.Builder;
import lombok.Data;
import org.tash.extensions.weather.pirep.PirepReport;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class PirepRelevanceResult {
    @Builder.Default private final List<PirepReport> relevant = new ArrayList<>();
    @Builder.Default private final List<String> diagnostics = new ArrayList<>();

    public List<PirepReport> getRelevant() {
        return Collections.unmodifiableList(relevant == null ? Collections.emptyList() : relevant);
    }

    public List<String> getDiagnostics() {
        return Collections.unmodifiableList(diagnostics == null ? Collections.emptyList() : diagnostics);
    }
}
