package org.tash.extensions.weather.product;

import lombok.Builder;
import lombok.Data;
import org.tash.extensions.weather.pirep.PirepReport;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class WeatherProductParseResult {
    private final boolean accepted;
    private final WeatherProduct product;
    private final PirepReport pirepReport;
    private final WeatherProductType classifiedType;
    private final String rawText;
    private final boolean classifiedOnly;
    @Builder.Default
    private final List<String> warnings = new ArrayList<>();
    @Builder.Default
    private final List<String> errors = new ArrayList<>();
    @Builder.Default
    private final List<WeatherParseDiagnostic> diagnostics = new ArrayList<>();
    @Builder.Default
    private final List<WeatherSourceSpan> sourceSpans = new ArrayList<>();

    public List<String> getWarnings() {
        return Collections.unmodifiableList(warnings);
    }

    public List<String> getErrors() {
        return Collections.unmodifiableList(errors);
    }

    public List<WeatherParseDiagnostic> getDiagnostics() {
        return Collections.unmodifiableList(diagnostics);
    }

    public List<WeatherSourceSpan> getSourceSpans() {
        return Collections.unmodifiableList(sourceSpans);
    }
}
