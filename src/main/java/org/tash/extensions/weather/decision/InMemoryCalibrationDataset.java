package org.tash.extensions.weather.decision;

import lombok.Builder;
import lombok.Data;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class InMemoryCalibrationDataset implements CalibrationDataset {
    private final String id;
    @Builder.Default
    private final List<HistoricalWeatherOutcome> outcomes = new ArrayList<>();

    @Override
    public List<HistoricalWeatherOutcome> outcomes() {
        return Collections.unmodifiableList(outcomes == null ? Collections.emptyList() : outcomes);
    }
}
