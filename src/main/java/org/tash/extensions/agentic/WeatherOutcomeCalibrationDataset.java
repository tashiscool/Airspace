package org.tash.extensions.agentic;

import java.util.List;

public interface WeatherOutcomeCalibrationDataset {
    List<WeatherOutcomeCalibrationRecord> records();

    default double averageBlockedOutcome() {
        return records().stream().mapToDouble(WeatherOutcomeCalibrationRecord::getBlockedOutcome).average().orElse(0.0);
    }
}
