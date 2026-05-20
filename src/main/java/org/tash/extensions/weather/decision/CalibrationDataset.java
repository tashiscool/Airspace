package org.tash.extensions.weather.decision;

import java.util.List;

public interface CalibrationDataset {
    String getId();

    List<HistoricalWeatherOutcome> outcomes();
}
