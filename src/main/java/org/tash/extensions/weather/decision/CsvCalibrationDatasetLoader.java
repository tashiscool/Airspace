package org.tash.extensions.weather.decision;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

public class CsvCalibrationDatasetLoader {
    public CalibrationDataset load(String id, String csv) {
        List<HistoricalWeatherOutcome> outcomes = new ArrayList<>();
        if (csv != null) {
            String[] lines = csv.split("\\R");
            for (int i = 0; i < lines.length; i++) {
                String line = lines[i].trim();
                if (line.isEmpty() || line.startsWith("#") || (i == 0 && line.toLowerCase().contains("product"))) {
                    continue;
                }
                String[] cells = line.split(",");
                if (cells.length < 6) {
                    continue;
                }
                outcomes.add(HistoricalWeatherOutcome.builder()
                        .productId(cells[0].trim())
                        .routeId(cells[1].trim())
                        .forecastTime(ZonedDateTime.parse(cells[2].trim()))
                        .routeBlocked(Boolean.parseBoolean(cells[3].trim()))
                        .observedDeviationRate(Double.parseDouble(cells[4].trim()))
                        .observedCapacityImpact(Double.parseDouble(cells[5].trim()))
                        .build());
            }
        }
        return InMemoryCalibrationDataset.builder().id(id).outcomes(outcomes).build();
    }
}
