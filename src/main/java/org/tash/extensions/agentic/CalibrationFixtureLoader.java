package org.tash.extensions.agentic;

import jakarta.enterprise.context.ApplicationScoped;

import java.util.ArrayList;
import java.util.List;

@ApplicationScoped
public class CalibrationFixtureLoader {
    public WeatherOutcomeCalibrationDataset weatherOutcomesFromCsv(String csv) {
        List<WeatherOutcomeCalibrationRecord> records = new ArrayList<>();
        for (String line : lines(csv)) {
            String[] parts = line.split(",");
            if (parts.length < 5 || "productType".equalsIgnoreCase(parts[0].trim())) {
                continue;
            }
            records.add(WeatherOutcomeCalibrationRecord.builder()
                    .productType(parts[0].trim())
                    .severityScore(number(parts[1]))
                    .echoTopsFeet(number(parts[2]))
                    .leadTimeHours(number(parts[3]))
                    .blockedOutcome(number(parts[4]))
                    .build());
        }
        return () -> records;
    }

    public StormLifecycleCalibrationDataset stormLifecycleFromCsv(String csv) {
        List<StormLifecycleObservation> observations = new ArrayList<>();
        for (String line : lines(csv)) {
            String[] parts = line.split(",");
            if (parts.length < 5 || "cellId".equalsIgnoreCase(parts[0].trim())) {
                continue;
            }
            observations.add(StormLifecycleObservation.builder()
                    .cellId(parts[0].trim())
                    .phase(parts[1].trim())
                    .growthRate(number(parts[2]))
                    .movementKt(number(parts[3]))
                    .confidence(number(parts[4]))
                    .build());
        }
        return () -> observations;
    }

    public SectorDemandCalibrationDataset sectorDemandFromCsv(String csv) {
        List<SectorDemandCalibrationRecord> records = new ArrayList<>();
        for (String line : lines(csv)) {
            String[] parts = line.split(",");
            if (parts.length < 4 || "sectorId".equalsIgnoreCase(parts[0].trim())) {
                continue;
            }
            records.add(SectorDemandCalibrationRecord.builder()
                    .sectorId(parts[0].trim())
                    .baselineCapacity(number(parts[1]))
                    .activeDemand(number(parts[2]))
                    .weatherCoverageRatio(number(parts[3]))
                    .build());
        }
        return () -> records;
    }

    private List<String> lines(String csv) {
        List<String> lines = new ArrayList<>();
        if (csv == null) {
            return lines;
        }
        for (String line : csv.split("\\R")) {
            if (!line.trim().isEmpty() && !line.trim().startsWith("#")) {
                lines.add(line.trim());
            }
        }
        return lines;
    }

    private double number(String value) {
        try {
            return Double.parseDouble(value.trim());
        } catch (Exception ignored) {
            return 0.0;
        }
    }
}
