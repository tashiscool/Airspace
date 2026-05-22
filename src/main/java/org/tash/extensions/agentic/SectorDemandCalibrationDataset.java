package org.tash.extensions.agentic;

import java.util.List;

public interface SectorDemandCalibrationDataset {
    List<SectorDemandCalibrationRecord> records();

    default double averageDemandRatio() {
        return records().stream().mapToDouble(SectorDemandCalibrationRecord::demandRatio).average().orElse(0.0);
    }
}
