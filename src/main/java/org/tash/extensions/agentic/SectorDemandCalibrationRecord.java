package org.tash.extensions.agentic;

import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;
import lombok.AllArgsConstructor;

@Data
@NoArgsConstructor
@AllArgsConstructor
@Builder
public class SectorDemandCalibrationRecord {
    private String sectorId;
    private double baselineCapacity;
    private double activeDemand;
    private double weatherCoverageRatio;

    public double demandRatio() {
        return baselineCapacity <= 0.0 ? 0.0 : activeDemand / baselineCapacity;
    }
}
