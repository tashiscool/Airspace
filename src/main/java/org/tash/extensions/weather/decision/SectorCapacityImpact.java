package org.tash.extensions.weather.decision;

import lombok.Builder;
import lombok.Data;

@Data
@Builder
public class SectorCapacityImpact {
    private final String sectorId;
    private final double demandCapacityRatio;
    private final double capacityImpact;
    private final String rationale;
}
