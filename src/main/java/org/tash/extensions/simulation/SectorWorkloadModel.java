package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class SectorWorkloadModel {
    private String sectorId;
    private int activeAircraft;
    private int baselineCapacity;
    private double workloadRatio;
    private String capacityState;
    @Builder.Default
    private List<ControllerPositionState> controllerPositions = new ArrayList<>();
    private FrequencyCongestionState frequencyCongestion;
}
