package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class SimulationSectorWorkloadState {
    private String sectorId;
    private int activeAircraft;
    private int baselineCapacity;
    private int controllerPositionsStaffed;
    private int handoffQueueDepth;
    private int estimatedHandoffDelaySeconds;
    private double frequencyCongestion;
    private double workloadRatio;
    private String capacityState;
    private String rationale;
}
