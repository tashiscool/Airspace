package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class TfmSectorLoadSummary {
    private String sectorId;
    private int offsetMinutes;
    private String timestamp;
    private int activeAircraft;
    private int baselineCapacity;
    private double workloadRatio;
    private int handoffQueueDepth;
    private double frequencyUtilization;
    private int estimatedHandoffDelaySeconds;
    private String status;
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
}
