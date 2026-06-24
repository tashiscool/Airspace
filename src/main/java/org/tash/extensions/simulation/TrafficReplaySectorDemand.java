package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class TrafficReplaySectorDemand {
    private String sectorId;
    private String timestamp;
    private int offsetMinutes;
    private int activeAircraft;
    private int baselineCapacity;
    private int handoffQueueDepth;
    private double frequencyUtilization;
    private int estimatedHandoffDelaySeconds;
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
}
