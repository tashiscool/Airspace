package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class FrequencyCongestionState {
    private String frequencyId;
    private double utilization;
    private int blockedTransmissions;
    private int averageWaitSeconds;
    private String congestionState;
}
