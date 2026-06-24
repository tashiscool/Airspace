package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class TrafficFlowScenario {
    private String id;
    private String sourceMode;
    @Builder.Default
    private List<SimulatedAircraft> aircraft = new ArrayList<>();
    private String assumptions;
}
