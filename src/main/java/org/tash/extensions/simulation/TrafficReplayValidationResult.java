package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class TrafficReplayValidationResult {
    private String replayId;
    private String sourceMode;
    private boolean accepted;
    private int flightPlanCount;
    private int positionCount;
    private int airportDemandSnapshotCount;
    private int sectorDemandSnapshotCount;
    private int trafficManagementInitiativeCount;
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
}
