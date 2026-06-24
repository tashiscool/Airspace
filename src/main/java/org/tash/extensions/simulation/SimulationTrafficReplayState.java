package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class SimulationTrafficReplayState {
    private String sourceMode;
    private String replaySourceId;
    private String providerFamily;
    private String providerReceiptId;
    private String authorizationMode;
    private int replayedAircraftCount;
    private int replayedFlightPlanCount;
    private int replayedPositionCount;
    private int airportDemandSnapshotCount;
    private int sectorDemandSnapshotCount;
    private int activeTrafficManagementInitiativeCount;
    private int activeTmiRecommendationCount;
    @Builder.Default
    private List<String> activeTmiTypes = new ArrayList<>();
    private boolean liveSwimNasDataUsed;
    private boolean fixtureBacked;
    private String rationale;
}
