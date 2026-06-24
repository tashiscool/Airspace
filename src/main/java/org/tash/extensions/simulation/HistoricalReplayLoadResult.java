package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class HistoricalReplayLoadResult {
    private String dayId;
    private String replayId;
    private String sourceMode;
    private String authorizationMode;
    private boolean accepted;
    private boolean ranSimulation;
    private String runId;
    private String finalAction;
    private int flightPlanCount;
    private int positionCount;
    private int airportDemandSnapshotCount;
    private int sectorDemandSnapshotCount;
    private int tmiCount;
    private int expectedOutcomeCount;
    private HistoricalReplayCalibrationReport calibrationReport;
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
    @Builder.Default
    private List<String> warnings = new ArrayList<>();
}
