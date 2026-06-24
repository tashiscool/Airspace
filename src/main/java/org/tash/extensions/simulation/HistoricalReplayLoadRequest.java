package org.tash.extensions.simulation;

import lombok.Data;

@Data
public class HistoricalReplayLoadRequest {
    private String dayId;
    private String scenarioId;
    private String actor;
    private boolean strictValidation = true;
    private boolean runSimulation;
    private boolean includeCalibrationReport = true;
    private TrafficReplayBundle trafficReplay;
}
