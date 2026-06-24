package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class SimulationPilotOperatorState {
    private String behaviorModel;
    private String pilotAction;
    private String controllerAction;
    private String operatorAction;
    private boolean humanApprovalRequired;
    private int communicationDelaySeconds;
    private double acceptanceProbability;
    private String rationale;
}
