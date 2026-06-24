package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class ControllerPositionState {
    private String positionId;
    private String role;
    private boolean staffed;
    private int activeAircraft;
    private int handoffQueueDepth;
    private int coordinationDelaySeconds;
}
