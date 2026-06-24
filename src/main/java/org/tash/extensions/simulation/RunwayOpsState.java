package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class RunwayOpsState {
    private String airportId;
    private String runwayId;
    private double runwayLengthFeet;
    private int runwayVisualRangeFeet;
    private String brakingAction;
    private boolean runwayClosed;
    private boolean smgcsActive;
    private boolean lowVisibilityProcedureActive;
    private int departureRatePerHour;
    private int arrivalRatePerHour;
    private int queueDepth;
    private int surfaceDelaySeconds;
}
