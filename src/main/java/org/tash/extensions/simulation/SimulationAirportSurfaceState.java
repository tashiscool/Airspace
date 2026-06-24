package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class SimulationAirportSurfaceState {
    private String airportId;
    private int runwayVisualRangeFeet;
    private String runwayId;
    private double runwayLengthFeet;
    private String brakingAction;
    private boolean smgcsActive;
    private boolean lowVisibilityProcedureActive;
    private boolean terminologyAmbiguity;
    private int departureQueueDepth;
    private int surfaceDelaySeconds;
    private String rationale;
}
