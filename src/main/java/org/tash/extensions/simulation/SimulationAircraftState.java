package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class SimulationAircraftState {
    private String aircraftId;
    private double latitude;
    private double longitude;
    private double altitudeFeet;
    private double groundSpeedKnots;
    private double climbRateFeetPerMinute;
    private double fuelRemainingPounds;
    private double estimatedFuelBurnPounds;
    private double runwayRequiredFeet;
    private double runwayAvailableFeet;
    private boolean takeoffPerformanceAcceptable;
    private String performancePhase;
    private String rationale;
}
