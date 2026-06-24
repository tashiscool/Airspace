package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class AircraftPerformanceProfile {
    private AircraftClass aircraftClass;
    private double cruiseSpeedKnots;
    private double climbRateFeetPerMinute;
    private double descentRateFeetPerMinute;
    private double fuelBurnPoundsPerMinute;
    private double takeoffDistanceFeet;
    private double landingDistanceFeet;
    private double minimumRvrFeet;
    private double brakingActionPenaltyFactor;
    private String assumptions;
}
