package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class AircraftTrajectoryState {
    private double latitude;
    private double longitude;
    private double altitudeFeet;
    private double groundSpeedKnots;
    private double routeProgress;
    private double delaySeconds;
    private String phase;
}
