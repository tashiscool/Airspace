package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class SimulatedAircraft {
    private String id;
    private String callsign;
    private AircraftClass aircraftClass;
    private AircraftPerformanceProfile performanceProfile;
    private FlightPlanIntent flightPlan;
    private AircraftTrajectoryState trajectory;
    private String rerouteAssignment;
    private boolean impacted;
    @Builder.Default
    private List<String> impactedSourceRefs = new ArrayList<>();
}
