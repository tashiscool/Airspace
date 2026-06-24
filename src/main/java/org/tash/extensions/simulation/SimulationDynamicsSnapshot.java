package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class SimulationDynamicsSnapshot {
    private int minute;
    private SimulationAircraftState aircraft;
    private SimulationAirportSurfaceState airportSurface;
    private SimulationSectorWorkloadState sectorWorkload;
    private SimulationPilotOperatorState pilotOperator;
    private SimulationWeatherEvolutionState weatherEvolution;
    private SimulationTrafficReplayState trafficReplay;
    private NationalDemandCapacitySnapshot nationalDemandCapacity;
    @Builder.Default
    private List<TmiRecommendationModel> trafficManagementRecommendations = new ArrayList<>();
    @Builder.Default
    private List<String> assumptions = new ArrayList<>();
}
