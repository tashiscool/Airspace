package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class SimulationKpiSummary {
    private long timeToGuidanceSeconds;
    private int falseClearCount;
    private int falseBlockCount;
    private double sourceRefPreservationRate;
    private double rerouteFoundRate;
    private int staleDataDowngradeCount;
    private long coordinationDraftLatencySeconds;
    private boolean pilotBriefAvailable;
    private double replayVerificationPassRate;
    private int minuteStepCount;
    private int aircraftStateUpdateCount;
    private double peakSectorWorkloadRatio;
    private int maxHandoffDelaySeconds;
    private int maxSurfaceDelaySeconds;
    private int stochasticEnsembleMemberCount;
    private int behaviorDecisionCount;
    private int trafficReplayAircraftCount;
    private int nationalFlightCount;
    private int overloadedAirportCount;
    private int overloadedSectorCount;
    private double peakAirportDemandCapacityRatio;
    private double peakNationalSectorDemandCapacityRatio;
    private int nationalTmiRecommendationCount;
    private int simulatedDayCount;
    private double baselineDelayMinutes;
    private double mitigatedDelayMinutes;
    private double delayMinutesSaved;
    private double rerouteMiles;
    private double additionalFuelPounds;
    private double holdingFuelSavedPounds;
    private double fuelImpactPounds;
    private int sectorOverloadAvoidedCount;
    private double sourceRefCompletenessRate;
    private long operatorTimeToDecisionSeconds;
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
}
