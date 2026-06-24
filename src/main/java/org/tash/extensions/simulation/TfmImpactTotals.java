package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class TfmImpactTotals {
    private int flightCount;
    private int activeFlightCount;
    private int airportCount;
    private int sectorCount;
    private int overloadedAirportCount;
    private int overloadedSectorCount;
    private double maxAirportDemandCapacityRatio;
    private double maxSectorDemandCapacityRatio;
    private int totalExpectedDelayMinutes;
    private int activeConstraintCount;
    private int proposedTmiCount;
    private int routeAlternativeCount;
    private int affectedFlightCount;
    private boolean humanApprovalRequired;
    private String sourceMode;
    private String sourceFreshnessStatus;
    private String commonOperatingPictureStatus;
}
