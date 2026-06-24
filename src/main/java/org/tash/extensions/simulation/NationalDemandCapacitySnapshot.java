package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class NationalDemandCapacitySnapshot {
    private int minute;
    private int totalFlightCount;
    private int activeFlightCount;
    private int airportCount;
    private int sectorCount;
    private int totalAirportDepartureDemandPerHour;
    private int totalAirportArrivalDemandPerHour;
    private int totalAirportDepartureCapacityPerHour;
    private int totalAirportArrivalCapacityPerHour;
    private int overloadedAirportCount;
    private int overloadedSectorCount;
    private double maxAirportDemandCapacityRatio;
    private double maxSectorDemandCapacityRatio;
    private String busiestAirportId;
    private String busiestSectorId;
    private int totalExpectedDelayMinutes;
    private int generatedTmiRecommendationCount;
    @Builder.Default
    private List<TmiRecommendationModel> recommendations = new ArrayList<>();
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
}
