package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class TfmAirportDemandSummary {
    private String airportId;
    private int offsetMinutes;
    private String timestamp;
    private int departureDemandPerHour;
    private int arrivalDemandPerHour;
    private int departureCapacityPerHour;
    private int arrivalCapacityPerHour;
    private double demandCapacityRatio;
    private int departureQueueDepth;
    private int arrivalQueueDepth;
    private int averageDelayMinutes;
    private String runwayConfiguration;
    private String status;
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
}
