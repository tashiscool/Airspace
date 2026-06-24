package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class TrafficReplayAirportDemand {
    private String airportId;
    private String timestamp;
    private int offsetMinutes;
    private int departureDemandPerHour;
    private int arrivalDemandPerHour;
    private int departureCapacityPerHour;
    private int arrivalCapacityPerHour;
    private int departureQueueDepth;
    private int arrivalQueueDepth;
    private int averageDelaySeconds;
    private String runwayConfiguration;
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
}
