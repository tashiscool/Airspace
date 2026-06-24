package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class TrafficReplayFlightPlan {
    private String flightId;
    private String callsign;
    private AircraftClass aircraftClass;
    private String operator;
    private String origin;
    private String destination;
    private String filedRouteText;
    @Builder.Default
    private List<List<Double>> filedRoutePoints = new ArrayList<>();
    private String scheduledDepartureTime;
    private String scheduledArrivalTime;
    private String estimatedDepartureTime;
    private String estimatedArrivalTime;
    private String actualDepartureTime;
    private String actualArrivalTime;
    private Integer requestedAltitudeFeet;
    private String requestedAltitudeBlock;
    private Integer filedSpeedKnots;
    private String missionId;
    private String reservationId;
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
}
