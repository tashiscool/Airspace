package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class TrafficReplayPosition {
    private String flightId;
    private String timestamp;
    private int offsetMinutes;
    private double latitude;
    private double longitude;
    private double altitudeFeet;
    private double groundSpeedKnots;
    private double headingDegrees;
    private double routeProgress;
    private double delaySeconds;
    private String phase;
    private String trackingSourceType;
    private String trackingSourceLabel;
    private String providerTrackId;
    private String surveillanceQuality;
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
}
