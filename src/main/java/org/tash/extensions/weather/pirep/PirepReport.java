package org.tash.extensions.weather.pirep;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;

import java.time.ZonedDateTime;

@Data
@Builder(toBuilder = true)
public class PirepReport {
    private final String id;
    private final String aircraftId;
    private final String aircraftType;
    private final String reporter;
    private final ZonedDateTime observationTime;
    private final ZonedDateTime receivedTime;
    private final GeoCoordinate location;
    private final String locationText;
    private final Double altitudeFeet;
    private final PirepPhenomenon phenomenon;
    private final PirepIntensity intensity;
    private final boolean urgent;
    private final String remarks;
    private final String source;
    private final String rawText;
    private final String normalizedText;
    private final Double locationQuality;
    private final Double codingQuality;
    private final PirepDisseminationStatus disseminationStatus;
}
