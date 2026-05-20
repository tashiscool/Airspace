package org.tash.extensions.notam;

import lombok.Builder;
import lombok.Data;
import org.tash.spatial.SpatialVolume;

import java.time.ZonedDateTime;

/**
 * Parsed airspace applicability from an ICAO-style NOTAM.
 */
@Data
@Builder
public class NotamAirspaceRestriction {
    private String id;
    private String notamType;
    private String accountability;
    private String affectedLocation;
    private String qCode;
    private String traffic;
    private String purpose;
    private String scope;
    private ZonedDateTime effectiveStart;
    private ZonedDateTime effectiveEnd;
    private double lowerAltitudeFeet;
    private double upperAltitudeFeet;
    private double centerLatitude;
    private double centerLongitude;
    private double radiusNauticalMiles;
    private String scheduleText;
    private String description;
    private SpatialVolume volume;
}
