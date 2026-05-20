package org.tash.extensions.reservation;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;

import java.time.ZonedDateTime;
import java.util.List;

@Data
@Builder
public class CarfReservationEvent {
    private CarfReservationEventType type;
    private ZonedDateTime startTime;
    private ZonedDateTime endTime;
    private double lowerAltitudeFeet;
    private double upperAltitudeFeet;
    private double protectedRadiusNauticalMiles;
    private double routeWidthNauticalMiles;
    private List<GeoCoordinate> points;
    private List<String> sourceFixes;
    private List<String> routeGraphNodeIds;
    private List<String> diagnostics;
    private double avanaMinutes;
    private double longitudinalSeparationMinutes;
    private String shapeIntent;
    private String sourceText;
}
