package org.tash.extensions.reservation;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;
import org.tash.spatial.SpatialVolume;

import java.time.ZonedDateTime;
import java.util.List;

/**
 * CARF/FAA-style route reservation with dimensional separation requirements.
 */
@Data
@Builder
public class AirspaceReservation {
    private String id;
    private ZonedDateTime startTime;
    private ZonedDateTime endTime;
    private ZonedDateTime deconflictionEndTime;
    private double lowerAltitudeFeet;
    private double upperAltitudeFeet;
    private double deconflictionLowerAltitudeFeet;
    private double deconflictionUpperAltitudeFeet;
    private double verticalSeparationFeet;
    private double lateralSeparationNauticalMiles;
    private double longitudinalSeparationNauticalMiles;
    private double avanaMinutes;
    private double longitudinalSeparationMinutes;
    private double routeWidthNauticalMiles;
    private double routeSegmentDistanceNauticalMiles;
    private double sourceRatioStart;
    private double sourceRatioEnd;
    private String reservationType;
    private String routeStartFix;
    private String routeEndFix;
    private String sourceText;
    private String displayShapeIntent;
    private String deconflictionShapeIntent;
    private List<String> sourceFixes;
    private List<String> routeGraphNodeIds;
    private List<String> diagnostics;
    private GeoCoordinate routeStart;
    private GeoCoordinate routeEnd;
    private SpatialVolume protectedVolume;

    public ZonedDateTime getEffectiveDeconflictionEndTime() {
        return deconflictionEndTime != null ? deconflictionEndTime : endTime;
    }

    public ZonedDateTime getEffectiveConflictStartTime() {
        return startTime.minusMinutes((long) longitudinalSeparationMinutes);
    }

    public ZonedDateTime getEffectiveConflictEndTime() {
        return getEffectiveDeconflictionEndTime()
                .plusMinutes((long) avanaMinutes)
                .plusMinutes((long) longitudinalSeparationMinutes);
    }
}
