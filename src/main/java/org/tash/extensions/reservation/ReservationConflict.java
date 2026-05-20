package org.tash.extensions.reservation;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;

import java.time.ZonedDateTime;

@Data
@Builder
public class ReservationConflict {
    private AirspaceReservation first;
    private AirspaceReservation second;
    private ZonedDateTime startTime;
    private ZonedDateTime endTime;
    private double minimumLateralDistanceNauticalMiles;
    private double longitudinalSeparationNauticalMiles;
    private double verticalSeparationFeet;
    private double requiredLateralSeparationNauticalMiles;
    private double requiredLongitudinalSeparationNauticalMiles;
    private double requiredVerticalSeparationFeet;
    private double durationSeconds;
    private double firstConflictStartRatio;
    private double firstConflictEndRatio;
    private double secondConflictStartRatio;
    private double secondConflictEndRatio;
    private double firstFactoredStartRatio;
    private double firstFactoredEndRatio;
    private double secondFactoredStartRatio;
    private double secondFactoredEndRatio;
    private GeoCoordinate firstStartPoint;
    private GeoCoordinate firstEndPoint;
    private GeoCoordinate secondStartPoint;
    private GeoCoordinate secondEndPoint;
    private double distanceAtStartNauticalMiles;
    private double distanceAtEndNauticalMiles;
    private double angleBetweenRoutesDegrees;
    private String explanation;
    private boolean verticalSeparationMet;
    private boolean lateralSeparationMet;
    private boolean longitudinalSeparationMet;
    private boolean belowMinimumDuration;
}
