package org.tash.extensions.reservation;

import org.tash.data.GeoCoordinate;

import java.time.Duration;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

/**
 * Three-dimensional reservation deconfliction.
 *
 * A conflict exists only when no configured dimension provides required
 * separation. If lateral separation is met, parallel routes are safe even when
 * their vertical ranges and time/longitudinal windows overlap.
 */
public class ReservationConflictDetector {
    private final double minimumConflictDurationSeconds;
    private final ConflictRatioCalculator ratioCalculator = new ConflictRatioCalculator();
    private final RouteSeparationGeometry routeGeometry = new RouteSeparationGeometry();

    public ReservationConflictDetector() {
        this(0);
    }

    public ReservationConflictDetector(double minimumConflictDurationSeconds) {
        this.minimumConflictDurationSeconds = Math.max(0, minimumConflictDurationSeconds);
    }

    public List<ReservationConflict> detectConflicts(List<AirspaceReservation> first,
                                                     List<AirspaceReservation> second) {
        List<ReservationConflict> conflicts = new ArrayList<>();
        for (AirspaceReservation a : first) {
            for (AirspaceReservation b : second) {
                ReservationConflict conflict = detectConflict(a, b);
                if (conflict != null) {
                    conflicts.add(conflict);
                }
            }
        }
        return conflicts;
    }

    public ReservationConflict detectConflict(AirspaceReservation first, AirspaceReservation second) {
        ZonedDateTime overlapStart = max(first.getEffectiveConflictStartTime(), second.getEffectiveConflictStartTime());
        ZonedDateTime overlapEnd = min(first.getEffectiveConflictEndTime(), second.getEffectiveConflictEndTime());
        if (overlapStart.isAfter(overlapEnd)) {
            return null;
        }
        double durationSeconds = Math.max(0, Duration.between(overlapStart, overlapEnd).toMillis() / 1000.0);
        if (durationSeconds < minimumConflictDurationSeconds) {
            return null;
        }

        double verticalGap = verticalGap(first, second);
        double requiredVertical = Math.max(first.getVerticalSeparationFeet(), second.getVerticalSeparationFeet());
        boolean verticalMet = isSeparationConfigured(requiredVertical) && verticalGap >= requiredVertical;
        if (verticalMet) {
            return null;
        }

        double lateralDistance = minimumRouteDistance(first, second);
        double requiredLateral = Math.max(first.getLateralSeparationNauticalMiles(),
                second.getLateralSeparationNauticalMiles());
        boolean lateralMet = isSeparationConfigured(requiredLateral) && lateralDistance >= requiredLateral;
        if (lateralMet) {
            return null;
        }

        double longitudinalGap = longitudinalGap(first, second);
        double requiredLongitudinal = Math.max(first.getLongitudinalSeparationNauticalMiles(),
                second.getLongitudinalSeparationNauticalMiles());
        boolean longitudinalMet = isSeparationConfigured(requiredLongitudinal) && longitudinalGap >= requiredLongitudinal;
        if (longitudinalMet) {
            return null;
        }

        ConflictRatioWindow firstWindow = ratioCalculator.sourceWindow(overlapStart, overlapEnd, first, true, true);
        ConflictRatioWindow secondWindow = ratioCalculator.sourceWindow(overlapStart, overlapEnd, second, true, true);
        GeoCoordinate firstStartPoint = pointAt(first.getRouteStart(), first.getRouteEnd(), firstWindow.getFactoredStartRatio());
        GeoCoordinate firstEndPoint = pointAt(first.getRouteStart(), first.getRouteEnd(), firstWindow.getFactoredEndRatio());
        GeoCoordinate secondStartPoint = pointAt(second.getRouteStart(), second.getRouteEnd(), secondWindow.getFactoredStartRatio());
        GeoCoordinate secondEndPoint = pointAt(second.getRouteStart(), second.getRouteEnd(), secondWindow.getFactoredEndRatio());

        return ReservationConflict.builder()
                .first(first)
                .second(second)
                .startTime(overlapStart)
                .endTime(overlapEnd)
                .minimumLateralDistanceNauticalMiles(lateralDistance)
                .longitudinalSeparationNauticalMiles(longitudinalGap)
                .verticalSeparationFeet(verticalGap)
                .requiredLateralSeparationNauticalMiles(requiredLateral)
                .requiredLongitudinalSeparationNauticalMiles(requiredLongitudinal)
                .requiredVerticalSeparationFeet(requiredVertical)
                .durationSeconds(durationSeconds)
                .firstConflictStartRatio(firstWindow.getRawStartRatio())
                .firstConflictEndRatio(firstWindow.getRawEndRatio())
                .secondConflictStartRatio(secondWindow.getRawStartRatio())
                .secondConflictEndRatio(secondWindow.getRawEndRatio())
                .firstFactoredStartRatio(firstWindow.getFactoredStartRatio())
                .firstFactoredEndRatio(firstWindow.getFactoredEndRatio())
                .secondFactoredStartRatio(secondWindow.getFactoredStartRatio())
                .secondFactoredEndRatio(secondWindow.getFactoredEndRatio())
                .firstStartPoint(firstStartPoint)
                .firstEndPoint(firstEndPoint)
                .secondStartPoint(secondStartPoint)
                .secondEndPoint(secondEndPoint)
                .distanceAtStartNauticalMiles(firstStartPoint.distanceTo(secondStartPoint))
                .distanceAtEndNauticalMiles(firstEndPoint.distanceTo(secondEndPoint))
                .angleBetweenRoutesDegrees(angleBetweenRoutes(first, second))
                .verticalSeparationMet(verticalMet)
                .lateralSeparationMet(lateralMet)
                .longitudinalSeparationMet(longitudinalMet)
                .belowMinimumDuration(durationSeconds < minimumConflictDurationSeconds)
                .explanation(explanation(first, second, durationSeconds, verticalGap, requiredVertical,
                        lateralDistance, requiredLateral, longitudinalGap, requiredLongitudinal,
                        firstStartPoint.distanceTo(secondStartPoint), firstEndPoint.distanceTo(secondEndPoint),
                        angleBetweenRoutes(first, second)))
                .build();
    }

    private double minimumRouteDistance(AirspaceReservation first, AirspaceReservation second) {
        return routeGeometry.segmentDistanceNauticalMiles(
                first.getRouteStart(), first.getRouteEnd(),
                second.getRouteStart(), second.getRouteEnd());
    }

    private double longitudinalGap(AirspaceReservation first, AirspaceReservation second) {
        return routeGeometry.longitudinalGapNauticalMiles(
                first.getRouteStart(), first.getRouteEnd(),
                second.getRouteStart(), second.getRouteEnd());
    }

    private double verticalGap(AirspaceReservation first, AirspaceReservation second) {
        double firstLower = first.getDeconflictionLowerAltitudeFeet();
        double firstUpper = first.getDeconflictionUpperAltitudeFeet();
        double secondLower = second.getDeconflictionLowerAltitudeFeet();
        double secondUpper = second.getDeconflictionUpperAltitudeFeet();

        if (firstUpper < secondLower) {
            return secondLower - firstUpper;
        }
        if (secondUpper < firstLower) {
            return firstLower - secondUpper;
        }
        return 0;
    }

    private GeoCoordinate pointAt(GeoCoordinate start, GeoCoordinate end, double fraction) {
        return start.interpolate(end, Math.max(0, Math.min(1, fraction)));
    }

    private double angleBetweenRoutes(AirspaceReservation first, AirspaceReservation second) {
        double firstBearing = first.getRouteStart().initialBearingTo(first.getRouteEnd());
        double secondBearing = second.getRouteStart().initialBearingTo(second.getRouteEnd());
        double diff = Math.abs(firstBearing - secondBearing) % 360.0;
        return diff > 180.0 ? 360.0 - diff : diff;
    }

    private boolean isSeparationConfigured(double value) {
        return value > 0 && !Double.isInfinite(value) && !Double.isNaN(value);
    }

    private String explanation(AirspaceReservation first,
                               AirspaceReservation second,
                               double durationSeconds,
                               double verticalGap,
                               double requiredVertical,
                               double lateralDistance,
                               double requiredLateral,
                               double longitudinalGap,
                               double requiredLongitudinal,
                               double distanceAtStart,
                               double distanceAtEnd,
                               double angleBetweenRoutes) {
        return "Conflict " + first.getId() + " vs " + second.getId()
                + ": duration=" + durationSeconds + "s"
                + ", verticalGap=" + verticalGap + "ft required=" + requiredVertical + "ft"
                + ", lateralDistance=" + lateralDistance + "NM required=" + requiredLateral + "NM"
                + ", longitudinalGap=" + longitudinalGap + "NM required=" + requiredLongitudinal + "NM"
                + ", distanceAtStart=" + distanceAtStart + "NM"
                + ", distanceAtEnd=" + distanceAtEnd + "NM"
                + ", angleBetweenRoutes=" + angleBetweenRoutes + "deg";
    }

    private ZonedDateTime max(ZonedDateTime a, ZonedDateTime b) {
        return a.isAfter(b) ? a : b;
    }

    private ZonedDateTime min(ZonedDateTime a, ZonedDateTime b) {
        return a.isBefore(b) ? a : b;
    }
}
