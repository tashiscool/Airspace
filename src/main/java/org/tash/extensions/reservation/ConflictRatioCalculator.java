package org.tash.extensions.reservation;

import java.time.Duration;
import java.time.ZonedDateTime;

public class ConflictRatioCalculator {
    public double ratioFromTime(ZonedDateTime value, ZonedDateTime segmentStart, ZonedDateTime segmentEnd) {
        double segmentMillis = Duration.between(segmentStart, segmentEnd).toMillis();
        if (segmentMillis <= 0) {
            return 0;
        }
        double valueOffsetMillis = Duration.between(segmentStart, value).toMillis();
        return valueOffsetMillis / segmentMillis;
    }

    public ZonedDateTime timeFromRatio(double ratio, ZonedDateTime segmentStart, ZonedDateTime segmentEnd) {
        long segmentMillis = Duration.between(segmentStart, segmentEnd).toMillis();
        long valueOffset = Math.round(ratio * segmentMillis);
        return segmentStart.plusNanos(valueOffset * 1_000_000L);
    }

    public double avanaRatio(AirspaceReservation reservation) {
        return durationRatio(reservation.getAvanaMinutes(), reservation);
    }

    public double longitudinalRatio(AirspaceReservation reservation) {
        return durationRatio(reservation.getLongitudinalSeparationMinutes(), reservation);
    }

    public ConflictRatioWindow sourceWindow(double rawStartRatio,
                                            double rawEndRatio,
                                            AirspaceReservation reservation,
                                            boolean includeAvana,
                                            boolean includeLongitudinal) {
        double start = rawStartRatio;
        double end = rawEndRatio;
        if (includeAvana) {
            start -= avanaRatio(reservation);
        }
        if (includeLongitudinal) {
            double longitudinal = longitudinalRatio(reservation);
            start -= longitudinal;
            end += longitudinal;
        }
        return ConflictRatioWindow.builder()
                .rawStartRatio(rawStartRatio)
                .rawEndRatio(rawEndRatio)
                .factoredStartRatio(clamp(start))
                .factoredEndRatio(clamp(end))
                .build();
    }

    public ConflictRatioWindow sourceWindow(ZonedDateTime conflictStart,
                                            ZonedDateTime conflictEnd,
                                            AirspaceReservation reservation,
                                            boolean includeAvana,
                                            boolean includeLongitudinal) {
        return sourceWindow(
                ratioFromTime(conflictStart, reservation.getStartTime(), reservation.getEndTime()),
                ratioFromTime(conflictEnd, reservation.getStartTime(), reservation.getEndTime()),
                reservation,
                includeAvana,
                includeLongitudinal);
    }

    private double durationRatio(double minutes, AirspaceReservation reservation) {
        double routeMinutes = Duration.between(reservation.getStartTime(), reservation.getEndTime()).toMillis() / 60000.0;
        if (minutes <= 0 || routeMinutes <= 0) {
            return 0;
        }
        return minutes / routeMinutes;
    }

    private double clamp(double value) {
        return Math.max(0.0, Math.min(1.0, value));
    }
}
