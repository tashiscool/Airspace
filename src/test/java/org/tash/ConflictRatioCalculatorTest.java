package org.tash;

import org.junit.jupiter.api.Test;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.reservation.AirspaceReservation;
import org.tash.extensions.reservation.ConflictRatioCalculator;
import org.tash.extensions.reservation.ConflictRatioWindow;
import org.tash.extensions.reservation.ReservationConflict;
import org.tash.extensions.reservation.ReservationConflictDetector;

import java.time.ZonedDateTime;

import static org.junit.jupiter.api.Assertions.*;

class ConflictRatioCalculatorTest {
    private final ConflictRatioCalculator calculator = new ConflictRatioCalculator();

    @Test
    void convertsBetweenTimesAndRouteRatiosLikeLegacyActualConflict() {
        ZonedDateTime start = ZonedDateTime.parse("2010-04-10T15:00:00Z");
        ZonedDateTime end = ZonedDateTime.parse("2010-04-10T17:00:00Z");

        assertEquals(0.5, calculator.ratioFromTime(start.plusHours(1), start, end), 0.0001);
        assertEquals(start.plusMinutes(30), calculator.timeFromRatio(0.25, start, end));
    }

    @Test
    void factorsAvanaAndLongitudinalWindowsAndClampsToRouteSegment() {
        AirspaceReservation reservation = reservation("A",
                "2010-04-10T15:00:00Z",
                "2010-04-10T17:00:00Z",
                60,
                15);

        ConflictRatioWindow window = calculator.sourceWindow(0.5, 0.75, reservation, true, true);

        assertEquals(0.5, calculator.avanaRatio(reservation), 0.0001);
        assertEquals(0.125, calculator.longitudinalRatio(reservation), 0.0001);
        assertEquals(0.5, window.getRawStartRatio(), 0.0001);
        assertEquals(0.75, window.getRawEndRatio(), 0.0001);
        assertEquals(0.0, window.getFactoredStartRatio(), 0.0001);
        assertEquals(0.875, window.getFactoredEndRatio(), 0.0001);
    }

    @Test
    void detectorExposesRawAndFactoredConflictRatios() {
        AirspaceReservation first = reservation("A",
                "2010-04-10T15:00:00Z",
                "2010-04-10T17:00:00Z",
                60,
                15);
        AirspaceReservation second = reservation("B",
                "2010-04-10T16:00:00Z",
                "2010-04-10T18:00:00Z",
                0,
                0);

        ReservationConflict conflict = new ReservationConflictDetector().detectConflict(first, second);

        assertNotNull(conflict);
        assertEquals(0.5, conflict.getFirstConflictStartRatio(), 0.0001);
        assertEquals(1.5, conflict.getFirstConflictEndRatio(), 0.0001);
        assertEquals(0.0, conflict.getFirstFactoredStartRatio(), 0.0001);
        assertEquals(1.0, conflict.getFirstFactoredEndRatio(), 0.0001);
    }

    private AirspaceReservation reservation(String id,
                                            String start,
                                            String end,
                                            double avanaMinutes,
                                            double longitudinalMinutes) {
        return AirspaceReservation.builder()
                .id(id)
                .startTime(ZonedDateTime.parse(start))
                .endTime(ZonedDateTime.parse(end))
                .lowerAltitudeFeet(24000)
                .upperAltitudeFeet(26000)
                .deconflictionLowerAltitudeFeet(23750)
                .deconflictionUpperAltitudeFeet(26250)
                .verticalSeparationFeet(499)
                .lateralSeparationNauticalMiles(100)
                .longitudinalSeparationNauticalMiles(100)
                .avanaMinutes(avanaMinutes)
                .longitudinalSeparationMinutes(longitudinalMinutes)
                .routeStart(GeoCoordinate.builder().latitude(0).longitude(0).altitude(0).build())
                .routeEnd(GeoCoordinate.builder().latitude(0).longitude(1).altitude(0).build())
                .build();
    }
}
