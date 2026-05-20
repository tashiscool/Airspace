package org.tash;

import org.junit.jupiter.api.Test;
import org.tash.extensions.reservation.AirspaceReservation;
import org.tash.extensions.reservation.CarfRouteMessage;
import org.tash.extensions.reservation.CarfRouteMessageParser;
import org.tash.extensions.reservation.ReservationConflict;
import org.tash.extensions.reservation.ReservationConflictDetector;
import org.tash.extensions.reservation.ReservationMessageParser;

import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import static org.junit.jupiter.api.Assertions.*;

class ReservationConflictDetectorTest {
    @Test
    void doesNotConflictParallelRoutesWhenLateralSeparationIsMet() {
        ReservationMessageParser parser = new ReservationMessageParser();
        List<AirspaceReservation> message1 = parser.parse("MESSAGE1", message(
                reservation(0, "15:00:00", "16:00:00", "17:00:00", "0600000W 020060N", "0400000W 020060N", "1201.4070850217033"),
                reservation(1, "16:00:00", "17:00:00", "18:00:00", "0400000W 020060N", "0300000W 020060N", "600.7064189629257"),
                reservation(2, "17:00:00", "18:00:00", "19:00:00", "0300000W 020060N", "0200000W 020060N", "600.7064189629257"),
                reservation(3, "18:00:00", "19:00:00", "20:00:00", "0200000W 020060N", "0100000W 020060N", "600.7064189629257"),
                reservation(4, "19:00:00", "20:00:00", "21:00:00", "0100000W 020060N", "0000000E 020060N", "600.7064189629257")
        ));
        List<AirspaceReservation> message2 = parser.parse("MESSAGE2", message(
                reservation(0, "15:00:00", "16:00:00", "17:00:00", "0600000W 000000N", "0400000W 000000N", "1202.1543270545649"),
                reservation(1, "16:00:00", "17:00:00", "18:00:00", "0400000W 000000N", "0300000W 000000N", "601.0771635272827"),
                reservation(2, "17:00:00", "18:00:00", "19:00:00", "0300000W 000000N", "0200000W 000000N", "601.0771635272827"),
                reservation(3, "18:00:00", "19:00:00", "20:00:00", "0200000W 000000N", "0100000W 000000N", "601.0771635272827"),
                reservation(4, "19:00:00", "20:00:00", "21:00:00", "0100000W 000000N", "0000000E 000000N", "601.0771635272827")
        ));

        List<ReservationConflict> conflicts =
                new ReservationConflictDetector().detectConflicts(message1, message2);

        assertTrue(conflicts.isEmpty(), "parallel routes separated by about 120.4 NM should not conflict");
    }

    @Test
    void conflictsWhenNoDimensionMeetsRequiredSeparation() {
        ReservationMessageParser parser = new ReservationMessageParser();
        List<AirspaceReservation> message1 = parser.parse("MESSAGE1", message(
                reservation(0, "15:00:00", "16:00:00", "17:00:00", "0600000W 020060N", "0400000W 020060N", "1201.4070850217033")
        ));
        List<AirspaceReservation> message2 = parser.parse("MESSAGE2", message(
                reservation(0, "15:30:00", "16:30:00", "17:30:00", "0600000W 020060N", "0400000W 020060N", "1201.4070850217033")
        ));

        List<ReservationConflict> conflicts =
                new ReservationConflictDetector().detectConflicts(message1, message2);

        assertEquals(1, conflicts.size());
        assertTrue(conflicts.get(0).getMinimumLateralDistanceNauticalMiles() < 1.0);
        assertEquals(0.0, conflicts.get(0).getVerticalSeparationFeet(), 0.0001);
        assertNotNull(conflicts.get(0).getFirstStartPoint());
        assertNotNull(conflicts.get(0).getSecondStartPoint());
        assertTrue(conflicts.get(0).getDistanceAtStartNauticalMiles() >= 0.0);
        assertTrue(conflicts.get(0).getDistanceAtEndNauticalMiles() >= 0.0);
        assertEquals(0.0, conflicts.get(0).getAngleBetweenRoutesDegrees(), 0.0001);
        assertTrue(conflicts.get(0).getExplanation().contains("distanceAtStart"));
    }

    @Test
    void detectsHorizontalReservationConflictsFromCarfHorSepMeters() {
        ReservationMessageParser parser = new ReservationMessageParser();
        List<AirspaceReservation> message1 = parser.parse("MESSAGE1", message(
                horReservation(0, "13:00:00", "15:00:00", "FL240B260", "23750.5-26249.5", "1500000W 300000N", "1600000W 300000N"),
                horReservation(1, "14:00:00", "16:00:00", "FL240B260", "23750.5-26249.5", "1600000W 300000N", "1700000W 300000N"),
                horReservation(2, "15:00:00", "17:00:00", "FL240B260", "23750.5-26249.5", "1700000W 300000N", "1800000W 300000N")
        ));
        List<AirspaceReservation> message2 = parser.parse("MESSAGE2", message(
                horReservation(0, "15:00:00", "17:00:00", "FL240B260", "23750.5-26249.5", "1500000W 310000N", "1600000W 310000N"),
                horReservation(1, "16:00:00", "18:00:00", "FL240B280", "23750.5-28249.5", "1600000W 310000N", "1700000W 310000N"),
                horReservation(2, "17:00:00", "19:00:00", "FL270B280", "26750.5-28249.5", "1700000W 310000N", "1800000W 310000N")
        ));

        List<ReservationConflict> conflicts =
                new ReservationConflictDetector().detectConflicts(message1, message2);

        assertEquals(4, conflicts.size());
        assertConflict(conflicts.get(0), "MESSAGE1-R0", "MESSAGE2-R0");
        assertConflict(conflicts.get(1), "MESSAGE1-R1", "MESSAGE2-R0");
        assertConflict(conflicts.get(2), "MESSAGE1-R1", "MESSAGE2-R1");
        assertConflict(conflicts.get(3), "MESSAGE1-R2", "MESSAGE2-R1");
        assertTrue(conflicts.stream().allMatch(c -> c.getMinimumLateralDistanceNauticalMiles() < 100.0));
    }

    @Test
    void longitudinalMinutesExpandConflictTimeWindow() {
        AirspaceReservation first = AirspaceReservation.builder()
                .id("A")
                .startTime(java.time.ZonedDateTime.parse("2010-08-16T22:00:00Z"))
                .endTime(java.time.ZonedDateTime.parse("2010-08-16T22:10:00Z"))
                .lowerAltitudeFeet(24000)
                .upperAltitudeFeet(26000)
                .deconflictionLowerAltitudeFeet(23750.5)
                .deconflictionUpperAltitudeFeet(26249.5)
                .verticalSeparationFeet(499)
                .lateralSeparationNauticalMiles(100)
                .longitudinalSeparationMinutes(30)
                .routeStart(org.tash.data.GeoCoordinate.builder().latitude(0).longitude(0).altitude(0).build())
                .routeEnd(org.tash.data.GeoCoordinate.builder().latitude(1).longitude(0).altitude(0).build())
                .build();
        AirspaceReservation second = AirspaceReservation.builder()
                .id("B")
                .startTime(java.time.ZonedDateTime.parse("2010-08-16T22:35:00Z"))
                .endTime(java.time.ZonedDateTime.parse("2010-08-16T22:45:00Z"))
                .lowerAltitudeFeet(24000)
                .upperAltitudeFeet(26000)
                .deconflictionLowerAltitudeFeet(23750.5)
                .deconflictionUpperAltitudeFeet(26249.5)
                .verticalSeparationFeet(499)
                .lateralSeparationNauticalMiles(100)
                .longitudinalSeparationMinutes(30)
                .routeStart(org.tash.data.GeoCoordinate.builder().latitude(0).longitude(0).altitude(0).build())
                .routeEnd(org.tash.data.GeoCoordinate.builder().latitude(1).longitude(0).altitude(0).build())
                .build();

        ReservationConflict conflict = new ReservationConflictDetector().detectConflict(first, second);

        assertNotNull(conflict);
        assertEquals("2010-08-16T22:05Z", conflict.getStartTime().toString());
        assertTrue(conflict.getExplanation().contains("duration"));
    }

    @Test
    void parsesRawCarfTimingMessagesIntoEquivalentReservations() throws Exception {
        CarfRouteMessageParser parser = new CarfRouteMessageParser();
        CarfRouteMessage deal = parser.parse(rawDealMessage());
        CarfRouteMessage woody = parser.parse(rawWoodyMessage());

        assertEquals("DEAL01-02", deal.getActivityName());
        assertEquals(450, deal.getTrueAirspeedKnots());
        assertEquals(3, deal.getReservations().size());
        assertEquals(24000.0, deal.getReservations().get(0).getLowerAltitudeFeet(), 0.0001);
        assertEquals(28000.0, deal.getReservations().get(1).getUpperAltitudeFeet(), 0.0001);
        assertEquals(27000.0, deal.getReservations().get(2).getLowerAltitudeFeet(), 0.0001);
        assertEquals("2010-03-02T15:00-05:00[America/New_York]",
                deal.getReservations().get(0).getStartTime().toString());
        assertEquals("2010-03-02T17:00-05:00[America/New_York]",
                deal.getReservations().get(0).getEndTime().toString());

        List<ReservationConflict> conflicts =
                new ReservationConflictDetector().detectConflicts(woody.getReservations(), deal.getReservations());

        assertEquals(5, conflicts.size());
        assertTrue(conflicts.stream().allMatch(c -> c.getDistanceAtStartNauticalMiles() >= 0));
        assertTrue(conflicts.stream().allMatch(c -> c.getDistanceAtEndNauticalMiles() >= 0));
        assertTrue(conflicts.stream().allMatch(c -> c.getAngleBetweenRoutesDegrees() >= 0
                && c.getAngleBetweenRoutesDegrees() <= 180));
        assertEquals(Arrays.asList(
                "WOODY01-02-R0->DEAL01-02-R0",
                "WOODY01-02-R0->DEAL01-02-R1",
                "WOODY01-02-R1->DEAL01-02-R0",
                "WOODY01-02-R1->DEAL01-02-R1",
                "WOODY01-02-R2->DEAL01-02-R1"
        ), conflicts.stream()
                .map(c -> c.getFirst().getId() + "->" + c.getSecond().getId())
                .sorted()
                .collect(Collectors.toList()));
    }

    private String message(String... reservations) {
        StringBuilder builder = new StringBuilder();
        for (String reservation : reservations) {
            builder.append(reservation).append("\n");
        }
        return builder.toString();
    }

    private String reservation(int index,
                               String start,
                               String end,
                               String endWithAvana,
                               String routeStart,
                               String routeEnd,
                               String longitudinalSep) {
        return "reservation[" + index + "]\n"
                + "START DATE\n"
                + "Sat Apr 10 " + start + " EDT 2010\n"
                + "END DATE\n"
                + "Sat Apr 10 " + end + " EDT 2010\n"
                + "END DATE W/AVANA\n"
                + "Sat Apr 10 " + endWithAvana + " EDT 2010\n"
                + "VERT RANGE\n"
                + "FL240B260\n"
                + "VERT SEPARATION\n"
                + "2000.0\n"
                + "VERT RANGE FOR DECONFLICTION\n"
                + "23000.0-27000.0\n"
                + "SOURCE\n"
                + "fixes[x-y], ratio[0.0-1.0], points[" + routeStart + "-" + routeEnd + "], "
                + "route seg distance: 1113194.907792064, lateralSep 119.99999989872, "
                + "longitudinalSep " + longitudinalSep + "\n"
                + "DECON SHAPES\n"
                + "DISP_CONF SHAPES\n"
                + "DISP_NORM SHAPES\n";
    }

    private String horReservation(int index,
                                  String start,
                                  String end,
                                  String verticalRange,
                                  String deconflictionRange,
                                  String routeStart,
                                  String routeEnd) {
        return "reservation[" + index + "]\n"
                + "START DATE\n"
                + "Tue Mar 02 " + start + " EST 2010\n"
                + "END DATE\n"
                + "Tue Mar 02 " + end + " EST 2010\n"
                + "VERT RANGE\n"
                + verticalRange + "\n"
                + "VERT SEPARATION\n"
                + "499.0\n"
                + "VERT RANGE FOR DECONFLICTION\n"
                + deconflictionRange + "\n"
                + "SOURCE\n"
                + "fixes[x-y], ratio[0.0-1.0], points[" + routeStart + "-" + routeEnd + "], "
                + "horSep 185218.52000000002\n"
                + "DECON SHAPES\n"
                + "DISP_CONF SHAPES\n"
                + "DISP_NORM SHAPES\n";
    }

    private void assertConflict(ReservationConflict conflict, String firstId, String secondId) {
        assertEquals(firstId, conflict.getFirst().getId());
        assertEquals(secondId, conflict.getSecond().getId());
    }

    private String rawDealMessage() {
        return "A. DEAL01-02\n\n"
                + "B. 2F15/I\n\n"
                + "C. ZZZZ\n\n"
                + "D. ((PR FL240B260 3100N 15000W 0100 3100N 16000W 0200 "
                + "CLMB FL270B280 LVLOF BY 3100N 17000W 0300 3100N 18000W 0400 ...\n\n"
                + "E. ZZZZ\n\n"
                + "F. ETD 021400 MAR 2010 ADMIS 1 MIN AVANA 021500\n\n"
                + "G. TAS:  450KTS\n\n"
                + "PROJECT OFFICER:  O/F CARF\n\n"
                + "ARTCCS CONCERNED:  ZOA\n\n"
                + "ADDITIONAL INFO:  MARSA\n";
    }

    private String rawWoodyMessage() {
        return "A. WOODY01-02\n\n"
                + "B. 2F22/I\n\n"
                + "C. ZZZZ\n\n"
                + "D. ((PR FL240B260 3000N 15000W 0100 3000N 16000W 0200 "
                + "3000N 17000W 0300 3000N 18000W 0400 ...\n\n"
                + "E. ZZZZ\n\n"
                + "F. ETD 021200 MAR 2010 ADMIS 1 MIN AVANA 021300\n\n"
                + "G. TAS:  450KTS\n\n"
                + "PROJECT OFFICER:  O/F CARF\n\n"
                + "ARTCCS CONCERNED:  ZAK\n\n"
                + "ADDITIONAL INFO:  MARSA\n";
    }
}
