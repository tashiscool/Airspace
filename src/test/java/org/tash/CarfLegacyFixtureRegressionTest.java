package org.tash;

import org.junit.jupiter.api.Test;
import org.tash.extensions.reservation.AirspaceReservation;
import org.tash.extensions.reservation.CarfRouteSeparationResolver;
import org.tash.extensions.reservation.CarfRouteMessage;
import org.tash.extensions.reservation.CarfRouteMessageParser;
import org.tash.extensions.reservation.CarfSeparationStandard;
import org.tash.extensions.reservation.CompositeCarfWaypointResolver;
import org.tash.extensions.reservation.DefaultCarfWaypointResolver;
import org.tash.extensions.reservation.ReservationConflict;
import org.tash.extensions.reservation.ReservationConflictDetector;
import org.tash.extensions.reservation.ReservationMessageParser;
import org.tash.extensions.reservation.WaypointFileResolver;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.ZonedDateTime;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

class CarfLegacyFixtureRegressionTest {
    private final CarfRouteMessageParser parser = new CarfRouteMessageParser();

    @Test
    void parsesPriorityZeroNamedFixArtifacts() throws Exception {
        assertParsesReservations(resource("tworessies/MOVG HDON CON ONE OF TWO.txt"));
        assertParsesReservations(resource("tworessies/MOVG HDON CON TWO OF TWO.txt"));
        assertParsesReservations(resource("runtheseandseewhyconflict/FLIGHT1-NOFYI.txt"));
        assertParsesReservations(resource("runtheseandseewhyconflict/FLIGHT2-NOFYI.txt"));
        assertParsesReservations(resource("attachments/HEADON ONE.txt"));
        assertParsesReservations(resource("attachments/HEADON TWO.txt"));
    }

    @Test
    void ptr045HeadOnNoClimbVariantIsVerticallySeparated() throws Exception {
        CarfRouteMessage first = parser.parse(read(resource("attachments/HEADON ONE.txt")));
        CarfRouteMessage second = parser.parse(read(resource("attachments/HEADON TWO.txt")));

        assertTrue(first.getReservations().stream().anyMatch(r -> "4126N 9503W".equals(r.getRouteEndFix())));
        assertTrue(second.getReservations().stream().anyMatch(r -> "4052N 9450W".equals(r.getRouteEndFix())));

        List<ReservationConflict> conflicts =
                new ReservationConflictDetector().detectConflicts(first.getReservations(), second.getReservations());

        assertTrue(conflicts.isEmpty(), "PTR 045 no-CLMB attachment pair should be vertically separated");
    }

    @Test
    void ptr002OppositeDirectionLongitudinalSeparationHasNoFalseConflict() throws Exception {
        CarfRouteMessage first = parser.parse(read(resource("ptr-02-A.txt")));
        CarfRouteMessage second = parser.parse(read(resource("ptr-02-B.txt")));

        assertEquals(5, first.getReservations().size());
        assertEquals(5, second.getReservations().size());
        assertTrue(first.getReservations().stream().anyMatch(r -> "ALB".equals(r.getRouteStartFix())
                && "BUF".equals(r.getRouteEndFix())));
        assertTrue(second.getReservations().stream().anyMatch(r -> "GRB".equals(r.getRouteStartFix())
                && "PECOK".equals(r.getRouteEndFix())));

        List<ReservationConflict> conflicts =
                new ReservationConflictDetector().detectConflicts(first.getReservations(), second.getReservations());

        assertTrue(conflicts.isEmpty(), "PTR 002 opposite-direction pair should not show the original false conflict");
    }

    @Test
    void parsesNavaidAndRouteControlArtifacts() throws Exception {
        CarfRouteMessage ace = parser.parse(read(resource("navaids/ace34l1.txt")));
        CarfRouteMessage ace70 = parser.parse(read(resource("navaids/ACE70L2.txt")));

        assertEquals("RETRO98-99 BLUE49", ace.getActivityName());
        assertTrue(ace.getReservations().size() >= 6);
        assertTrue(ace.getReservations().stream().anyMatch(r -> "YQI".equals(r.getRouteEndFix())));
        assertTrue(ace.getReservations().stream().allMatch(r -> r.getRouteStart() != null && r.getRouteEnd() != null));
        assertTrue(ace.getResolvedWaypointNames().contains("YHZ"));
        assertTrue(ace.getResolvedWaypointNames().contains("WRB"));
        assertTrue(ace.getUnresolvedWaypointNames().isEmpty());

        assertEquals("MAZDA51-56 BLUE01", ace70.getActivityName());
        assertTrue(ace70.getResolvedWaypointNames().contains("DOVEY"));
        assertTrue(ace70.getResolvedWaypointNames().contains("SSC"));
        assertTrue(ace70.getUnresolvedWaypointNames().isEmpty());
    }

    @Test
    void parsesMrkociLateralRadialDmeRegressionPair() throws Exception {
        CarfRouteMessage first = parser.parse(read(resource("mrkoci/JR-TEST-LAT1A.txt")));
        CarfRouteMessage second = parser.parse(read(resource("mrkoci/JR-TEST-LAT1B.txt")));

        assertEquals("SPOILR1-6", first.getActivityName());
        assertEquals("FITER01-06", second.getActivityName());
        assertEquals(5, first.getReservations().size());
        assertEquals(5, second.getReservations().size());
        assertTrue(first.getReservations().stream().anyMatch(r -> "GSH090/001".equals(r.getRouteEndFix())));
        assertTrue(second.getReservations().stream().anyMatch(r -> "GSH".equals(r.getRouteEndFix())));
        assertTrue(first.getReservations().stream().allMatch(r -> r.getProtectedVolume() != null));
        assertTrue(second.getReservations().stream().allMatch(r -> r.getProtectedVolume() != null));

        List<ReservationConflict> conflicts =
                new ReservationConflictDetector().detectConflicts(first.getReservations(), second.getReservations());

        assertEquals(18, conflicts.size());
        assertTrue(conflicts.stream().anyMatch(c -> c.getMinimumLateralDistanceNauticalMiles() <= 0.0001));
        assertTrue(conflicts.stream().allMatch(c -> c.getAngleBetweenRoutesDegrees() >= 0.0
                && c.getAngleBetweenRoutesDegrees() <= 180.0));
        assertTrue(conflicts.stream().allMatch(c -> c.getExplanation().contains("distanceAtStart")));
    }

    @Test
    void headOnArtifactsProduceExplainableConflicts() throws Exception {
        CarfRouteMessage first = parser.parse(read(resource("runtheseandseewhyconflict/FLIGHT1-NOFYI.txt")));
        CarfRouteMessage second = parser.parse(read(resource("runtheseandseewhyconflict/FLIGHT2-NOFYI.txt")));

        List<ReservationConflict> conflicts =
                new ReservationConflictDetector().detectConflicts(first.getReservations(), second.getReservations());

        assertFalse(conflicts.isEmpty());
        assertTrue(conflicts.stream().allMatch(c -> c.getExplanation().contains("verticalGap")));
        assertTrue(conflicts.stream().allMatch(c -> c.getRequiredLateralSeparationNauticalMiles() > 0));
    }

    @Test
    void conflictTooShortListingsCanBeFilteredByMinimumDuration() throws Exception {
        String listing = read(resource("conflicttoshort/90-91conflictslisting.txt"));
        List<AirspaceReservation> first = new ReservationMessageParser().parse("MESSAGE1", section(listing, "MESSAGE1"));
        List<AirspaceReservation> second = new ReservationMessageParser().parse("MESSAGE2", section(listing, "MESSAGE2"));

        List<ReservationConflict> defaultConflicts = new ReservationConflictDetector().detectConflicts(first, second);
        List<ReservationConflict> durationFilteredConflicts = new ReservationConflictDetector(1)
                .detectConflicts(first, second);

        assertFalse(defaultConflicts.isEmpty());
        assertTrue(durationFilteredConflicts.size() <= defaultConflicts.size());
        assertTrue(defaultConflicts.stream().allMatch(c -> c.getDurationSeconds() >= 0));
    }

    @Test
    void parsesAdmissionSecondsAndMitoVariants() throws Exception {
        CarfRouteMessage mito = parser.parse(read(resource("tworessies/MOVG HDON CON ONE OF TWO.txt")));
        CarfRouteMessage seconds = parser.parse(read(resource("tworessies/MOVG HDON CON TWO OF TWO.txt")));

        assertEquals(0, mito.getAdmissionSeconds());
        assertEquals(20, seconds.getAdmissionSeconds());
        assertEquals(ZonedDateTime.parse("2011-02-06T15:30:00-05:00[America/New_York]"), mito.getAvanaTime());
    }

    @Test
    void parsesPtrAirflAndRadialDmeExamples() throws Exception {
        CarfRouteMessage airfl = parser.parse(ptr139Message());
        CarfRouteMessage radial = parser.parse(ptr148RadialMessage());

        assertEquals("DIRCA51-56  BORA01", airfl.getActivityName());
        assertEquals(20, airfl.getAdmissionSeconds());
        assertTrue(airfl.getReservations().size() >= 8);
        assertTrue(airfl.getReservations().stream().anyMatch(r -> "CANOE".equals(r.getRouteStartFix())
                || "CANOE".equals(r.getRouteEndFix())));
        assertEquals("TABOR41-46 CLEAN92", radial.getActivityName());
        assertTrue(radial.getReservations().stream().anyMatch(r -> r.getRouteStartFix().startsWith("TOU")
                || r.getRouteEndFix().startsWith("TOU")));
        assertTrue(radial.getReservations().stream().anyMatch(r -> r.getRouteStartFix().startsWith("ANN")
                || r.getRouteEndFix().startsWith("ANN")));
    }

    @Test
    void splitsRouteLegWhenResolvedSeparationStandardsChange() {
        CarfSeparationStandard eastStandard = CarfSeparationStandard.builder()
                .lateralSeparationNauticalMiles(100)
                .verticalSeparationFeet(499)
                .longitudinalSeparationMinutes(0)
                .routeWidthNauticalMiles(100)
                .build();
        CarfSeparationStandard westStandard = CarfSeparationStandard.builder()
                .lateralSeparationNauticalMiles(120)
                .verticalSeparationFeet(999)
                .longitudinalSeparationMinutes(30)
                .routeWidthNauticalMiles(120)
                .build();
        CarfRouteSeparationResolver resolver = (point, lower, upper) ->
                point.getLongitude() > -155.0 ? eastStandard : westStandard;
        CarfRouteMessageParser parserWithResolver =
                new CarfRouteMessageParser(new DefaultCarfWaypointResolver(), resolver);

        CarfRouteMessage message = parserWithResolver.parse(standardTransitionMessage());

        assertEquals(2, message.getReservations().size());
        assertEquals("STDTEST-R0A", message.getReservations().get(0).getId());
        assertEquals("STDTEST-R0B", message.getReservations().get(1).getId());
        assertEquals(100, message.getReservations().get(0).getLateralSeparationNauticalMiles(), 0.0001);
        assertEquals(120, message.getReservations().get(1).getLateralSeparationNauticalMiles(), 0.0001);
        assertEquals(999, message.getReservations().get(1).getVerticalSeparationFeet(), 0.0001);
        assertEquals(30, message.getReservations().get(1).getLongitudinalSeparationMinutes(), 0.0001);
        assertEquals(message.getReservations().get(0).getEndTime(), message.getReservations().get(1).getStartTime());
        assertEquals(message.getReservations().get(0).getSourceRatioEnd(),
                message.getReservations().get(1).getSourceRatioStart(), 0.0001);
    }

    @Test
    void parsesCarfRoutesAgainstExternalWaypointExportBeforeFallbackCatalog() throws Exception {
        Path export = Files.createTempFile("external-waypoints", ".csv");
        Files.write(export, java.util.Arrays.asList(
                "identifier,latitude,longitude",
                "KVMFIX,44.1,-70.2",
                "NASR1,44.3,-69.7"));
        CarfRouteMessageParser parserWithExternalResolver = new CarfRouteMessageParser(
                new CompositeCarfWaypointResolver(
                        WaypointFileResolver.fromDelimited(export),
                        new DefaultCarfWaypointResolver()));

        CarfRouteMessage message = parserWithExternalResolver.parse(externalWaypointMessage());

        assertEquals("KVMTEST", message.getActivityName());
        assertTrue(message.getResolvedWaypointNames().contains("KVMFIX"));
        assertTrue(message.getResolvedWaypointNames().contains("NASR1"));
        assertTrue(message.getUnresolvedWaypointNames().isEmpty());
        assertEquals(1, message.getReservations().size());
        assertEquals(44.1, message.getReservations().get(0).getRouteStart().getLatitude(), 0.0001);
    }

    @Test
    void parsesTimingTextFixturesAndClimbAltitudeProfile() throws Exception {
        CarfRouteMessage baseline = parser.parse(read(resource("texts/timing test1.txt")));
        CarfRouteMessage climb = parser.parse(read(resource("texts/timing test 2.txt")));

        assertEquals("WOODY01-02", baseline.getActivityName());
        assertEquals("DEAL01-02", climb.getActivityName());
        assertEquals(3, baseline.getReservations().size());
        assertEquals(3, climb.getReservations().size());
        assertEquals(ZonedDateTime.parse("2010-03-02T12:00:00-05:00[America/New_York]"),
                baseline.getEstimatedDepartureTime());
        assertEquals(ZonedDateTime.parse("2015-03-02T14:00:00-05:00[America/New_York]"),
                climb.getEstimatedDepartureTime());
        assertTrue(baseline.getReservations().stream().allMatch(r -> r.getAvanaMinutes() == 60));
        assertTrue(climb.getReservations().stream().allMatch(r -> r.getAvanaMinutes() == 60));

        assertTrue(baseline.getReservations().stream()
                .allMatch(r -> r.getLowerAltitudeFeet() == 24000 && r.getUpperAltitudeFeet() == 26000));
        assertEquals(24000, climb.getReservations().get(0).getLowerAltitudeFeet(), 0.0001);
        assertEquals(26000, climb.getReservations().get(0).getUpperAltitudeFeet(), 0.0001);
        assertEquals(24000, climb.getReservations().get(1).getLowerAltitudeFeet(), 0.0001);
        assertEquals(28000, climb.getReservations().get(1).getUpperAltitudeFeet(), 0.0001);
        assertEquals(27000, climb.getReservations().get(2).getLowerAltitudeFeet(), 0.0001);
        assertEquals(28000, climb.getReservations().get(2).getUpperAltitudeFeet(), 0.0001);

        List<ReservationConflict> conflicts =
                new ReservationConflictDetector().detectConflicts(baseline.getReservations(), climb.getReservations());
        assertTrue(conflicts.isEmpty(), "fixtures are five years apart and should not conflict");
    }

    @Test
    void messagesZipTimingPairReproducesSameYearClimbConflicts() throws Exception {
        Map<String, String> entries = new HashMap<>();
        entries.put("timing test1.txt", read(resource("messages/timing-test1-2010.txt")));
        entries.put("timing test 2.txt", read(resource("messages/timing-test2-2010.txt")));
        Map<String, CarfRouteMessage> messagesByActivity = new HashMap<>();
        for (String text : entries.values()) {
            CarfRouteMessage message = parser.parse(text);
            messagesByActivity.put(message.getActivityName(), message);
        }

        CarfRouteMessage baseline = messagesByActivity.get("WOODY01-02");
        CarfRouteMessage climb = messagesByActivity.get("DEAL01-02");
        assertNotNull(baseline);
        assertNotNull(climb);
        assertEquals(2010, baseline.getEstimatedDepartureTime().getYear());
        assertEquals(2010, climb.getEstimatedDepartureTime().getYear());

        List<ReservationConflict> conflicts =
                new ReservationConflictDetector().detectConflicts(baseline.getReservations(), climb.getReservations());

        assertEquals(5, conflicts.size());
        assertTrue(conflicts.stream().allMatch(c -> c.getStartTime().getYear() == 2010));
        assertTrue(conflicts.stream().allMatch(c -> c.getExplanation().contains("lateralDistance")));
    }

    private void assertParsesReservations(Path path) throws Exception {
        CarfRouteMessage message = parser.parse(read(path));
        assertFalse(message.getReservations().isEmpty(), path.toString());
        assertTrue(message.getReservations().stream().allMatch(r -> r.getSourceFixes() != null && !r.getSourceFixes().isEmpty()));
        assertTrue(message.getReservations().stream().allMatch(r -> r.getProtectedVolume() != null));
        assertTrue(message.getReservations().stream().allMatch(r -> r.getProtectedVolume().getBasePolygon().getVertices().size() > 4));
    }

    private String section(String text, String label) {
        int start = text.indexOf(label);
        assertTrue(start >= 0, "missing " + label);
        int next = text.indexOf("============================================================================", start + label.length());
        return next < 0 ? text.substring(start) : text.substring(start, next);
    }

    private Path resource(String relative) throws Exception {
        java.net.URL url = getClass().getResource("/legacy/carf/" + relative);
        assertNotNull(url, "Missing test resource: " + relative);
        return Paths.get(url.toURI());
    }

    private String read(Path path) throws Exception {
        assertTrue(Files.exists(path), "Missing fixture: " + path);
        return new String(Files.readAllBytes(path), "UTF-8");
    }

    private String ptr139Message() {
        return "A. DIRCA51-56  BORA01\n"
                + "B. 12A10/I  2DC10/Q\n"
                + "C. PWAK\n"
                + "D. FL140B210 AWK 0000 LVLOF BY 1937N 16547E 0013 2045N 16300E 0049 "
                + "2107N 16207E 0059 BEGIN AIRFL 2200N 16000E 0132 2256N 15700E 0210 "
                + "CANOE 0239 MLT 0257 2525N 15125E 0336 2600N 15000E 0358 "
                + "2900N 14000E 0610 2930N 13500E 0711 2944N 13328E 0733 END AIRFL CMPS "
                + "FL190B210 AGIKA 0757 SEPIA 0807 BOMAP 0816 RUSAR 0839 ESBIS 0857 "
                + "RUGMA 0913 CJU 0924 IPDAS 0935 KWA 0945 LINTA 0950 RINBO 0955 ENTEL 1001 OLMEN 1005 SOT 1009 LAND\n"
                + "E. RKSO\n"
                + "F. ETD DIRCA51-56 BORA01 082100 MAR 2011 ADMIS 20 SEC [AVANA 082200]\n"
                + "G. TAS: 300KTS/ CRUISE 250 KTS AIRFL\n";
    }

    private String ptr148RadialMessage() {
        return "A. TABOR41-46 CLEAN92\n"
                + "B. 6F22/I 1K35R/W\n"
                + "C. KLSV\n"
                + "D. FL240B260 FYTTR 0005 LVLOF BY BTY 0013 OAL 0026 FMG 0044 BAARB "
                + "0057 LKV 0109 UBG 0136 JOIN CLEAN91 ARRIE 0157 LEAVE CLEAN92 TOU 068/023 0202 "
                + "TOU 360/025 0206 YZT 0229 DUGGS 0251 HANRY 0305 ANN 127/026 0306 ANN "
                + "0310 LVD 0323 SSR 0340 YAK 0402 JOH 0432 ANC 0447 EDF 0449 LAND\n"
                + "E. PAED\n"
                + "F. ETD 121800 MAR 2011 ADMIS 20 SEC AVANA 121900\n"
                + "G. TAS: 445 KTAS AIRFL/CRUISE\n";
    }

    private String standardTransitionMessage() {
        return "A. STDTEST\n"
                + "B. 1F22/I\n"
                + "C. KZNY\n"
                + "D. FL240B260 3000N 15000W 0000 3000N 16000W 0100\n"
                + "E. KZNY\n"
                + "F. ETD STDTEST 021200 MAR 2010 AVANA 021300\n"
                + "G. TAS: 300 KTS\n";
    }

    private String externalWaypointMessage() {
        return "A. KVMTEST\n"
                + "B. 1F22/I\n"
                + "C. KZNY\n"
                + "D. FL240B260 KVMFIX 0000 NASR1 0100\n"
                + "E. KZNY\n"
                + "F. ETD KVMTEST 021200 MAR 2010 AVANA 021300\n"
                + "G. TAS: 300 KTS\n";
    }
}
