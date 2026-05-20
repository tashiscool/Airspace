package org.tash;

import org.junit.jupiter.api.Test;
import org.tash.extensions.notam.DomesticAirspaceRestrictionParser;
import org.tash.extensions.notam.FdcLaserAirspaceParser;
import org.tash.extensions.notam.NotamAirspaceParser;
import org.tash.extensions.notam.NotamAirspaceRestriction;

import java.time.ZoneOffset;
import java.time.ZonedDateTime;

import static org.junit.jupiter.api.Assertions.*;

class NotamAirspaceParserTest {
    @Test
    void parsesIcaoQFieldIntoNativeSpatialVolume() {
        String notam = "(FDC 6/1234 NOTAMN "
                + "Q) ZDC/QRTCA/IV/BO/W/000/180/3845N07702W005 "
                + "A) KDCA "
                + "B) 2601011200 "
                + "C) 2601011800 "
                + "E) TEMPORARY FLIGHT RESTRICTION WI AN AREA CENTERED ON 3845N07702W "
                + "F) SFC "
                + "G) FL180)";

        NotamAirspaceRestriction restriction = new NotamAirspaceParser().parse(notam);

        assertEquals("NOTAMN", restriction.getNotamType());
        assertEquals("ZDC", restriction.getAccountability());
        assertEquals("KDCA", restriction.getAffectedLocation());
        assertEquals("QRTCA", restriction.getQCode());
        assertEquals(38.75, restriction.getCenterLatitude(), 0.0001);
        assertEquals(-77.033333, restriction.getCenterLongitude(), 0.0001);
        assertEquals(5.0, restriction.getRadiusNauticalMiles(), 0.0001);
        assertEquals(0.0, restriction.getLowerAltitudeFeet(), 0.0001);
        assertEquals(18000.0, restriction.getUpperAltitudeFeet(), 0.0001);
        assertEquals(ZonedDateTime.of(2026, 1, 1, 12, 0, 0, 0, ZoneOffset.UTC), restriction.getEffectiveStart());
        assertEquals(ZonedDateTime.of(2026, 1, 1, 18, 0, 0, 0, ZoneOffset.UTC), restriction.getEffectiveEnd());
        assertTrue(restriction.getVolume().contains(38.75, -77.033333, 10000));
    }

    @Test
    void rejectsNotamsWithoutSpatialApplicability() {
        NotamAirspaceParser parser = new NotamAirspaceParser();

        assertThrows(IllegalArgumentException.class, () ->
                parser.parse("A) KDCA B) 2601011200 C) 2601011800 E) RUNWAY CLOSED"));
    }

    @Test
    void parsesDomesticAerobaticAirspaceRestriction() {
        NotamAirspaceRestriction restriction = new DomesticAirspaceRestrictionParser()
                .parse("!CRG 01/001 CRG AIRSPACE AEROBATIC AREA 3500/BLW 3 NMR WEF 1101011200-1101011300");

        assertEquals("CRG", restriction.getAccountability());
        assertEquals("CRG", restriction.getAffectedLocation());
        assertEquals(0, restriction.getLowerAltitudeFeet(), 0.0001);
        assertEquals(3500, restriction.getUpperAltitudeFeet(), 0.0001);
        assertEquals(3, restriction.getRadiusNauticalMiles(), 0.0001);
        assertNotNull(restriction.getVolume());
    }

    @Test
    void parsesDomesticAirspaceRadialDmeRows() {
        String row = "!IPT 01/001 IPT AIRSPACE AEROBATIC AREA 3500/BLW 2 NMR RAV059013.8 1101011200-1101011300";
        NotamAirspaceRestriction restriction = new DomesticAirspaceRestrictionParser().parse(row);

        assertEquals("IPT", restriction.getAccountability());
        assertEquals("IPT", restriction.getAffectedLocation());
        assertEquals(2, restriction.getRadiusNauticalMiles(), 0.0001);
        assertNotEquals(0, restriction.getCenterLatitude(), 0.0001);
        assertNotNull(restriction.getVolume().getBasePolygon());
    }

    @Test
    void parsesFdcLaserAirspaceNotice() {
        String record = "!FDC 1/3690 ZMA WEST PALM BEACH, FL LASER LIGHT ACTIVITY "
                + "AIRBORNE TO GROUND LASER ACTIVITY WILL BE CONDUCTED "
                + "WI AN AREA CENTERED ON 265600N/0801500W "
                + "5000 FEET AND BELOW EFFECTIVE 1101261200 UTC UNTIL 1101272200 UTC";
        NotamAirspaceRestriction restriction = new FdcLaserAirspaceParser().parse(record);

        assertEquals("FDC", restriction.getNotamType());
        assertEquals("ZMA", restriction.getAccountability());
        assertEquals("WEST PALM BEACH, FL LASER LIGHT ACTIVITY", restriction.getAffectedLocation());
        assertEquals("FDC-LASER", restriction.getQCode());
        assertEquals(26.933333, restriction.getCenterLatitude(), 0.0001);
        assertEquals(-80.25, restriction.getCenterLongitude(), 0.0001);
        assertEquals(5000, restriction.getUpperAltitudeFeet(), 0.0001);
        assertEquals(ZonedDateTime.of(2011, 1, 26, 12, 0, 0, 0, ZoneOffset.UTC),
                restriction.getEffectiveStart());
        assertEquals(ZonedDateTime.of(2011, 1, 27, 22, 0, 0, 0, ZoneOffset.UTC),
                restriction.getEffectiveEnd());
        assertNotNull(restriction.getVolume());
    }
}
