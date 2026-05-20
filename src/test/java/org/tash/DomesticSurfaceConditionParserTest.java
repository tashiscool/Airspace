package org.tash;

import org.junit.jupiter.api.Test;
import org.tash.extensions.notam.DomesticSurfaceCondition;
import org.tash.extensions.notam.DomesticSurfaceConditionParser;

import static org.junit.jupiter.api.Assertions.*;

class DomesticSurfaceConditionParserTest {
    private final DomesticSurfaceConditionParser parser = new DomesticSurfaceConditionParser();

    @Test
    void parsesBermRows() {
        String row = "!SEA 01/001 SEA RWY 16L/34R 12 IN BERM 1101011200-1101011300";
        DomesticSurfaceCondition condition = parser.parse(row);

        assertEquals("SEA", condition.getAccountability());
        assertEquals("RWY", condition.getKeyword());
        assertEquals("16L/34R", condition.getAffectedSurface());
        assertTrue(condition.getHazards().contains(DomesticSurfaceCondition.Hazard.BERM));
        assertEquals(12, condition.getBermHeightInches(), 0.0001);
        assertNotNull(condition.getEffectiveStart());
    }

    @Test
    void parsesTurnaroundRows() {
        String row = "!CDV 01/001 CDV RWY 9/27 TURNAROUNDS THN SN AND ICE 1101011200-1101011300";
        DomesticSurfaceCondition condition = parser.parse(row);

        assertEquals("CDV", condition.getLocation());
        assertEquals("RWY", condition.getKeyword());
        assertEquals("9/27", condition.getAffectedSurface());
        assertTrue(condition.getHazards().contains(DomesticSurfaceCondition.Hazard.TURNAROUND));
        assertTrue(condition.getHazards().contains(DomesticSurfaceCondition.Hazard.SNOW));
        assertTrue(condition.getHazards().contains(DomesticSurfaceCondition.Hazard.ICE));
    }

    @Test
    void parsesSnowPileRows() {
        String row = "!ORH 01/001 ORH RAMP ALL 15 FT SNOWPILES 1101011200-1101011300";
        DomesticSurfaceCondition condition = parser.parse(row);

        assertEquals("ORH", condition.getAccountability());
        assertEquals("RAMP", condition.getKeyword());
        assertEquals("ALL", condition.getAffectedSurface());
        assertTrue(condition.getHazards().contains(DomesticSurfaceCondition.Hazard.SNOW_PILE));
        assertEquals(15, condition.getSnowPileHeightFeet(), 0.0001);
        assertNotNull(condition.getEffectiveEnd());
    }
}
