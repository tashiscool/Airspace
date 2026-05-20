package org.tash;

import org.junit.jupiter.api.Test;
import org.tash.extensions.evaluation.LegacyArtifactTextExtractor;
import org.tash.extensions.notam.DomesticSurfaceCondition;
import org.tash.extensions.notam.DomesticSurfaceConditionParser;

import java.nio.file.Paths;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class DomesticSurfaceConditionParserTest {
    private final DomesticSurfaceConditionParser parser = new DomesticSurfaceConditionParser();

    @Test
    void parsesBermRowsExtractedFromLegacySpreadsheet() throws Exception {
        List<String> rows = new LegacyArtifactTextExtractor()
                .notamLines(Paths.get(System.getProperty("user.home"), "Downloads", "BERMS.xls"));
        String row = rows.stream()
                .filter(line -> line.contains("SEA RWY 16L/34R 12 IN BERM"))
                .findFirst()
                .orElseThrow(() -> new AssertionError("Missing SEA berm row"));

        DomesticSurfaceCondition condition = parser.parse(row);

        assertEquals("SEA", condition.getAccountability());
        assertEquals("RWY", condition.getKeyword());
        assertEquals("16L/34R", condition.getAffectedSurface());
        assertTrue(condition.getHazards().contains(DomesticSurfaceCondition.Hazard.BERM));
        assertEquals(12, condition.getBermHeightInches(), 0.0001);
        assertNotNull(condition.getEffectiveStart());
    }

    @Test
    void parsesTurnaroundRowsExtractedFromLegacySpreadsheet() throws Exception {
        List<String> rows = new LegacyArtifactTextExtractor()
                .notamLines(Paths.get(System.getProperty("user.home"), "Downloads", "TURNAROUND.xls"));
        String row = rows.stream()
                .filter(line -> line.contains("CDV RWY 9/27 TURNAROUNDS THN SN"))
                .findFirst()
                .orElseThrow(() -> new AssertionError("Missing CDV turnaround row"));

        DomesticSurfaceCondition condition = parser.parse(row);

        assertEquals("CDV", condition.getLocation());
        assertEquals("RWY", condition.getKeyword());
        assertEquals("9/27", condition.getAffectedSurface());
        assertTrue(condition.getHazards().contains(DomesticSurfaceCondition.Hazard.TURNAROUND));
        assertTrue(condition.getHazards().contains(DomesticSurfaceCondition.Hazard.SNOW));
        assertTrue(condition.getHazards().contains(DomesticSurfaceCondition.Hazard.ICE));
    }

    @Test
    void parsesSnowPileRowsExtractedFromLegacySpreadsheet() throws Exception {
        List<String> rows = new LegacyArtifactTextExtractor()
                .notamLines(Paths.get(System.getProperty("user.home"), "Downloads", "PILE.xls"));
        String row = rows.stream()
                .filter(line -> line.contains("ORH RAMP ALL 15 FT SNOWPILES"))
                .findFirst()
                .orElseThrow(() -> new AssertionError("Missing ORH snow pile row"));

        DomesticSurfaceCondition condition = parser.parse(row);

        assertEquals("ORH", condition.getAccountability());
        assertEquals("RAMP", condition.getKeyword());
        assertEquals("ALL", condition.getAffectedSurface());
        assertTrue(condition.getHazards().contains(DomesticSurfaceCondition.Hazard.SNOW_PILE));
        assertEquals(15, condition.getSnowPileHeightFeet(), 0.0001);
        assertNotNull(condition.getEffectiveEnd());
    }
}
