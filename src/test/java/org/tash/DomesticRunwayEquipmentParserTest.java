package org.tash;

import org.junit.jupiter.api.Test;
import org.tash.extensions.evaluation.LegacyArtifactTextExtractor;
import org.tash.extensions.notam.DomesticRunwayEquipmentParser;
import org.tash.extensions.notam.DomesticRunwayEquipmentStatus;

import java.nio.file.Paths;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class DomesticRunwayEquipmentParserTest {
    private final DomesticRunwayEquipmentParser parser = new DomesticRunwayEquipmentParser();

    @Test
    void parsesDistanceRemainingSignOutageRowsExtractedFromLegacySpreadsheet() throws Exception {
        List<String> rows = new LegacyArtifactTextExtractor()
                .notamLines(Paths.get(System.getProperty("user.home"), "Downloads", "REMAINING.xls"));
        String row = rows.stream()
                .filter(line -> line.contains("BOS RWY 15R 6000 FT DISTANCE REMAINING SIGN UNLGTD"))
                .findFirst()
                .orElseThrow(() -> new AssertionError("Missing BOS distance-remaining row"));

        DomesticRunwayEquipmentStatus status = parser.parse(row);

        assertEquals("BOS", status.getAccountability());
        assertEquals("15R", status.getRunway());
        assertEquals(1, status.getDistancesFeet().size());
        assertEquals(6000, status.getDistancesFeet().get(0).intValue());
        assertTrue(status.getEquipment().contains(DomesticRunwayEquipmentStatus.Equipment.DISTANCE_REMAINING_SIGN));
        assertTrue(status.getStatuses().contains(DomesticRunwayEquipmentStatus.Status.UNLIGHTED));
    }

    @Test
    void parsesSlashSeparatedDistancesAndMissingStatus() throws Exception {
        List<String> rows = new LegacyArtifactTextExtractor()
                .notamLines(Paths.get(System.getProperty("user.home"), "Downloads", "REMAINING.xls"));
        String row = rows.stream()
                .filter(line -> line.contains("GUC RWY 6 4000/8000 DISTANCE REMAINING SIGN UNLGTD"))
                .findFirst()
                .orElseThrow(() -> new AssertionError("Missing GUC slash-distance row"));

        DomesticRunwayEquipmentStatus status = parser.parse(row);

        assertEquals("6", status.getRunway());
        assertEquals(4000, status.getDistancesFeet().get(0).intValue());
        assertEquals(8000, status.getDistancesFeet().get(1).intValue());
    }

    @Test
    void parsesMarkerAndNonstandardRows() throws Exception {
        DomesticRunwayEquipmentStatus marker = parser.parse(
                "!PIT 01/021 PIT RWY 10C 4000 FT REMAINING MARKER UNLGTD");
        DomesticRunwayEquipmentStatus nonstandard = parser.parse(
                "!ANC 01/030 ANC RWY 7R RWY LGTS AND MARKINGS NONSTD BTWN 7000 AND 6000 FT DIST REMAINING MARKER");

        assertTrue(marker.getEquipment().contains(DomesticRunwayEquipmentStatus.Equipment.DISTANCE_REMAINING_MARKER));
        assertTrue(nonstandard.getEquipment().contains(DomesticRunwayEquipmentStatus.Equipment.RUNWAY_LIGHTS));
        assertTrue(nonstandard.getEquipment().contains(DomesticRunwayEquipmentStatus.Equipment.RUNWAY_MARKINGS));
        assertTrue(nonstandard.getStatuses().contains(DomesticRunwayEquipmentStatus.Status.NONSTANDARD));
        assertTrue(nonstandard.getEquipment().contains(DomesticRunwayEquipmentStatus.Equipment.DISTANCE_REMAINING_MARKER));
    }
}
