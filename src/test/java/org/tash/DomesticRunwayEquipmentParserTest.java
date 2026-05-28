package org.tash;

import org.junit.jupiter.api.Test;
import org.tash.extensions.notam.DomesticRunwayEquipmentParser;
import org.tash.extensions.notam.DomesticRunwayEquipmentStatus;

import static org.junit.jupiter.api.Assertions.*;

class DomesticRunwayEquipmentParserTest {
    private final DomesticRunwayEquipmentParser parser = new DomesticRunwayEquipmentParser();

    @Test
    void parsesDistanceRemainingSignOutageRows() {
        String row = "!BOS 01/001 BOS RWY 15R 6000 FT DISTANCE REMAINING SIGN UNLGTD 1101011200-1101011300";
        DomesticRunwayEquipmentStatus status = parser.parse(row);

        assertEquals("BOS", status.getAccountability());
        assertEquals("15R", status.getRunway());
        assertEquals(1, status.getDistancesFeet().size());
        assertEquals(6000, status.getDistancesFeet().get(0).intValue());
        assertTrue(status.getEquipment().contains(DomesticRunwayEquipmentStatus.Equipment.DISTANCE_REMAINING_SIGN));
        assertTrue(status.getStatuses().contains(DomesticRunwayEquipmentStatus.Status.UNLIGHTED));
    }

    @Test
    void parsesSlashSeparatedDistancesAndMissingStatus() {
        String row = "!GUC 01/001 GUC RWY 6 4000/8000 DISTANCE REMAINING SIGN UNLGTD 1101011200-1101011300";
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

    @Test
    void parsesRunwayVisualRangeServiceOutageRows() {
        DomesticRunwayEquipmentStatus status = parser.parse(
                "!JFK 01/777 JFK SVC RWY 04L RVR OTS 2605281200-2605281800");

        assertEquals("04L", status.getRunway());
        assertTrue(status.getEquipment().contains(DomesticRunwayEquipmentStatus.Equipment.RUNWAY_VISUAL_RANGE));
        assertTrue(status.getStatuses().contains(DomesticRunwayEquipmentStatus.Status.OUT_OF_SERVICE));
    }
}
