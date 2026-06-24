package org.tash.extensions.simulation;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

class NationalDemandCapacitySimulatorTest {
    @Test
    void generatesDeterministicNationalScaleDemandCapacityReport() {
        NationalDemandCapacitySimulator simulator = new NationalDemandCapacitySimulator();
        NationalDemandCapacityConfig config = NationalDemandCapacityConfig.builder()
                .id("unit")
                .flightCount(1000)
                .airportCount(12)
                .sectorCount(18)
                .durationMinutes(180)
                .tickIntervalMinutes(15)
                .randomSeed(42)
                .demandSpikeFactor(1.45)
                .capacityReductionFactor(0.65)
                .build();

        NationalDemandCapacityReport first = simulator.run(config);
        NationalDemandCapacityReport second = simulator.run(config);

        assertEquals("national-demand-unit", first.getId());
        assertEquals(1000, first.getFlightCount());
        assertEquals(12, first.getAirportCount());
        assertEquals(18, first.getSectorCount());
        assertEquals(first.getPeakAirportDemandCapacityRatio(), second.getPeakAirportDemandCapacityRatio(), 0.0001);
        assertEquals(first.getPeakSectorDemandCapacityRatio(), second.getPeakSectorDemandCapacityRatio(), 0.0001);
        assertFalse(first.getSnapshots().isEmpty());
        assertNotNull(first.getTrafficReplay());
        assertEquals(1000, first.getTrafficReplay().getFlightPlans().size());
        assertEquals(3000, first.getTrafficReplay().getPositions().size());
        assertTrue(first.getTrafficReplay().getAirportDemand().size() >= 12);
        assertTrue(first.getTrafficReplay().getSectorDemand().size() >= 18);
        assertTrue(first.getTotalTmiRecommendationCount() > 0);
        assertTrue(first.getPeakOverloadedAirportCount() > 0 || first.getPeakOverloadedSectorCount() > 0);
    }

    @Test
    void exposesMinuteScopedNationalSnapshot() {
        NationalDemandCapacitySimulator simulator = new NationalDemandCapacitySimulator();
        NationalDemandCapacityReport report = simulator.run(NationalDemandCapacityConfig.builder()
                .id("snapshot")
                .flightCount(300)
                .airportCount(8)
                .sectorCount(10)
                .durationMinutes(120)
                .tickIntervalMinutes(10)
                .randomSeed(7)
                .build());

        NationalDemandCapacitySnapshot snapshot = simulator.snapshotAtMinute(report, 37);

        assertNotNull(snapshot);
        assertEquals(30, snapshot.getMinute());
        assertEquals(300, snapshot.getTotalFlightCount());
        assertTrue(snapshot.getAirportCount() > 0);
        assertTrue(snapshot.getSectorCount() > 0);
        assertTrue(snapshot.getMaxAirportDemandCapacityRatio() >= 0);
        assertTrue(snapshot.getMaxSectorDemandCapacityRatio() >= 0);
    }
}
