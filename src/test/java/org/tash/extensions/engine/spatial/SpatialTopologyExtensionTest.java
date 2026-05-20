package org.tash.extensions.engine.spatial;

import org.junit.jupiter.api.Test;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.engine.AltitudeBand;
import org.tash.extensions.engine.ConstraintSpatialIndex;
import org.tash.extensions.engine.EngineConfig;
import org.tash.extensions.engine.GeometryOverlapType;
import org.tash.extensions.engine.OperationalConstraint;
import org.tash.extensions.engine.OperationalConstraintType;
import org.tash.extensions.engine.ProductValidityPolicy;
import org.tash.extensions.engine.RouteCorridor;
import org.tash.extensions.engine.TimeWindow;
import org.tash.extensions.weather.decision.WeatherDecisionSeverity;

import java.time.ZonedDateTime;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

class SpatialTopologyExtensionTest {
    @Test
    void registryExposesAvailableBackendsAndFallsBackToNative() {
        SpatialTopologyExtensionRegistry registry = SpatialTopologyExtensionRegistry.defaults();

        assertFalse(registry.availableExtensions().isEmpty());
        assertSame(SpatialTopologyBackend.JTS,
                registry.bestForPreciseTopology(SpatialTopologyBackend.JTS).backend());
        assertSame(SpatialTopologyBackend.JTS,
                registry.bestForPreciseTopology(SpatialTopologyBackend.H3).backend());
    }

    @Test
    void jtsPerformsPrecisePolygonLineOverlap() {
        JtsSpatialTopologyExtension jts = new JtsSpatialTopologyExtension();
        List<GeoCoordinate> polygon = polygon();
        List<GeoCoordinate> crossingLine = Arrays.asList(point(0.5, -0.25), point(0.5, 1.25));
        List<GeoCoordinate> nearMiss = Arrays.asList(point(1.25, -0.25), point(1.25, 1.25));

        assertTrue(jts.overlap(polygon, crossingLine).isOverlaps());
        assertTrue(jts.overlap(polygon, crossingLine).getType() == GeometryOverlapType.SEGMENT_CROSSING
                || jts.overlap(polygon, crossingLine).getType() == GeometryOverlapType.CONTAINMENT);
        assertFalse(jts.overlap(polygon, nearMiss).isOverlaps());
    }

    @Test
    void h3AndS2AreOptionalDiscreteIndexBackends() {
        H3SpatialTopologyExtension h3 = new H3SpatialTopologyExtension();
        S2SpatialTopologyExtension s2 = new S2SpatialTopologyExtension();

        assertDoesNotThrow(() -> h3.coveringCells(polygon(), 5));
        assertDoesNotThrow(() -> s2.coveringCells(polygon(), 10));
        if (h3.isAvailable()) {
            assertFalse(h3.coveringCells(polygon(), 5).isEmpty());
        }
        if (s2.isAvailable()) {
            assertFalse(s2.coveringCells(polygon(), 10).isEmpty());
        }
    }

    @Test
    void pluggableGeometryServiceCanReplaceNativeOverlap() {
        PluggableOperationalGeometryService geometry = new PluggableOperationalGeometryService(
                SpatialTopologyExtensionRegistry.defaults(),
                SpatialTopologyBackend.JTS,
                SpatialTopologyBackend.H3);

        assertTrue(geometry.overlaps(polygon(), Arrays.asList(point(0.5, -0.25), point(0.5, 1.25))));
        assertFalse(geometry.coveringCells(polygon(), 5).isEmpty());
    }

    @Test
    void constraintSpatialIndexAcceptsTopologyExtension() {
        ZonedDateTime now = ZonedDateTime.parse("2026-05-20T12:00:00Z");
        OperationalConstraint constraint = OperationalConstraint.builder()
                .id("wx-1")
                .type(OperationalConstraintType.WEATHER_HAZARD)
                .startTime(now)
                .endTime(now.plusHours(1))
                .lowerAltitudeFeet(20000)
                .upperAltitudeFeet(40000)
                .severity(WeatherDecisionSeverity.WARNING)
                .confidence(0.9)
                .geometry(polygon())
                .build();
        ConstraintSpatialIndex index = new ConstraintSpatialIndex(
                Collections.singletonList(constraint),
                EngineConfig.builder().discreteIndexBackend(SpatialTopologyBackend.H3).build(),
                new H3SpatialTopologyExtension());

        RouteCorridor corridor = RouteCorridor.builder()
                .points(Arrays.asList(point(0.5, -0.25), point(0.5, 1.25)))
                .bufferNauticalMiles(10.0)
                .build();

        assertFalse(index.query(corridor,
                TimeWindow.builder().start(now.minusMinutes(10)).end(now.plusMinutes(10)).build(),
                AltitudeBand.builder().lowerFeet(25000).upperFeet(35000).build(),
                ProductValidityPolicy.INCLUDE_UNKNOWN_VALIDITY).isEmpty());
        assertDoesNotThrow(() -> index.coveringCellsFor(constraint));
    }

    private List<GeoCoordinate> polygon() {
        return Arrays.asList(point(0.0, 0.0), point(0.0, 1.0), point(1.0, 1.0), point(1.0, 0.0));
    }

    private GeoCoordinate point(double latitude, double longitude) {
        return GeoCoordinate.builder().latitude(latitude).longitude(longitude).build();
    }
}
