package org.tash.extensions.engine.spatial;

import org.tash.data.GeoCoordinate;
import org.tash.extensions.engine.GeometryOverlapResult;

import java.util.Collections;
import java.util.List;

public interface SpatialTopologyExtension {
    SpatialTopologyBackend backend();

    boolean isAvailable();

    default boolean supportsPreciseTopology() {
        return false;
    }

    default boolean supportsDiscreteIndex() {
        return false;
    }

    GeometryOverlapResult overlap(List<GeoCoordinate> leftGeometry, List<GeoCoordinate> rightGeometry);

    default List<String> coveringCells(List<GeoCoordinate> geometry, int resolutionOrLevel) {
        return Collections.emptyList();
    }
}
