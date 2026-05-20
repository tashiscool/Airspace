package org.tash.extensions.engine.spatial;

import org.tash.data.GeoCoordinate;
import org.tash.extensions.engine.GeometryOverlapResult;
import org.tash.extensions.engine.OperationalGeometryService;

import java.util.List;

public class PluggableOperationalGeometryService extends OperationalGeometryService {
    private final SpatialTopologyExtensionRegistry registry;
    private final SpatialTopologyBackend preciseBackend;
    private final SpatialTopologyBackend discreteBackend;

    public PluggableOperationalGeometryService() {
        this(SpatialTopologyExtensionRegistry.defaults(), SpatialTopologyBackend.JTS, SpatialTopologyBackend.H3);
    }

    public PluggableOperationalGeometryService(
            SpatialTopologyExtensionRegistry registry,
            SpatialTopologyBackend preciseBackend,
            SpatialTopologyBackend discreteBackend) {
        this.registry = registry == null ? SpatialTopologyExtensionRegistry.defaults() : registry;
        this.preciseBackend = preciseBackend == null ? SpatialTopologyBackend.JTS : preciseBackend;
        this.discreteBackend = discreteBackend == null ? SpatialTopologyBackend.H3 : discreteBackend;
    }

    @Override
    public boolean overlaps(List<GeoCoordinate> leftGeometry, List<GeoCoordinate> rightGeometry) {
        return overlap(leftGeometry, rightGeometry).isOverlaps();
    }

    @Override
    public GeometryOverlapResult overlap(List<GeoCoordinate> leftGeometry, List<GeoCoordinate> rightGeometry) {
        return registry.bestForPreciseTopology(preciseBackend).overlap(leftGeometry, rightGeometry);
    }

    public List<String> coveringCells(List<GeoCoordinate> geometry, int resolutionOrLevel) {
        return registry.bestForDiscreteIndex(discreteBackend).coveringCells(geometry, resolutionOrLevel);
    }
}
