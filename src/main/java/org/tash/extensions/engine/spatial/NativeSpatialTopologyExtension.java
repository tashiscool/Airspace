package org.tash.extensions.engine.spatial;

import org.tash.data.GeoCoordinate;
import org.tash.extensions.engine.GeometryOverlapResult;
import org.tash.extensions.engine.OperationalGeometryService;

import java.util.ArrayList;
import java.util.List;

public class NativeSpatialTopologyExtension implements SpatialTopologyExtension {
    private final OperationalGeometryService geometryService;

    public NativeSpatialTopologyExtension() {
        this(new OperationalGeometryService());
    }

    public NativeSpatialTopologyExtension(OperationalGeometryService geometryService) {
        this.geometryService = geometryService == null ? new OperationalGeometryService() : geometryService;
    }

    @Override
    public SpatialTopologyBackend backend() {
        return SpatialTopologyBackend.NATIVE;
    }

    @Override
    public boolean isAvailable() {
        return true;
    }

    @Override
    public boolean supportsPreciseTopology() {
        return true;
    }

    @Override
    public GeometryOverlapResult overlap(List<GeoCoordinate> leftGeometry, List<GeoCoordinate> rightGeometry) {
        return geometryService.overlap(leftGeometry, rightGeometry);
    }

    @Override
    public List<String> coveringCells(List<GeoCoordinate> geometry, int resolutionOrLevel) {
        List<String> cells = new ArrayList<>();
        if (geometry == null) {
            return cells;
        }
        int scale = Math.max(1, resolutionOrLevel);
        for (GeoCoordinate point : geometry) {
            if (point == null) {
                continue;
            }
            int lat = (int) Math.floor(point.getLatitude() * scale);
            int lon = (int) Math.floor(point.getLongitude() * scale);
            cells.add("native:" + scale + ":" + lat + ":" + lon);
        }
        return cells;
    }
}
