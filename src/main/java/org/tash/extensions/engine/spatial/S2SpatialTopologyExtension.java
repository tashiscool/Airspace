package org.tash.extensions.engine.spatial;

import org.tash.data.GeoCoordinate;
import org.tash.extensions.engine.GeometryOverlapResult;
import org.tash.extensions.engine.GeometryOverlapType;

import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

public class S2SpatialTopologyExtension implements SpatialTopologyExtension {
    private static final int DEFAULT_LEVEL = 10;
    private final NativeSpatialTopologyExtension nativeFallback = new NativeSpatialTopologyExtension();

    @Override
    public SpatialTopologyBackend backend() {
        return SpatialTopologyBackend.S2;
    }

    @Override
    public boolean isAvailable() {
        try {
            Class.forName("com.google.common.geometry.S2CellId");
            Class.forName("com.google.common.geometry.S2LatLng");
            return true;
        } catch (ClassNotFoundException ignored) {
            return false;
        }
    }

    @Override
    public boolean supportsDiscreteIndex() {
        return isAvailable();
    }

    @Override
    public GeometryOverlapResult overlap(List<GeoCoordinate> leftGeometry, List<GeoCoordinate> rightGeometry) {
        Set<String> left = new LinkedHashSet<>(coveringCells(leftGeometry, DEFAULT_LEVEL));
        Set<String> right = new LinkedHashSet<>(coveringCells(rightGeometry, DEFAULT_LEVEL));
        left.retainAll(right);
        if (!left.isEmpty()) {
            return GeometryOverlapResult.builder()
                    .overlaps(true)
                    .type(GeometryOverlapType.BBOX_PREFILTER_ONLY)
                    .minimumDistanceNauticalMiles(0.0)
                    .confidence(0.78)
                    .build();
        }
        return nativeFallback.overlap(leftGeometry, rightGeometry);
    }

    @Override
    public List<String> coveringCells(List<GeoCoordinate> geometry, int resolutionOrLevel) {
        if (!isAvailable() || geometry == null || geometry.isEmpty()) {
            return Collections.emptyList();
        }
        List<String> cells = new ArrayList<>();
        int level = Math.max(0, Math.min(30, resolutionOrLevel));
        for (GeoCoordinate point : representativePoints(geometry)) {
            String token = cellToken(point, level);
            if (token != null && !cells.contains(token)) {
                cells.add(token);
            }
        }
        return cells;
    }

    private String cellToken(GeoCoordinate point, int level) {
        try {
            Class<?> latLngClass = Class.forName("com.google.common.geometry.S2LatLng");
            Class<?> cellIdClass = Class.forName("com.google.common.geometry.S2CellId");
            Object latLng = latLngClass.getMethod("fromDegrees", double.class, double.class)
                    .invoke(null, point.getLatitude(), point.getLongitude());
            Object cellId = cellIdClass.getMethod("fromLatLng", latLngClass).invoke(null, latLng);
            Object parent = cellIdClass.getMethod("parent", int.class).invoke(cellId, level);
            Method toToken = cellIdClass.getMethod("toToken");
            return String.valueOf(toToken.invoke(parent));
        } catch (ReflectiveOperationException ignored) {
            return null;
        }
    }

    private List<GeoCoordinate> representativePoints(List<GeoCoordinate> geometry) {
        List<GeoCoordinate> points = new ArrayList<>(geometry);
        GeoCoordinate centroid = centroid(geometry);
        if (centroid != null) {
            points.add(centroid);
        }
        return points;
    }

    private GeoCoordinate centroid(List<GeoCoordinate> geometry) {
        double lat = 0.0;
        double lon = 0.0;
        int count = 0;
        for (GeoCoordinate point : geometry) {
            if (point == null) {
                continue;
            }
            lat += point.getLatitude();
            lon += point.getLongitude();
            count++;
        }
        if (count == 0) {
            return null;
        }
        return GeoCoordinate.builder().latitude(lat / count).longitude(lon / count).build();
    }
}
