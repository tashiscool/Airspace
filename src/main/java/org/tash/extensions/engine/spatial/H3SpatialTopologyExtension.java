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

public class H3SpatialTopologyExtension implements SpatialTopologyExtension {
    private static final int DEFAULT_RESOLUTION = 5;
    private final NativeSpatialTopologyExtension nativeFallback = new NativeSpatialTopologyExtension();

    @Override
    public SpatialTopologyBackend backend() {
        return SpatialTopologyBackend.H3;
    }

    @Override
    public boolean isAvailable() {
        try {
            Class.forName("com.uber.h3core.H3Core");
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
        Set<String> left = new LinkedHashSet<>(coveringCells(leftGeometry, DEFAULT_RESOLUTION));
        Set<String> right = new LinkedHashSet<>(coveringCells(rightGeometry, DEFAULT_RESOLUTION));
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
        Object h3 = h3Core();
        if (h3 == null) {
            return cells;
        }
        int resolution = Math.max(0, Math.min(15, resolutionOrLevel));
        for (GeoCoordinate point : representativePoints(geometry)) {
            String cell = cellAddress(h3, point, resolution);
            if (cell != null && !cells.contains(cell)) {
                cells.add(cell);
            }
        }
        return cells;
    }

    private Object h3Core() {
        try {
            Class<?> h3Class = Class.forName("com.uber.h3core.H3Core");
            return h3Class.getMethod("newInstance").invoke(null);
        } catch (ReflectiveOperationException ignored) {
            return null;
        }
    }

    private String cellAddress(Object h3, GeoCoordinate point, int resolution) {
        for (String methodName : new String[]{"latLngToCellAddress", "geoToH3Address"}) {
            try {
                Method method = h3.getClass().getMethod(methodName, double.class, double.class, int.class);
                Object value = method.invoke(h3, point.getLatitude(), point.getLongitude(), resolution);
                return value == null ? null : String.valueOf(value);
            } catch (ReflectiveOperationException ignored) {
                // Try the next supported API spelling.
            }
        }
        return null;
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
