package org.tash.extensions.engine.spatial;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.geom.Polygon;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.engine.GeometryOverlapResult;
import org.tash.extensions.engine.GeometryOverlapType;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class JtsSpatialTopologyExtension implements SpatialTopologyExtension {
    private final GeometryFactory geometryFactory = new GeometryFactory();
    private final NativeSpatialTopologyExtension nativeFallback = new NativeSpatialTopologyExtension();

    @Override
    public SpatialTopologyBackend backend() {
        return SpatialTopologyBackend.JTS;
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
        List<GeoCoordinate> left = safe(leftGeometry);
        List<GeoCoordinate> right = safe(rightGeometry);
        if (left.isEmpty() || right.isEmpty()) {
            return nativeFallback.overlap(left, right);
        }
        Geometry leftShape = toGeometry(left);
        Geometry rightShape = toGeometry(right);
        if (leftShape == null || rightShape == null || leftShape.isEmpty() || rightShape.isEmpty()) {
            return nativeFallback.overlap(left, right);
        }

        boolean intersects = leftShape.intersects(rightShape);
        double distanceDegrees = leftShape.distance(rightShape);
        GeometryOverlapType type = type(leftShape, rightShape, intersects);
        return GeometryOverlapResult.builder()
                .overlaps(intersects)
                .type(type)
                .minimumDistanceNauticalMiles(distanceDegrees * 60.0)
                .confidence(intersects ? 0.99 : 0.98)
                .build();
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
            cells.add("jts-bbox:" + scale + ":"
                    + (int) Math.floor(point.getLatitude() * scale) + ":"
                    + (int) Math.floor(point.getLongitude() * scale));
        }
        return cells;
    }

    private GeometryOverlapType type(Geometry left, Geometry right, boolean intersects) {
        if (!intersects) {
            return GeometryOverlapType.NONE;
        }
        if (left.touches(right)) {
            return GeometryOverlapType.POINT_TOUCH;
        }
        if (left.contains(right) || right.contains(left) || left.within(right) || right.within(left)) {
            return GeometryOverlapType.CONTAINMENT;
        }
        return GeometryOverlapType.SEGMENT_CROSSING;
    }

    private Geometry toGeometry(List<GeoCoordinate> points) {
        if (points.size() == 1) {
            return geometryFactory.createPoint(coordinate(points.get(0)));
        }
        Coordinate[] coordinates = coordinates(points);
        if (points.size() == 2) {
            return geometryFactory.createLineString(coordinates);
        }
        Coordinate[] closed = close(coordinates);
        Polygon polygon = geometryFactory.createPolygon(closed);
        if (polygon.isValid()) {
            return polygon;
        }
        LineString line = geometryFactory.createLineString(coordinates);
        return line.isValid() ? line : polygon.buffer(0.0);
    }

    private Coordinate[] coordinates(List<GeoCoordinate> points) {
        Coordinate[] coordinates = new Coordinate[points.size()];
        for (int i = 0; i < points.size(); i++) {
            coordinates[i] = coordinate(points.get(i));
        }
        return coordinates;
    }

    private Coordinate coordinate(GeoCoordinate point) {
        return new Coordinate(point.getLongitude(), point.getLatitude());
    }

    private Coordinate[] close(Coordinate[] coordinates) {
        if (coordinates.length == 0 || coordinates[0].equals2D(coordinates[coordinates.length - 1])) {
            return coordinates;
        }
        Coordinate[] closed = new Coordinate[coordinates.length + 1];
        System.arraycopy(coordinates, 0, closed, 0, coordinates.length);
        closed[closed.length - 1] = new Coordinate(coordinates[0]);
        return closed;
    }

    private List<GeoCoordinate> safe(List<GeoCoordinate> values) {
        return values == null ? Collections.emptyList() : values;
    }
}
