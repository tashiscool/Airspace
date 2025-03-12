package org.tash.extensions.weather.avoid;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.weather.HazardSeverity;
import org.tash.extensions.weather.WeatherElementType;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

/**
 * A polygonal weather cell (e.g., weather front, SIGMET area)
 */
@Data
public class PolygonalWeatherCell extends WeatherCell {

    private final List<GeoCoordinate> vertices;

    @Builder
    public PolygonalWeatherCell(
            String id,
            WeatherElementType type,
            HazardSeverity severity,
            ZonedDateTime startTime,
            ZonedDateTime endTime,
            double minAltitude,
            double maxAltitude,
            List<GeoCoordinate> vertices) {
        super(id, type, severity, startTime, endTime, minAltitude, maxAltitude);
        this.vertices = new ArrayList<>(vertices);

        // Ensure the polygon is closed
        if (!vertices.get(0).equals(vertices.get(vertices.size() - 1))) {
            this.vertices.add(vertices.get(0));
        }
    }

    @Override
    protected boolean containsHorizontally(GeoCoordinate point) {
        // Implementation of ray casting algorithm
        boolean inside = false;
        int n = vertices.size();

        for (int i = 0, j = n - 1; i < n; j = i++) {
            double lat_i = vertices.get(i).getLatitude();
            double lon_i = vertices.get(i).getLongitude();
            double lat_j = vertices.get(j).getLatitude();
            double lon_j = vertices.get(j).getLongitude();

            if (((lat_i > point.getLatitude()) != (lat_j > point.getLatitude())) &&
                    (point.getLongitude() < (lon_j - lon_i) * (point.getLatitude() - lat_i) / (lat_j - lat_i) + lon_i)) {
                inside = !inside;
            }
        }

        return inside;
    }

    @Override
    public List<GeoCoordinate> getBoundaryPoints() {
        return new ArrayList<>(vertices);
    }

    @Override
    protected boolean doesPathIntersectBoundary(GeoCoordinate start, GeoCoordinate end) {
        // Check if path intersects any of the polygon edges
        int n = vertices.size();

        for (int i = 0, j = n - 1; i < n; j = i++) {
            GeoCoordinate v1 = vertices.get(j);
            GeoCoordinate v2 = vertices.get(i);

            if (doLineSegmentsIntersect(start, end, v1, v2)) {
                return true;
            }
        }

        return false;
    }

    /**
     * Check if two line segments intersect
     *
     * @param p1 First point of first line segment
     * @param p2 Second point of first line segment
     * @param p3 First point of second line segment
     * @param p4 Second point of second line segment
     * @return True if the line segments intersect
     */
    private boolean doLineSegmentsIntersect(
            GeoCoordinate p1, GeoCoordinate p2,
            GeoCoordinate p3, GeoCoordinate p4) {

        // Convert to a local Cartesian coordinate system for simplicity
        // Use the first point as the origin
        double x1 = 0;
        double y1 = 0;
        double x2 = p2.getLatitude() - p1.getLatitude();
        double y2 = p2.getLongitude() - p1.getLongitude();
        double x3 = p3.getLatitude() - p1.getLatitude();
        double y3 = p3.getLongitude() - p1.getLongitude();
        double x4 = p4.getLatitude() - p1.getLatitude();
        double y4 = p4.getLongitude() - p1.getLongitude();

        // Calculate orientation of 3 points
        // Returns -1 (counterclockwise), 0 (collinear), 1 (clockwise)
        BiFunction<Double, Double, Double, Double, Integer, Integer, Integer> orientation =
                (x1o, y1o, x2o, y2o, x3o, y3o) -> {
                    double val = (y2 - y1) * (x3 - x2) - (x2 - x1) * (y3 - y2);
                    return val == 0 ? 0 : (val > 0 ? 1 : -1);
                };

        // Check if point q lies on segment pr
        // p, q, r are in CCW order
        BiFunction<Double, Double, Double, Double, Double, Double, Boolean> onSegment =
                (px, py, qx, qy, rx, ry) -> {
                    return qx <= Math.max(px, rx) && qx >= Math.min(px, rx) &&
                            qy <= Math.max(py, ry) && qy >= Math.min(py, ry);
                };

        // Calculate orientations
        int o1 = (int) orientation.apply(x1, y1, x2, y2, (int) x3, (int) y3);
        int o2 = (int) orientation.apply(x1, y1, x2, y2, (int) x4, (int) y4);
        int o3 = (int) orientation.apply(x3, y3, x4, y4,(int)  x1, (int) y1);
        int o4 = (int) orientation.apply(x3, y3, x4, y4, (int) x2, (int) y2);

        // General case
        if (o1 != o2 && o3 != o4) {
            return true;
        }

        // Special cases for collinear points
        if (o1 == 0 && onSegment.apply(x1, y1, x3, y3, x2, y2)) return true;
        if (o2 == 0 && onSegment.apply(x1, y1, x4, y4, x2, y2)) return true;
        if (o3 == 0 && onSegment.apply(x3, y3, x1, y1, x4, y4)) return true;
        if (o4 == 0 && onSegment.apply(x3, y3, x2, y2, x4, y4)) return true;

        return false;
    }

    /**
     * Functional interface for a function with six parameters
     */
    @FunctionalInterface
    private interface BiFunction<A, B, C, D, E, F, R> {
        R apply(A a, B b, C c, D d, E e, F f);
    }
}
