package org.tash.extensions.engine;

import org.tash.data.GeoCoordinate;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class OperationalGeometryService {
    public boolean overlaps(List<GeoCoordinate> leftGeometry, List<GeoCoordinate> rightGeometry) {
        return overlap(leftGeometry, rightGeometry).isOverlaps();
    }

    public GeometryOverlapResult overlap(List<GeoCoordinate> leftGeometry, List<GeoCoordinate> rightGeometry) {
        List<GeoCoordinate> left = safe(leftGeometry);
        List<GeoCoordinate> right = safe(rightGeometry);
        if (left.isEmpty() || right.isEmpty()) {
            return result(true, GeometryOverlapType.UNKNOWN_GEOMETRY, Double.NaN, 0.25);
        }
        double[] a = bbox(left);
        double[] b = bbox(right);
        if (!(a[1] >= b[0] && a[0] <= b[1] && a[3] >= b[2] && a[2] <= b[3])) {
            return result(false, GeometryOverlapType.NONE, minDistance(left, right), 0.95);
        }
        if (left.size() == 1 || right.size() == 1) {
            boolean overlaps = pointGeometryOverlaps(left, right);
            return result(overlaps, overlaps ? GeometryOverlapType.POINT_TOUCH : GeometryOverlapType.NONE,
                    minDistance(left, right), 0.9);
        }
        if (segmentsIntersect(left, right)) {
            return result(true, GeometryOverlapType.SEGMENT_CROSSING, 0.0, 0.95);
        }
        if (isPolygon(left) && right.stream().anyMatch(point -> pointInPolygon(point, left))) {
            return result(true, GeometryOverlapType.CONTAINMENT, 0.0, 0.9);
        }
        boolean contained = isPolygon(right) && left.stream().anyMatch(point -> pointInPolygon(point, right));
        return result(contained, contained ? GeometryOverlapType.CONTAINMENT : GeometryOverlapType.NONE,
                minDistance(left, right), contained ? 0.9 : 0.8);
    }

    public double distanceToRoute(List<GeoCoordinate> route, GeoCoordinate point) {
        List<GeoCoordinate> safeRoute = safe(route);
        if (point == null || safeRoute.isEmpty()) {
            return Double.POSITIVE_INFINITY;
        }
        if (safeRoute.size() == 1) {
            return safeRoute.get(0).distanceTo(point);
        }
        double min = Double.POSITIVE_INFINITY;
        for (int i = 0; i + 1 < safeRoute.size(); i++) {
            min = Math.min(min, distanceToSegmentNauticalMiles(point, safeRoute.get(i), safeRoute.get(i + 1)));
        }
        return min;
    }

    public List<Integer> routeSegmentsImpacted(List<GeoCoordinate> route, List<GeoCoordinate> geometry, double corridorWidthNauticalMiles) {
        List<Integer> impacted = new ArrayList<>();
        List<GeoCoordinate> safeRoute = safe(route);
        for (int i = 0; i + 1 < safeRoute.size(); i++) {
            List<GeoCoordinate> corridor = toCorridor(safeRoute.get(i), safeRoute.get(i + 1), corridorWidthNauticalMiles);
            if (overlaps(corridor, geometry)) {
                impacted.add(i);
            }
        }
        return impacted;
    }

    public List<GeoCoordinate> toCorridor(GeoCoordinate start, GeoCoordinate end, double widthNauticalMiles) {
        if (start == null || end == null) {
            return Collections.emptyList();
        }
        double halfWidthDegrees = Math.max(0.01, widthNauticalMiles / 2.0 / 60.0);
        double dLat = end.getLatitude() - start.getLatitude();
        double dLon = end.getLongitude() - start.getLongitude();
        double length = Math.sqrt(dLat * dLat + dLon * dLon);
        if (length == 0.0) {
            return Collections.singletonList(start);
        }
        double offLat = -dLon / length * halfWidthDegrees;
        double offLon = dLat / length * halfWidthDegrees;
        List<GeoCoordinate> points = new ArrayList<>();
        points.add(point(start.getLatitude() + offLat, start.getLongitude() + offLon, start.getAltitude()));
        points.add(point(end.getLatitude() + offLat, end.getLongitude() + offLon, end.getAltitude()));
        points.add(point(end.getLatitude() - offLat, end.getLongitude() - offLon, end.getAltitude()));
        points.add(point(start.getLatitude() - offLat, start.getLongitude() - offLon, start.getAltitude()));
        return points;
    }

    private boolean pointGeometryOverlaps(List<GeoCoordinate> left, List<GeoCoordinate> right) {
        if (left.size() == 1 && right.size() == 1) {
            return close(left.get(0), right.get(0));
        }
        if (left.size() == 1) {
            return pointTouchesGeometry(left.get(0), right);
        }
        return pointTouchesGeometry(right.get(0), left);
    }

    private boolean pointTouchesGeometry(GeoCoordinate point, List<GeoCoordinate> geometry) {
        if (point == null) {
            return false;
        }
        if (isPolygon(geometry) && pointInPolygon(point, geometry)) {
            return true;
        }
        for (int i = 0; i + 1 < geometry.size(); i++) {
            if (distanceToSegmentDegrees(point, geometry.get(i), geometry.get(i + 1)) < 0.02) {
                return true;
            }
        }
        return geometry.stream().anyMatch(other -> close(point, other));
    }

    private boolean segmentsIntersect(List<GeoCoordinate> left, List<GeoCoordinate> right) {
        for (int i = 0; i + 1 < left.size(); i++) {
            for (int j = 0; j + 1 < right.size(); j++) {
                if (segmentsIntersect(left.get(i), left.get(i + 1), right.get(j), right.get(j + 1))) {
                    return true;
                }
            }
        }
        return false;
    }

    private boolean segmentsIntersect(GeoCoordinate a, GeoCoordinate b, GeoCoordinate c, GeoCoordinate d) {
        if (a == null || b == null || c == null || d == null) {
            return false;
        }
        double o1 = orientation(a, b, c);
        double o2 = orientation(a, b, d);
        double o3 = orientation(c, d, a);
        double o4 = orientation(c, d, b);
        if (o1 * o2 < 0.0 && o3 * o4 < 0.0) {
            return true;
        }
        return Math.abs(o1) < 1e-9 && onSegment(a, c, b)
                || Math.abs(o2) < 1e-9 && onSegment(a, d, b)
                || Math.abs(o3) < 1e-9 && onSegment(c, a, d)
                || Math.abs(o4) < 1e-9 && onSegment(c, b, d);
    }

    private double orientation(GeoCoordinate a, GeoCoordinate b, GeoCoordinate c) {
        return (b.getLongitude() - a.getLongitude()) * (c.getLatitude() - a.getLatitude())
                - (b.getLatitude() - a.getLatitude()) * (c.getLongitude() - a.getLongitude());
    }

    private boolean onSegment(GeoCoordinate a, GeoCoordinate b, GeoCoordinate c) {
        return b.getLatitude() <= Math.max(a.getLatitude(), c.getLatitude()) + 1e-9
                && b.getLatitude() >= Math.min(a.getLatitude(), c.getLatitude()) - 1e-9
                && b.getLongitude() <= Math.max(a.getLongitude(), c.getLongitude()) + 1e-9
                && b.getLongitude() >= Math.min(a.getLongitude(), c.getLongitude()) - 1e-9;
    }

    private boolean pointInPolygon(GeoCoordinate point, List<GeoCoordinate> polygon) {
        boolean inside = false;
        for (int i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
            GeoCoordinate pi = polygon.get(i);
            GeoCoordinate pj = polygon.get(j);
            if (pi == null || pj == null) {
                continue;
            }
            boolean crossesLatitude = (pi.getLatitude() > point.getLatitude()) != (pj.getLatitude() > point.getLatitude());
            if (crossesLatitude) {
                double lonAtLatitude = (pj.getLongitude() - pi.getLongitude())
                        * (point.getLatitude() - pi.getLatitude())
                        / (pj.getLatitude() - pi.getLatitude())
                        + pi.getLongitude();
                if (point.getLongitude() < lonAtLatitude) {
                    inside = !inside;
                }
            }
        }
        return inside;
    }

    private double distanceToSegmentNauticalMiles(GeoCoordinate point, GeoCoordinate start, GeoCoordinate end) {
        double degrees = distanceToSegmentDegrees(point, start, end);
        return degrees * 60.0;
    }

    private double distanceToSegmentDegrees(GeoCoordinate point, GeoCoordinate start, GeoCoordinate end) {
        if (start == null || end == null || point == null) {
            return Double.POSITIVE_INFINITY;
        }
        double dx = end.getLongitude() - start.getLongitude();
        double dy = end.getLatitude() - start.getLatitude();
        if (dx == 0.0 && dy == 0.0) {
            return Math.hypot(point.getLongitude() - start.getLongitude(), point.getLatitude() - start.getLatitude());
        }
        double t = ((point.getLongitude() - start.getLongitude()) * dx + (point.getLatitude() - start.getLatitude()) * dy)
                / (dx * dx + dy * dy);
        t = Math.max(0.0, Math.min(1.0, t));
        double projectedLon = start.getLongitude() + t * dx;
        double projectedLat = start.getLatitude() + t * dy;
        return Math.hypot(point.getLongitude() - projectedLon, point.getLatitude() - projectedLat);
    }

    private double[] bbox(List<GeoCoordinate> points) {
        double minLat = Double.POSITIVE_INFINITY;
        double maxLat = Double.NEGATIVE_INFINITY;
        double minLon = Double.POSITIVE_INFINITY;
        double maxLon = Double.NEGATIVE_INFINITY;
        for (GeoCoordinate point : points) {
            if (point == null) continue;
            minLat = Math.min(minLat, point.getLatitude());
            maxLat = Math.max(maxLat, point.getLatitude());
            minLon = Math.min(minLon, point.getLongitude());
            maxLon = Math.max(maxLon, point.getLongitude());
        }
        return new double[]{minLat, maxLat, minLon, maxLon};
    }

    private boolean isPolygon(List<GeoCoordinate> geometry) {
        return geometry != null && geometry.size() >= 3;
    }

    private boolean close(GeoCoordinate left, GeoCoordinate right) {
        return left != null && right != null
                && Math.abs(left.getLatitude() - right.getLatitude()) < 0.02
                && Math.abs(left.getLongitude() - right.getLongitude()) < 0.02;
    }

    private List<GeoCoordinate> safe(List<GeoCoordinate> values) {
        return values == null ? Collections.emptyList() : values;
    }

    private double minDistance(List<GeoCoordinate> left, List<GeoCoordinate> right) {
        double min = Double.POSITIVE_INFINITY;
        for (GeoCoordinate a : left) {
            for (int i = 0; i + 1 < right.size(); i++) {
                min = Math.min(min, distanceToSegmentNauticalMiles(a, right.get(i), right.get(i + 1)));
            }
            for (GeoCoordinate b : right) {
                if (a != null && b != null) {
                    min = Math.min(min, a.distanceTo(b));
                }
            }
        }
        return min;
    }

    private GeometryOverlapResult result(boolean overlaps, GeometryOverlapType type, double distance, double confidence) {
        return GeometryOverlapResult.builder()
                .overlaps(overlaps)
                .type(type)
                .minimumDistanceNauticalMiles(distance)
                .confidence(confidence)
                .build();
    }

    private GeoCoordinate point(double lat, double lon, double alt) {
        return GeoCoordinate.builder().latitude(lat).longitude(lon).altitude(alt).build();
    }
}
