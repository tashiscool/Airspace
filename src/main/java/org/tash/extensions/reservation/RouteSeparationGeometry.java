package org.tash.extensions.reservation;

import org.tash.data.GeoCoordinate;

public class RouteSeparationGeometry {
    public double segmentDistanceNauticalMiles(GeoCoordinate aStart,
                                               GeoCoordinate aEnd,
                                               GeoCoordinate bStart,
                                               GeoCoordinate bEnd) {
        Projection projection = Projection.forCoordinates(aStart, aEnd, bStart, bEnd);
        Point a0 = projection.project(aStart);
        Point a1 = projection.project(aEnd);
        Point b0 = projection.project(bStart);
        Point b1 = projection.project(bEnd);
        if (segmentsIntersect(a0, a1, b0, b1)) {
            return 0.0;
        }
        return Math.min(
                Math.min(pointToSegmentDistance(a0, b0, b1), pointToSegmentDistance(a1, b0, b1)),
                Math.min(pointToSegmentDistance(b0, a0, a1), pointToSegmentDistance(b1, a0, a1)));
    }

    public double longitudinalGapNauticalMiles(GeoCoordinate baseStart,
                                               GeoCoordinate baseEnd,
                                               GeoCoordinate otherStart,
                                               GeoCoordinate otherEnd) {
        Projection projection = Projection.forCoordinates(baseStart, baseEnd, otherStart, otherEnd);
        Point a0 = projection.project(baseStart);
        Point a1 = projection.project(baseEnd);
        Point b0 = projection.project(otherStart);
        Point b1 = projection.project(otherEnd);
        Vector base = new Vector(a1.x - a0.x, a1.y - a0.y);
        double length = Math.sqrt(base.x * base.x + base.y * base.y);
        if (length == 0.0) {
            return 0.0;
        }
        double bStartAlong = alongTrackDistance(a0, base, length, b0);
        double bEndAlong = alongTrackDistance(a0, base, length, b1);
        double minOther = Math.min(bStartAlong, bEndAlong);
        double maxOther = Math.max(bStartAlong, bEndAlong);
        if (maxOther < 0.0) {
            return -maxOther;
        }
        if (minOther > length) {
            return minOther - length;
        }
        return 0.0;
    }

    private double alongTrackDistance(Point origin, Vector base, double length, Point point) {
        double dx = point.x - origin.x;
        double dy = point.y - origin.y;
        return ((dx * base.x) + (dy * base.y)) / length;
    }

    private double pointToSegmentDistance(Point point, Point start, Point end) {
        double dx = end.x - start.x;
        double dy = end.y - start.y;
        double lengthSquared = dx * dx + dy * dy;
        if (lengthSquared == 0.0) {
            return distance(point, start);
        }
        double ratio = ((point.x - start.x) * dx + (point.y - start.y) * dy) / lengthSquared;
        double clamped = Math.max(0.0, Math.min(1.0, ratio));
        Point closest = new Point(start.x + clamped * dx, start.y + clamped * dy);
        return distance(point, closest);
    }

    private double distance(Point first, Point second) {
        double dx = first.x - second.x;
        double dy = first.y - second.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    private boolean segmentsIntersect(Point a0, Point a1, Point b0, Point b1) {
        double o1 = orientation(a0, a1, b0);
        double o2 = orientation(a0, a1, b1);
        double o3 = orientation(b0, b1, a0);
        double o4 = orientation(b0, b1, a1);

        if (o1 * o2 < 0.0 && o3 * o4 < 0.0) {
            return true;
        }
        return onSegment(a0, b0, a1, o1)
                || onSegment(a0, b1, a1, o2)
                || onSegment(b0, a0, b1, o3)
                || onSegment(b0, a1, b1, o4);
    }

    private double orientation(Point first, Point second, Point third) {
        return ((second.y - first.y) * (third.x - second.x))
                - ((second.x - first.x) * (third.y - second.y));
    }

    private boolean onSegment(Point start, Point point, Point end, double orientation) {
        double epsilon = 1e-9;
        return Math.abs(orientation) < epsilon
                && point.x <= Math.max(start.x, end.x) + epsilon
                && point.x + epsilon >= Math.min(start.x, end.x)
                && point.y <= Math.max(start.y, end.y) + epsilon
                && point.y + epsilon >= Math.min(start.y, end.y);
    }

    private static class Projection {
        private final double referenceLatitude;
        private final double referenceLongitude;
        private final double longitudeScale;

        private Projection(double referenceLatitude, double referenceLongitude) {
            this.referenceLatitude = referenceLatitude;
            this.referenceLongitude = referenceLongitude;
            this.longitudeScale = Math.cos(Math.toRadians(referenceLatitude));
        }

        private static Projection forCoordinates(GeoCoordinate... coordinates) {
            double latitude = 0.0;
            double longitude = 0.0;
            for (GeoCoordinate coordinate : coordinates) {
                latitude += coordinate.getLatitude();
                longitude += coordinate.getLongitude();
            }
            return new Projection(latitude / coordinates.length, longitude / coordinates.length);
        }

        private Point project(GeoCoordinate coordinate) {
            double x = (coordinate.getLongitude() - referenceLongitude) * 60.0 * longitudeScale;
            double y = (coordinate.getLatitude() - referenceLatitude) * 60.0;
            return new Point(x, y);
        }
    }

    private static class Point {
        private final double x;
        private final double y;

        private Point(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }

    private static class Vector {
        private final double x;
        private final double y;

        private Vector(double x, double y) {
            this.x = x;
            this.y = y;
        }
    }
}
