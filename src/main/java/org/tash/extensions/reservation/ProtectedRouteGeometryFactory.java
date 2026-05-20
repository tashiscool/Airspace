package org.tash.extensions.reservation;

import org.tash.data.GeoCoordinate;
import org.tash.spatial.SpatialPoint;
import org.tash.spatial.SpatialPolygon;
import org.tash.spatial.SpatialVolume;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

/**
 * Native CARF route protected-airspace geometry.
 */
public class ProtectedRouteGeometryFactory {
    private static final int DEFAULT_CAP_SEGMENTS = 12;
    private static final double MINIMUM_HALF_WIDTH_NM = 0.1;

    public SpatialPolygon stripPolygon(String id, GeoCoordinate start, GeoCoordinate end, double halfWidthNauticalMiles) {
        validateRoute(start, end);
        double halfWidth = normalizedHalfWidth(halfWidthNauticalMiles);
        double bearing = start.initialBearingTo(end);
        double right = bearing + 90.0;
        double left = bearing - 90.0;

        List<SpatialPoint> vertices = new ArrayList<>();
        vertices.add(point(id + "-START-RIGHT", start.destinationPoint(halfWidth, right)));
        vertices.add(point(id + "-END-RIGHT", end.destinationPoint(halfWidth, right)));
        vertices.add(point(id + "-END-LEFT", end.destinationPoint(halfWidth, left)));
        vertices.add(point(id + "-START-LEFT", start.destinationPoint(halfWidth, left)));
        return polygon(id + "-STRIP", vertices);
    }

    public SpatialPolygon capsulePolygon(String id, GeoCoordinate start, GeoCoordinate end, double halfWidthNauticalMiles) {
        return capsulePolygon(id, start, end, halfWidthNauticalMiles, DEFAULT_CAP_SEGMENTS);
    }

    public SpatialPolygon capsulePolygon(String id, GeoCoordinate start, GeoCoordinate end,
                                         double halfWidthNauticalMiles, int capSegments) {
        validateRoute(start, end);
        int segments = Math.max(2, capSegments);
        double halfWidth = normalizedHalfWidth(halfWidthNauticalMiles);
        double bearing = start.initialBearingTo(end);
        double right = bearing + 90.0;
        double left = bearing - 90.0;
        double step = 180.0 / segments;

        List<SpatialPoint> vertices = new ArrayList<>();
        for (int i = 0; i <= segments; i++) {
            vertices.add(point(id + "-START-CAP-" + i, start.destinationPoint(halfWidth, right + (step * i))));
        }
        for (int i = 0; i <= segments; i++) {
            vertices.add(point(id + "-END-CAP-" + i, end.destinationPoint(halfWidth, left + (step * i))));
        }
        return polygon(id + "-CAPSULE", vertices);
    }

    public SpatialPolygon paddedPolygon(String id, List<GeoCoordinate> coordinates, double paddingNauticalMiles) {
        if (coordinates == null || coordinates.size() < 3) {
            throw new IllegalArgumentException("Polygon padding requires at least three coordinates");
        }
        double padding = Math.max(0.0, paddingNauticalMiles);
        GeoCoordinate center = centroid(coordinates);
        List<SpatialPoint> vertices = new ArrayList<>();
        for (int i = 0; i < coordinates.size(); i++) {
            GeoCoordinate coordinate = coordinates.get(i);
            GeoCoordinate padded = coordinate;
            if (padding > 0.0) {
                double distance = center.distanceTo(coordinate);
                double bearing = distance == 0.0 ? evenlySpacedBearing(i, coordinates.size()) : center.initialBearingTo(coordinate);
                padded = center.destinationPoint(distance + padding, bearing);
            }
            vertices.add(point(id + "-PADDED-" + i, padded));
        }
        return polygon(id + "-PADDED-POLYGON", vertices);
    }

    public SpatialVolume routeSegmentVolume(String id, GeoCoordinate start, GeoCoordinate end,
                                            double halfWidthNauticalMiles,
                                            double lowerAltitudeFeet,
                                            double upperAltitudeFeet,
                                            ZonedDateTime startTime,
                                            ZonedDateTime endTime) {
        return SpatialVolume.builder()
                .id(id + "-VOLUME-" + UUID.randomUUID())
                .basePolygon(capsulePolygon(id, start, end, halfWidthNauticalMiles))
                .lowerAltitude(lowerAltitudeFeet)
                .upperAltitude(upperAltitudeFeet)
                .startTime(startTime)
                .endTime(endTime)
                .build();
    }

    public SpatialVolume routeSegmentVolume(String id, AirspaceReservation reservation) {
        double lower = reservation.getDeconflictionLowerAltitudeFeet() != 0
                ? reservation.getDeconflictionLowerAltitudeFeet()
                : reservation.getLowerAltitudeFeet();
        double upper = reservation.getDeconflictionUpperAltitudeFeet() != 0
                ? reservation.getDeconflictionUpperAltitudeFeet()
                : reservation.getUpperAltitudeFeet();
        return routeSegmentVolume(
                id,
                reservation.getRouteStart(),
                reservation.getRouteEnd(),
                protectedHalfWidthNauticalMiles(reservation),
                lower,
                upper,
                reservation.getStartTime(),
                reservation.getEffectiveConflictEndTime());
    }

    public double protectedHalfWidthNauticalMiles(AirspaceReservation reservation) {
        if (reservation.getRouteWidthNauticalMiles() > 0) {
            return reservation.getRouteWidthNauticalMiles() / 2.0;
        }
        if (reservation.getLateralSeparationNauticalMiles() > 0) {
            return reservation.getLateralSeparationNauticalMiles() / 2.0;
        }
        return MINIMUM_HALF_WIDTH_NM;
    }

    private SpatialPolygon polygon(String id, List<SpatialPoint> vertices) {
        return SpatialPolygon.builder()
                .id(id + "-" + UUID.randomUUID())
                .vertices(vertices)
                .build();
    }

    private SpatialPoint point(String id, GeoCoordinate coordinate) {
        return SpatialPoint.builder()
                .id(id)
                .coordinate(coordinate)
                .build();
    }

    private GeoCoordinate centroid(List<GeoCoordinate> coordinates) {
        double latitude = 0.0;
        double longitude = 0.0;
        double altitude = 0.0;
        for (GeoCoordinate coordinate : coordinates) {
            latitude += coordinate.getLatitude();
            longitude += coordinate.getLongitude();
            altitude += coordinate.getAltitude();
        }
        int count = coordinates.size();
        return GeoCoordinate.builder()
                .latitude(latitude / count)
                .longitude(longitude / count)
                .altitude(altitude / count)
                .build();
    }

    private double evenlySpacedBearing(int index, int count) {
        return (360.0 * index) / count;
    }

    private double normalizedHalfWidth(double halfWidthNauticalMiles) {
        return Math.max(MINIMUM_HALF_WIDTH_NM, halfWidthNauticalMiles);
    }

    private void validateRoute(GeoCoordinate start, GeoCoordinate end) {
        if (start == null || end == null) {
            throw new IllegalArgumentException("Route geometry requires start and end coordinates");
        }
        if (start.distanceTo(end) == 0.0) {
            throw new IllegalArgumentException("Route geometry requires distinct start and end coordinates");
        }
    }
}
