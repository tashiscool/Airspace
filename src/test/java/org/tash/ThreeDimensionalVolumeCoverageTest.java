package org.tash;

import org.junit.jupiter.api.Test;
import org.tash.data.BoundingBox;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.geodetic.GeodeticCalculator;
import org.tash.flight.FlightTrajectory;
import org.tash.spatial.SpatialCircle;
import org.tash.spatial.SpatialLine;
import org.tash.spatial.SpatialPoint;
import org.tash.spatial.SpatialPolygon;
import org.tash.spatial.SpatialVolume;
import org.tash.spatial.threedee.ConicalVolume;
import org.tash.spatial.threedee.CylindricalVolume;
import org.tash.spatial.threedee.FrustumVolume;
import org.tash.trajectory.LinearTrajectorySegment;
import org.tash.trajectory.TrajectoryType;

import java.time.ZonedDateTime;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class ThreeDimensionalVolumeCoverageTest {
    @Test
    void cylindricalVolumePreservesAltitudeCircleAndPolygonalCompatibility() {
        ZonedDateTime start = ZonedDateTime.parse("2026-05-20T12:00:00Z");
        ZonedDateTime end = ZonedDateTime.parse("2026-05-20T13:00:00Z");
        CylindricalVolume cylinder = CylindricalVolume.builder()
                .id("CYL")
                .base(SpatialCircle.builder().id("CYL-CIRCLE").center(point("C", 40, -75, 5000)).radius(25).build())
                .lowerAltitude(4000)
                .upperAltitude(12000)
                .startTime(start)
                .endTime(end)
                .build();

        assertTrue(cylinder.containsPoint(coord(40.05, -75.05, 6000)));
        assertFalse(cylinder.containsPoint(coord(40.05, -75.05, 15000)));
        assertTrue(cylinder.containsTime(start.plusMinutes(15)));
        assertFalse(cylinder.containsTime(end.plusMinutes(1)));

        BoundingBox bounds = cylinder.getBoundingBox();
        assertTrue(bounds.getMinAlt() <= 4000);
        assertTrue(bounds.getMaxAlt() >= 12000);
        assertTrue(bounds.getMinLat() < 40);
        assertTrue(bounds.getMaxLat() > 40);

        SpatialPoint midpoint = cylinder.getPointAtFraction(0.5);
        assertTrue(midpoint.getCoordinate().getAltitude() > 7000);
        assertThrows(IllegalArgumentException.class, () -> cylinder.getPointAtFraction(1.1));

        SpatialVolume polygonal = cylinder.toPolygonalVolume(12);
        assertTrue(polygonal.contains(coord(40, -75, 6000)));
        assertFalse(polygonal.contains(coord(41, -75, 6000)));
    }

    @Test
    void conicalAndFrustumVolumesExposeOperationalGeometry() {
        ConicalVolume cone = ConicalVolume.builder()
                .id("CONE")
                .apex(point("APEX", 35, -100, 1000))
                .baseAltitude(11000)
                .baseRadius(30)
                .centralAxisBearing(0)
                .slopeAngle(0)
                .build();
        FrustumVolume frustum = FrustumVolume.builder()
                .id("FRUSTUM")
                .baseCenter(point("BASE", 35, -100, 5000))
                .baseRadius(5)
                .baseAltitude(5000)
                .topRadius(20)
                .topAltitude(15000)
                .centralAxisBearing(90)
                .length(40)
                .build();

        assertTrue(cone.getBoundingBox().getMaxAlt() >= 11000);
        assertTrue(cone.containsPoint(coord(35.01, -100, 6000)));
        assertFalse(cone.containsPoint(coord(35, -100, 500)));
        assertFalse(cone.containsPoint(coord(34.0, -101.0, 6000)));
        assertFalse(cone.getPerimeterAtAltitude(6000, 8).isEmpty());
        assertTrue(cone.getPerimeterAtAltitude(500, 8).isEmpty());
        assertThrows(UnsupportedOperationException.class, () -> cone.getPointAtFraction(0.5));

        assertNotNull(frustum.getTopCenter());
        assertTrue(frustum.getBoundingBox().getMaxAlt() >= 15000);
        assertTrue(frustum.containsPoint(coord(35, -100, 5000)));
        assertFalse(frustum.containsPoint(coord(35, -100, 20000)));
        assertFalse(frustum.getPerimeterAtAltitude(10000, 10).isEmpty());
        assertTrue(frustum.getPerimeterAtAltitude(3000, 10).isEmpty());
        assertThrows(UnsupportedOperationException.class, () -> frustum.getPointAtFraction(0.5));
    }

    @Test
    void trajectoryAndGeodeticHelpersSupportOperationalRouteGeometry() {
        SpatialPoint a = point("A", 40, -75, 10000);
        SpatialPoint b = point("B", 40.5, -74.5, 12000);
        SpatialPoint c = point("C", 41, -74, 14000);
        SpatialPoint d = point("D", 41.5, -73.5, 16000);
        LinearTrajectorySegment ab = segment("AB", a, b);
        LinearTrajectorySegment bc = segment("BC", b, c);
        LinearTrajectorySegment alt = segment("ALT", b, d);
        LinearTrajectorySegment cd = segment("CD", c, d);

        FlightTrajectory trajectory = FlightTrajectory.builder().id("FT").callsign("TASH1").build();
        trajectory.addSegment(ab);
        trajectory.addSegment(bc);
        trajectory.addSegment(cd);
        trajectory.addAlternativePath(b, List.of(alt));
        trajectory.joinAlternativePathsAt(d);

        assertTrue(trajectory.getAllSegments().contains(alt));
        assertFalse(trajectory.getAllPossiblePaths().isEmpty());
        assertNotNull(trajectory.getBoundingBox());

        SpatialVolume corridorVolume = SpatialVolume.builder()
                .id("VOL")
                .basePolygon(square("ROUTE-BOX", 39.9, -75.1, 40.6, -74.4, 0))
                .lowerAltitude(9000)
                .upperAltitude(13000)
                .build();
        assertTrue(trajectory.intersectsVolume(corridorVolume));
        assertThrows(IllegalStateException.class,
                () -> FlightTrajectory.builder().id("EMPTY").callsign("NONE").build().getBoundingBox());

        GeoCoordinate start = coord(40, -75, 10000);
        GeoCoordinate end = coord(41, -74, 12000);
        GeoCoordinate middle = GeodeticCalculator.midpoint(start, end);
        GeoCoordinate destination = GeodeticCalculator.destinationPoint(start, 60, 45);
        List<GeoCoordinate> path = GeodeticCalculator.greatCirclePath(start, end, 5);
        SpatialLine routeLine = SpatialLine.builder().id("ROUTE").startPoint(a).endPoint(c).build();
        SpatialLine obstacleLine = SpatialLine.builder()
                .id("OBSTACLE")
                .startPoint(point("O1", 40.1, -75.0, 10000))
                .endPoint(point("O2", 40.1, -74.0, 10000))
                .build();

        assertTrue(GeodeticCalculator.vincentyDistance(start, end) > 0);
        assertTrue(GeodeticCalculator.haversineDistance(start, end) > 0);
        assertTrue(GeodeticCalculator.initialBearing(start, end) >= 0);
        assertTrue(GeodeticCalculator.finalBearing(start, end) >= 0);
        assertTrue(middle.getAltitude() > 10000);
        assertTrue(destination.getLatitude() > 40);
        assertTrue(path.size() == 5);
        assertThrows(IllegalArgumentException.class, () -> GeodeticCalculator.greatCirclePath(start, end, 1));
        assertTrue(Math.abs(GeodeticCalculator.crossTrackDistance(start, end, middle)) < 5);
        assertTrue(GeodeticCalculator.alongTrackDistance(start, end, middle) > 0);
        assertTrue(GeodeticCalculator.isPointOnPathSegment(start, end, middle, 10));
        assertTrue(GeodeticCalculator.polygonArea(List.of(
                coord(40, -75, 0),
                coord(40, -74, 0),
                coord(41, -74, 0),
                coord(41, -75, 0))) > 0);
        assertTrue(GeodeticCalculator.minimumDistance(routeLine, obstacleLine) >= 0);
        assertTrue(GeodeticCalculator.minimumDistance(routeLine, corridorVolume) >= 0);
    }

    private CylindricalVolume cylinder(String id, double lat, double lon, double alt, double radiusNm,
                                       double lowerAlt, double upperAlt) {
        return CylindricalVolume.builder()
                .id(id)
                .base(SpatialCircle.builder().id(id + "-circle").center(point(id + "-center", lat, lon, alt))
                        .radius(radiusNm).build())
                .lowerAltitude(lowerAlt)
                .upperAltitude(upperAlt)
                .build();
    }

    private SpatialPolygon square(String id, double minLat, double minLon, double maxLat, double maxLon,
                                  double altitude) {
        return SpatialPolygon.builder()
                .id(id)
                .vertices(List.of(
                        point(id + "-SW", minLat, minLon, altitude),
                        point(id + "-SE", minLat, maxLon, altitude),
                        point(id + "-NE", maxLat, maxLon, altitude),
                        point(id + "-NW", maxLat, minLon, altitude)))
                .build();
    }

    private LinearTrajectorySegment segment(String id, SpatialPoint source, SpatialPoint target) {
        return LinearTrajectorySegment.builder()
                .id(id)
                .source(source)
                .target(target)
                .startTime(ZonedDateTime.parse("2026-05-20T12:00:00Z"))
                .endTime(ZonedDateTime.parse("2026-05-20T13:00:00Z"))
                .type(TrajectoryType.DIRECT)
                .build();
    }

    private SpatialPoint point(String id, double latitude, double longitude, double altitude) {
        return SpatialPoint.builder().id(id).coordinate(coord(latitude, longitude, altitude)).build();
    }

    private GeoCoordinate coord(double latitude, double longitude, double altitude) {
        return GeoCoordinate.builder().latitude(latitude).longitude(longitude).altitude(altitude).build();
    }
}
