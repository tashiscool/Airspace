package org.tash;

import org.junit.jupiter.api.Test;
import org.tash.data.GeoCoordinate;
import org.tash.spatial.SpatialArc;
import org.tash.spatial.SpatialPoint;
import org.tash.spatial.SpatialPolygon;
import org.tash.spatial.SpatialVolume;
import org.tash.time.ArcMotionModel;
import org.tash.time.CurvedMotionModel;
import org.tash.time.GreatCircleMotionModel;
import org.tash.time.HelicalMotionModel;
import org.tash.time.HoldingPatternMotionModel;
import org.tash.time.LinearMotionModel;
import org.tash.time.TimeInterval;
import org.tash.trajectory.ArcTrajectorySegment;
import org.tash.trajectory.CurvedTrajectorySegment;
import org.tash.trajectory.GreatCircleTrajectorySegment;
import org.tash.trajectory.HelicalTrajectorySegment;
import org.tash.trajectory.HoldingPatternTrajectorySegment;
import org.tash.trajectory.LinearTrajectorySegment;
import org.tash.trajectory.TrajectoryType;

import java.time.ZonedDateTime;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

class MotionAndTrajectoryCoverageTest {
    private final ZonedDateTime start = ZonedDateTime.parse("2026-05-20T12:00:00Z");
    private final ZonedDateTime end = start.plusHours(1);

    @Test
    void motionModelsClampInterpolateAndExposeTimeIntervals() {
        GeoCoordinate a = geo(0, 0, 1000);
        GeoCoordinate b = geo(1, 1, 3000);
        GeoCoordinate control = geo(1, 0, 5000);
        GeoCoordinate center = geo(0, 0, 2000);

        LinearMotionModel linear = LinearMotionModel.builder()
                .startPosition(a).endPosition(b).startTime(start).endTime(end).build();
        assertEquals(a, linear.getPositionAt(start.minusMinutes(1)));
        assertEquals(b, linear.getPositionAt(end.plusMinutes(1)));
        assertEquals(0.5, linear.getPositionAt(start.plusMinutes(30)).getLatitude(), 0.0001);

        CurvedMotionModel curved = CurvedMotionModel.builder()
                .startPosition(a).endPosition(b).controlPoint(control).startTime(start).endTime(end).build();
        assertEquals(0.75, curved.getPositionAt(start.plusMinutes(30)).getLatitude(), 0.0001);
        assertEquals(3500.0, curved.getPositionAt(start.plusMinutes(30)).getAltitude(), 0.0001);

        ArcMotionModel arc = ArcMotionModel.builder()
                .center(center).radius(60).startAngle(0).endAngle(90).isClockwise(false)
                .startTime(start).endTime(end).build();
        assertTrue(arc.getPositionAt(start.plusMinutes(30)).getLatitude() > center.getLatitude());
        assertTrue(arc.getTimeInterval().contains(start.plusMinutes(5)));

        GreatCircleMotionModel greatCircle = GreatCircleMotionModel.builder()
                .startPosition(a).endPosition(b).startTime(start).endTime(end).numIntermediatePoints(4).build();
        assertFalse(greatCircle.getPath().isEmpty());
        assertTrue(greatCircle.getTotalDistance() > 0);
        assertTrue(greatCircle.getInitialBearing() >= 0);
        assertTrue(greatCircle.getFinalBearing() >= 0);

        HelicalMotionModel helix = HelicalMotionModel.builder()
                .center(center).radius(20).startAngle(0).endAngle(180)
                .startAltitude(1000).endAltitude(5000).clockwise(false)
                .startTime(start).endTime(end).build();
        assertEquals(3000.0, helix.getPositionAtFraction(0.5).getAltitude(), 0.0001);
        assertEquals(5, helix.getHelixPoints(5).size());
        assertEquals(5, helix.getHelixVectors(5).size());

        HoldingPatternMotionModel holding = HoldingPatternMotionModel.builder()
                .centerPoint(center).legLength(10).heading(90).turns(2)
                .entryPoint(a).exitPoint(b).startTime(start).endTime(end).build();
        assertEquals(a, holding.getPositionAt(start.minusSeconds(1)));
        assertEquals(b, holding.getPositionAt(end.plusSeconds(1)));
        assertNotNull(holding.getPositionAt(0.25));

        TimeInterval interval = new TimeInterval(start, end);
        assertTrue(interval.overlaps(new TimeInterval(start.plusMinutes(30), end.plusMinutes(30))));
        assertEquals(60, interval.getDuration().toMinutes());
    }

    @Test
    void trajectorySegmentsBuildMotionModelsBoundingBoxesAndVolumeIntersections() {
        SpatialPoint source = point("source", 0, 0, 1000);
        SpatialPoint target = point("target", 1, 1, 3000);
        SpatialPoint control = point("control", 1, 0, 4000);
        SpatialPoint center = point("center", 0, 0, 2000);
        SpatialVolume volume = volumeAroundOrigin();

        LinearTrajectorySegment linear = LinearTrajectorySegment.builder()
                .id("linear").source(source).target(target).type(TrajectoryType.DIRECT)
                .startTime(start).endTime(end).build();
        assertEquals(0.5, linear.getPointAtFraction(0.5).getCoordinate().getLatitude(), 0.0001);
        assertTrue(linear.intersectsVolume(volume));
        assertNotNull(linear.toSpatialLine());
        assertTrue(linear.getDuration().getTime() > 0);

        CurvedTrajectorySegment curved = CurvedTrajectorySegment.builder()
                .id("curved").source(source).target(target).controlPoint(control)
                .type(TrajectoryType.STANDARD).startTime(start).endTime(end).build();
        assertNotNull(curved.getMotionModel());
        assertTrue(curved.getBoundingBox().contains(control.getCoordinate()));
        assertTrue(curved.intersectsVolume(volume));

        SpatialArc arcShape = SpatialArc.builder().id("arc").center(center).radius(60)
                .startAngle(0).endAngle(180).isClockwise(false).build();
        ArcTrajectorySegment arc = ArcTrajectorySegment.builder()
                .id("arc-traj").source(source).target(target).arc(arcShape)
                .type(TrajectoryType.STANDARD).startTime(start).endTime(end).build();
        assertNotNull(arc.getMotionModel());
        assertTrue(arc.getBoundingBox().contains(arc.getPointAtFraction(0.5).getCoordinate()));

        GreatCircleTrajectorySegment greatCircle = GreatCircleTrajectorySegment.builder()
                .id("gc").source(source).target(target).numIntermediatePoints(5)
                .type(TrajectoryType.DIRECT).startTime(start).endTime(end).build();
        assertNotNull(greatCircle.getMotionModel());
        assertTrue(greatCircle.getDuration().getTime() > 0);

        HelicalTrajectorySegment helix = HelicalTrajectorySegment.builder()
                .id("helix").source(source).target(target).center(center).radius(20)
                .startAngle(0).endAngle(180).startAltitude(1000).endAltitude(5000)
                .clockwise(false).type(TrajectoryType.STANDARD).startTime(start).endTime(end).build();
        assertEquals(0.5, helix.getNumberOfTurns(), 0.0001);
        assertTrue(helix.getVerticalRate() > 0);
        assertTrue(helix.get3DDistance() > helix.getGroundTrackDistance());

        HoldingPatternTrajectorySegment holding = HoldingPatternTrajectorySegment.builder()
                .id("hold").source(source).target(target).centerPoint(center)
                .legLength(10).heading(90).turns(1).type(TrajectoryType.HOLDING)
                .startTime(start).endTime(end).build();
        holding.getMotionModel();
        assertNotNull(holding.getPointAtFraction(0.5));
        assertTrue(holding.intersectsVolume(volume));
    }

    private SpatialVolume volumeAroundOrigin() {
        return SpatialVolume.builder()
                .id("volume")
                .basePolygon(SpatialPolygon.builder().id("base").vertices(List.of(
                        point("a", -0.25, -0.25, 1000),
                        point("b", -0.25, 0.25, 1000),
                        point("c", 0.25, 0.25, 1000),
                        point("d", 0.25, -0.25, 1000))).build())
                .lowerAltitude(0)
                .upperAltitude(6000)
                .startTime(start)
                .endTime(end)
                .build();
    }

    private SpatialPoint point(String id, double lat, double lon, double alt) {
        return SpatialPoint.builder().id(id).coordinate(geo(lat, lon, alt)).build();
    }

    private GeoCoordinate geo(double lat, double lon, double alt) {
        return GeoCoordinate.builder().latitude(lat).longitude(lon).altitude(alt).build();
    }
}
