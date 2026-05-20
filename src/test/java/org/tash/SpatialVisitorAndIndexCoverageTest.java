package org.tash;

import org.junit.jupiter.api.Test;
import org.tash.core.AirspaceElement;
import org.tash.core.SpatialElement;
import org.tash.data.BoundingBox;
import org.tash.data.GeoCoordinate;
import org.tash.spatial.SpatialArc;
import org.tash.spatial.SpatialCircle;
import org.tash.spatial.SpatialLine;
import org.tash.spatial.SpatialPoint;
import org.tash.spatial.SpatialPolygon;
import org.tash.spatial.SpatialVolume;
import org.tash.spatial.index.QuadtreeSpatialIndex;
import org.tash.trajectory.LinearTrajectorySegment;
import org.tash.trajectory.TrajectoryType;
import org.tash.visitor.CountingVisitor;
import org.tash.visitor.FinderVisitor;
import org.tash.visitor.IntersectionVisitor;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

class SpatialVisitorAndIndexCoverageTest {
    @Test
    void visitorsCountFindAndIntersectCommonSpatialShapes() {
        SpatialPoint center = point("center", 0, 0, 1000);
        SpatialPoint east = point("east", 0, 1, 1000);
        SpatialPoint north = point("north", 1, 0, 1000);
        SpatialLine diagonal = line("diag", point("sw", -1, -1, 1000), point("ne", 1, 1, 1000));
        SpatialLine crossing = line("cross", point("nw", 1, -1, 1000), point("se", -1, 1, 1000));
        SpatialPolygon square = square("square", -0.5, -0.5, 0.5, 0.5, 1000);
        SpatialVolume volume = SpatialVolume.builder()
                .id("volume")
                .basePolygon(square)
                .lowerAltitude(500)
                .upperAltitude(1500)
                .startTime(ZonedDateTime.parse("2026-05-20T12:00:00Z"))
                .endTime(ZonedDateTime.parse("2026-05-20T13:00:00Z"))
                .build();
        SpatialCircle circle = SpatialCircle.builder().id("circle").center(center).radius(90).build();
        SpatialArc arc = SpatialArc.builder().id("arc").center(center).radius(60)
                .startAngle(0).endAngle(90).isClockwise(false).build();

        CountingVisitor counting = new CountingVisitor();
        for (AirspaceElement element : List.of(center, diagonal, square, volume, circle, arc,
                linear("traj", point("a", -1, 0, 1000), point("b", 1, 0, 1000)))) {
            counting.visit(element);
        }
        assertEquals(1, counting.getPointCount());
        assertEquals(1, counting.getLineCount());
        assertEquals(1, counting.getPolygonCount());
        assertEquals(1, counting.getVolumeCount());
        assertEquals(1, counting.getTrajectoryCount());

        FinderVisitor finder = new FinderVisitor("diag");
        finder.visit(diagonal);
        assertSame(diagonal, finder.getFoundElement());

        assertIntersects(diagonal, crossing);
        assertIntersects(center, square);
        assertIntersects(square, volume);
        assertIntersects(volume, point("inside-volume", 0, 0, 1000));
        assertIntersects(east, circle);
        assertIntersects(diagonal, circle);
        assertIntersects(square, circle);
        assertIntersects(volume, circle);
        assertIntersects(arc.getPointAtAngle(45), arc);
        assertIntersects(arc, SpatialArc.builder().id("arc2").center(center).radius(60)
                .startAngle(45).endAngle(135).isClockwise(false).build());
        assertIntersects(square, linear("traj2", point("t0", -1, 0, 1000), point("t1", 1, 0, 1000)));

        assertDoesNotIntersect(line("parallel-a", point("p1", 2, 0, 1000), point("p2", 3, 0, 1000)), square);
        assertDoesNotIntersect(volume, point("above-volume", 0, 0, 3000));
        assertDoesNotIntersect(circle, SpatialCircle.builder().id("far-circle")
                .center(point("far", 5, 5, 1000)).radius(10).build());
        assertThrows(IllegalArgumentException.class,
                () -> new IntersectionVisitor(linear("unsupported-source", center, north)).visit(circle));
    }

    @Test
    void quadtreeSplitsMergesQueriesAndClearsSpatialElements() {
        QuadtreeSpatialIndex index = new QuadtreeSpatialIndex();
        List<SpatialPoint> points = new ArrayList<>();
        for (int i = 0; i < 18; i++) {
            SpatialPoint p = point("q-" + i, -10 + i, -20 + i, i * 100);
            points.add(p);
            index.add(p);
        }

        BoundingBox middle = BoundingBox.builder()
                .minLat(-1).maxLat(4)
                .minLon(-11).maxLon(-6)
                .minAlt(-1000).maxAlt(100000)
                .build();
        assertFalse(index.query(middle).isEmpty());
        assertEquals(index.query(middle).size(), index.findElementsWithin(middle).size());

        for (SpatialPoint point : points.subList(0, 8)) {
            index.remove(point);
        }
        assertTrue(index.query(points.get(12)).contains(points.get(12)));

        index.clear();
        assertTrue(index.query(BoundingBox.builder()
                .minLat(-90).maxLat(90).minLon(-180).maxLon(180).minAlt(-1000).maxAlt(100000)
                .build()).isEmpty());
    }

    private void assertIntersects(SpatialElement left, SpatialElement right) {
        assertTrue(left.intersects(right), left.getId() + " should intersect " + right.getId());
    }

    private void assertDoesNotIntersect(SpatialElement left, SpatialElement right) {
        assertFalse(left.intersects(right), left.getId() + " should not intersect " + right.getId());
    }

    private LinearTrajectorySegment linear(String id, SpatialPoint start, SpatialPoint end) {
        return LinearTrajectorySegment.builder()
                .id(id)
                .source(start)
                .target(end)
                .startTime(ZonedDateTime.parse("2026-05-20T12:00:00Z"))
                .endTime(ZonedDateTime.parse("2026-05-20T13:00:00Z"))
                .type(TrajectoryType.DIRECT)
                .build();
    }

    private SpatialPolygon square(String id, double minLat, double minLon, double maxLat, double maxLon, double altitude) {
        return SpatialPolygon.builder()
                .id(id)
                .vertices(List.of(
                        point(id + "-a", minLat, minLon, altitude),
                        point(id + "-b", minLat, maxLon, altitude),
                        point(id + "-c", maxLat, maxLon, altitude),
                        point(id + "-d", maxLat, minLon, altitude)))
                .build();
    }

    private SpatialLine line(String id, SpatialPoint start, SpatialPoint end) {
        return SpatialLine.builder().id(id).startPoint(start).endPoint(end).build();
    }

    private SpatialPoint point(String id, double lat, double lon, double alt) {
        return SpatialPoint.builder()
                .id(id)
                .coordinate(GeoCoordinate.builder().latitude(lat).longitude(lon).altitude(alt).build())
                .build();
    }
}
