package org.tash;

import org.junit.jupiter.api.Test;
import org.tash.core.uncertainty.FuzzyConflictResolver;
import org.tash.core.uncertainty.ResolutionAction;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.carf.refdata.PgRestoreCarfDumpExtractor;
import org.tash.extensions.carf.refdata.PostgresDumpRowExtractor;
import org.tash.spatial.SpatialPoint;
import org.tash.spatial.threedee.ConicalVolume;
import org.tash.spatial.threedee.FrustumVolume;
import org.tash.trajectory.LinearTrajectorySegment;

import java.nio.file.Paths;
import java.time.ZonedDateTime;

import static org.junit.jupiter.api.Assertions.*;

class FrameworkHardeningTest {
    @Test
    void fuzzyConflictResolverReturnsActionInsteadOfNull() {
        LinearTrajectorySegment first = segment("A", point(30, -150, 24000), point(30, -149, 24000));
        LinearTrajectorySegment second = segment("B", point(30.01, -150.01, 24500), point(30.01, -149.01, 24500));

        ResolutionAction action = new FuzzyConflictResolver().resolveConflicts(first, second);

        assertNotNull(action);
        assertTrue(action.getRiskScore() > 0);
        assertTrue(action.getHeadingChangeDegrees() > 0 || action.getAltitudeChangeFeet() > 0);
        assertNotNull(action.getRationale());
    }

    @Test
    void conicalAndFrustumContainmentDoNotRejectJustBecauseVolumeHasTimeBounds() {
        ZonedDateTime start = ZonedDateTime.parse("2010-01-01T00:00:00Z");
        ZonedDateTime end = ZonedDateTime.parse("2010-01-01T01:00:00Z");
        ConicalVolume cone = ConicalVolume.builder()
                .id("CONE")
                .apex(SpatialPoint.builder().id("APEX").coordinate(point(30, -150, 1000)).build())
                .baseAltitude(5000)
                .baseRadius(20)
                .centralAxisBearing(0)
                .slopeAngle(0)
                .startTime(start)
                .endTime(end)
                .build();
        FrustumVolume frustum = FrustumVolume.builder()
                .id("FRUSTUM")
                .baseCenter(SpatialPoint.builder().id("BASE").coordinate(point(30, -150, 1000)).build())
                .baseAltitude(1000)
                .topAltitude(5000)
                .baseRadius(10)
                .topRadius(20)
                .centralAxisBearing(90)
                .length(20)
                .startTime(start)
                .endTime(end)
                .build();

        assertTrue(cone.containsPoint(point(30.001, -150, 2000)));
        assertTrue(frustum.containsPoint(point(30, -150, 1000)));
    }

    @Test
    void pgRestoreExtractorReportsAvailabilityWithoutHardRequirement() throws Exception {
        PostgresDumpRowExtractor.Extraction extraction = new PgRestoreCarfDumpExtractor()
                .extract(Paths.get("/tmp/nonexistent-training.backup"));

        assertNotNull(extraction.getDiagnostic());
        if (!extraction.isAvailable()) {
            assertTrue(extraction.getDiagnostic().contains("pg_restore")
                    || extraction.getDiagnostic().contains("failed"));
        }
    }

    private LinearTrajectorySegment segment(String id, GeoCoordinate start, GeoCoordinate end) {
        return LinearTrajectorySegment.builder()
                .id(id)
                .source(SpatialPoint.builder().id(id + "-S").coordinate(start).build())
                .target(SpatialPoint.builder().id(id + "-T").coordinate(end).build())
                .startTime(ZonedDateTime.parse("2010-01-01T00:00:00Z"))
                .endTime(ZonedDateTime.parse("2010-01-01T01:00:00Z"))
                .build();
    }

    private GeoCoordinate point(double latitude, double longitude, double altitude) {
        return GeoCoordinate.builder().latitude(latitude).longitude(longitude).altitude(altitude).build();
    }
}
