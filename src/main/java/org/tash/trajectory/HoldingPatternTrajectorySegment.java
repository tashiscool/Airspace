package org.tash.trajectory;

import lombok.Data;
import lombok.experimental.SuperBuilder;
import org.tash.core.SpatialElement;
import org.tash.core.TemporalElement;
import org.tash.data.Vector3D;
import org.tash.spatial.SpatialLine;
import org.tash.spatial.SpatialPoint;
import org.tash.spatial.SpatialVolume;
import org.tash.time.HoldingPatternMotionModel;
import org.tash.time.MotionModel;

import java.time.ZonedDateTime;
import java.util.Date;

@Data
@SuperBuilder
public class HoldingPatternTrajectorySegment extends AbstractTrajectorySegment {
    SpatialPoint centerPoint;
    double legLength;
    double nautical;
//    double miles;
    double heading;
//    double degrees;
    int turns;
    HoldingPatternMotionModel motionModel;

    @Override
    public MotionModel getMotionModel() {
        return motionModel;
    }

    @Override
    public boolean intersects(SpatialElement other) {
        return super.intersects(other);
    }

    @Override
    public boolean contains(double lat, double lon, double alt) {
        return super.contains(lat, lon, alt);
    }

    @Override
    public SpatialPoint getPointAtFraction(double fraction) {
        return motionModel.getPositionAt(fraction);
    }

    @Override
    public SpatialPoint getPointAtTime(ZonedDateTime time) {
        return super.getPointAtTime(time);
    }

    @Override
    public SpatialLine toSpatialLine() {
        return super.toSpatialLine();
    }

    @Override
    public Vector3D getStartPosition() {
        SpatialPoint s = motionModel.getPositionAt(0.0);
         return new Vector3D(s);
    }

    @Override
    public Vector3D getEndPosition() {
        SpatialPoint e = motionModel.getPositionAt(1.0);
        return new Vector3D(e);
    }

    @Override
    public boolean intersectsVolume(SpatialVolume volume) {
        // Check if the start or end position intersects the volume
        if (volume.contains(centerPoint.getCoordinate().getLongitude(), centerPoint.getCoordinate().getLongitude(), centerPoint.getCoordinate().getAltitude())) {
            return true;
        }

        // Check if any point along the trajectory intersects the volume
        for (double fraction = 0.0; fraction <= 1.0; fraction += 0.01) {
            SpatialPoint point = getPointAtFraction(fraction);
            if (volume.contains(point.getCoordinate().getLatitude(), point.getCoordinate().getLongitude(), point.getCoordinate().getAltitude())) {
                return true;
            }
        }

        return false;
    }

    @Override
    public Date getDuration() {
        return new Date(motionModel.getTimeInterval().getDuration().toMillis());
    }

    @Override
    public boolean timeOverlaps(TemporalElement other) {
        return super.timeOverlaps(other);
    }

    @Override
    public boolean containsTime(ZonedDateTime time) {
        return super.containsTime(time);
    }
}
