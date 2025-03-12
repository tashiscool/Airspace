package org.tash.spatial.threedee;

import org.tash.core.SpatialElement;
import org.tash.spatial.SpatialPoint;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

/**
 * Factory methods for common compound volumes
 */
public class CompoundVolumeFactory {
    public static CompoundVolume createUnion(SpatialElement... volumes) {
        return new CompoundVolume(OperationType.UNION, volumes);
    }

    public static CompoundVolume createIntersect(SpatialElement... volumes) {
        return new CompoundVolume(OperationType.INTERSECT, volumes);
    }

    public static CompoundVolume createSubtract(SpatialElement... volumes) {
        return new CompoundVolume(OperationType.SUBTRACT, volumes);
    }

    /**
     * Create a corridor volume with a specified width and height
     *
     * @param id Volume ID
     * @param waypointList List of waypoints defining the corridor's centerline
     * @param width Width of the corridor in nautical miles
     * @param minAltitude Lower altitude bound in feet
     * @param maxAltitude Upper altitude bound in feet
     * @param startTime Start time
     * @param endTime End time
     * @return A compound volume representing the corridor
     */
    public static CompoundVolume createCorridor(
            String id,
            List<SpatialPoint> waypointList,
            double width,
            double minAltitude,
            double maxAltitude,
            ZonedDateTime startTime,
            ZonedDateTime endTime) {
        List<SpatialElement> volumes = new ArrayList<>();
        for (int i = 0; i < waypointList.size() - 1; i++) {
            SpatialPoint start = waypointList.get(i);
            SpatialPoint end = waypointList.get(i + 1);
            SpatialElement volume = new CylindricalVolume(
                    id + "_seg" + i,
                    start.getCoordinate().destinationPoint(width / 2, start.getCoordinate().bearingTo(end.getCoordinate())),
                    minAltitude,
                    maxAltitude,
                    startTime,
                    endTime);
            volumes.add(volume);
        }
        return createUnion(volumes.toArray(new SpatialElement[0]));
    }
}
