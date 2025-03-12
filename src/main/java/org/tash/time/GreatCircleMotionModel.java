package org.tash.time;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.NoArgsConstructor;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.geodetic.GeodeticCalculator;

import java.time.ZonedDateTime;
import java.util.List;

/**
 * Motion model for great circle path trajectory
 */
@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class GreatCircleMotionModel implements MotionModel {
    private GeoCoordinate startPosition;
    private GeoCoordinate endPosition;
    private ZonedDateTime startTime;
    private ZonedDateTime endTime;
    @Builder.Default
    private int numIntermediatePoints = 10;

    private List<GeoCoordinate> path; // Cached path

    /**
     * Get the pre-computed path or compute it if not available
     */
    public List<GeoCoordinate> getPath() {
        if (path == null) {
            // Compute and cache the path
            path = GeodeticCalculator.greatCirclePath(
                    startPosition, endPosition, numIntermediatePoints + 2);
        }
        return path;
    }

    @Override
    public GeoCoordinate getPositionAt(ZonedDateTime time) {
        // Check if time is outside the segment timeframe
        if (time.isBefore(startTime)) {
            return startPosition;
        } else if (time.isAfter(endTime)) {
            return endPosition;
        }

        // Calculate the time fraction (0 to 1)
        double totalMillis = java.time.Duration.between(startTime, endTime).toMillis();
        double elapsedMillis = java.time.Duration.between(startTime, time).toMillis();
        double fraction = totalMillis > 0 ? elapsedMillis / totalMillis : 0;

        // Get the path
        List<GeoCoordinate> path = getPath();

        // Find the appropriate segment in the path
        int index = (int) Math.floor(fraction * (path.size() - 1));
        index = Math.min(index, path.size() - 2);

        double segmentFraction = (fraction * (path.size() - 1)) - index;

        // Interpolate between the two points in the segment
        GeoCoordinate pos1 = path.get(index);
        GeoCoordinate pos2 = path.get(index + 1);

        return pos1.interpolate(pos2, segmentFraction);
    }

    @Override
    public TimeInterval getTimeInterval() {
        return new TimeInterval(startTime, endTime);
    }

    /**
     * Get the total great circle distance of this path
     *
     * @return Distance in nautical miles
     */
    public double getTotalDistance() {
        return GeodeticCalculator.vincentyDistance(startPosition, endPosition);
    }

    /**
     * Get the initial bearing of this path
     *
     * @return Bearing in degrees
     */
    public double getInitialBearing() {
        return GeodeticCalculator.initialBearing(startPosition, endPosition);
    }

    /**
     * Get the final bearing of this path
     *
     * @return Bearing in degrees
     */
    public double getFinalBearing() {
        return GeodeticCalculator.finalBearing(startPosition, endPosition);
    }
}
