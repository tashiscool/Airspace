package org.tash.extensions.weather.avoid;

import lombok.Data;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.geodetic.GeodeticCalculator;
import org.tash.extensions.weather.HazardousWeather;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Collectors;

/**
 * Simple deviation strategy that goes around circular hazards
 */
@Data
public class SimpleDeviationStrategy implements WeatherAvoidanceStrategy {
    /**
     * Default buffer distance in nautical miles
     */
    private static final double DEFAULT_BUFFER = 5.0;

    /**
     * Buffer distance to add around hazards
     */
    private final double bufferDistance;

    /**
     * Maximum number of intermediate waypoints to try
     */
    private final int maxWaypoints;

    /**
     * Create a simple deviation strategy with default settings
     */
    public SimpleDeviationStrategy() {
        this(DEFAULT_BUFFER, 3);
    }

    /**
     * Create a simple deviation strategy with custom settings
     *
     * @param bufferDistance Buffer distance in nautical miles
     * @param maxWaypoints   Maximum number of intermediate waypoints
     */
    public SimpleDeviationStrategy(double bufferDistance, int maxWaypoints) {
        this.bufferDistance = bufferDistance;
        this.maxWaypoints = maxWaypoints;
    }

    @Override
    public List<GeoCoordinate> findAvoidancePath(GeoCoordinate start, GeoCoordinate end,
                                                 ZonedDateTime time, List<HazardousWeather> hazards) {
        // If direct path is safe, use it
        if (isDirectPathSafe(start, end, time, hazards)) {
            List<GeoCoordinate> list = new ArrayList<>();
            list.add(start);
            list.add(end);
            return list;
        }

        // Filter to only weather cells and sort by distance from path
        List<WeatherCell> cells = hazards.stream()
                .filter(h -> h instanceof WeatherCell)
                .map(h -> (WeatherCell) h)
                .sorted(Comparator.comparingDouble(c -> getMinimumDistanceFromPath(c, start, end)))
                .collect(Collectors.toList());

        // Special case: if we're inside a cell, try to exit directly
        for (WeatherCell cell : cells) {
            if (cell.affectsPoint(start, time)) {
                // Find exit point
                GeoCoordinate exitPoint = findExitPoint(cell, start, end);

                // Recursively find path from exit point to end
                List<GeoCoordinate> remainingPath = findAvoidancePath(
                        exitPoint, end, time, hazards);

                // Combine paths
                List<GeoCoordinate> fullPath = new ArrayList<>();
                fullPath.add(start);
                fullPath.addAll(remainingPath);
                return fullPath;
            }
        }

        // Try deviating around the nearest cell
        if (!cells.isEmpty()) {
            WeatherCell nearest = cells.get(0);

            // Find deviation waypoints
            List<GeoCoordinate> deviationPoints = findDeviationPoints(nearest, start, end);

            // Try each deviation point
            for (GeoCoordinate waypoint : deviationPoints) {
                // Check path from start to waypoint
                if (isDirectPathSafe(start, waypoint, time, hazards)) {
                    // Check if we need to recursively find path from waypoint to end
                    if (isDirectPathSafe(waypoint, end, time, hazards)) {
                        List<GeoCoordinate> list = new ArrayList<>();
                        list.add(start);
                        list.add(waypoint);
                        list.add(end);
                        return list;
                    } else {
                        // Recursively find path from waypoint to end
                        List<GeoCoordinate> remainingPath = findAvoidancePath(
                                waypoint, end, time, hazards);

                        // Combine paths
                        List<GeoCoordinate> fullPath = new ArrayList<>();
                        fullPath.add(start);
                        fullPath.addAll(remainingPath.subList(0, remainingPath.size()));
                        return fullPath;
                    }
                }
            }
        }

        // If no good path found, just return direct path (probably not safe)
        List<GeoCoordinate> list = new ArrayList<>();
        list.add(start);
        list.add(end);
        return list;
    }

    /**
     * Find the minimum distance from a cell to a path
     *
     * @param cell  Weather cell
     * @param start Start point of path
     * @param end   End point of path
     * @return Minimum distance in nautical miles
     */
    private double getMinimumDistanceFromPath(WeatherCell cell, GeoCoordinate start, GeoCoordinate end) {
        if (cell instanceof CircularWeatherCell) {
            CircularWeatherCell circularCell = (CircularWeatherCell) cell;
            double crossTrack = GeodeticCalculator.crossTrackDistance(
                    start, end, circularCell.getCenter());
            return Math.max(0, Math.abs(crossTrack) - circularCell.getRadius());
        } else {
            // For non-circular cells, use the center point as an approximation
            List<GeoCoordinate> boundary = cell.getBoundaryPoints();
            if (boundary.isEmpty()) {
                return Double.MAX_VALUE;
            }

            // Calculate centroid
            double sumLat = 0;
            double sumLon = 0;
            for (GeoCoordinate point : boundary) {
                sumLat += point.getLatitude();
                sumLon += point.getLongitude();
            }
            GeoCoordinate center = GeoCoordinate.builder()
                    .latitude(sumLat / boundary.size())
                    .longitude(sumLon / boundary.size())
                    .altitude((cell.getMinAltitude() + cell.getMaxAltitude()) / 2)
                    .build();

            // Calculate distance from path
            double crossTrack = Math.abs(GeodeticCalculator.crossTrackDistance(start, end, center));

            // Get approx radius
            double maxDist = 0;
            for (GeoCoordinate point : boundary) {
                double dist = center.distanceTo(point);
                if (dist > maxDist) {
                    maxDist = dist;
                }
            }

            return Math.max(0, crossTrack - maxDist);
        }
    }

    /**
     * Find an exit point from a weather cell
     *
     * @param cell  Weather cell to exit
     * @param start Start point inside the cell
     * @param end   End point outside the cell
     * @return Exit point on the boundary of the cell
     */
    private GeoCoordinate findExitPoint(WeatherCell cell, GeoCoordinate start, GeoCoordinate end) {
        // For circular cells, we can calculate this directly
        if (cell instanceof CircularWeatherCell) {
            CircularWeatherCell circularCell = (CircularWeatherCell) cell;

            // Get bearing from center to end
            double bearing = circularCell.getCenter().initialBearingTo(end);

            // Calculate exit point on boundary
            return circularCell.getCenter().destinationPoint(
                    circularCell.getRadius() + bufferDistance, bearing);
        } else {
            // For polygonal cells, find the intersection with the boundary
            List<GeoCoordinate> boundary = cell.getBoundaryPoints();

            // Find shortest path to outside
            GeoCoordinate bestExit = end; // Default to end point
            double bestDistance = Double.MAX_VALUE;

            for (int i = 0; i < boundary.size() - 1; i++) {
                GeoCoordinate p1 = boundary.get(i);
                GeoCoordinate p2 = boundary.get(i + 1);

                // Find closest point on this segment
                GeoCoordinate closestPoint = findClosestPointOnSegment(p1, p2, start);

                // Add buffer distance
                double bearing = start.initialBearingTo(closestPoint);
                GeoCoordinate bufferPoint = closestPoint.destinationPoint(bufferDistance, bearing);

                // Check if this is a better exit point
                double distance = start.distanceTo(bufferPoint) + bufferPoint.distanceTo(end);
                if (distance < bestDistance) {
                    bestDistance = distance;
                    bestExit = bufferPoint;
                }
            }

            return bestExit;
        }
    }

    /**
     * Find the closest point on a line segment to a given point
     *
     * @param p1 First endpoint of segment
     * @param p2 Second endpoint of segment
     * @param p  Point to find closest point to
     * @return Closest point on segment
     */
    private GeoCoordinate findClosestPointOnSegment(GeoCoordinate p1, GeoCoordinate p2,
                                                    GeoCoordinate p) {
        // Convert to a local Cartesian coordinate system for simplicity
        // We'll use a flat earth approximation since segments are typically short
        double x1 = p1.getLatitude();
        double y1 = p1.getLongitude();
        double x2 = p2.getLatitude();
        double y2 = p2.getLongitude();
        double x = p.getLatitude();
        double y = p.getLongitude();

        // Calculate projection of point onto line
        double dx = x2 - x1;
        double dy = y2 - y1;
        double t = ((x - x1) * dx + (y - y1) * dy) / (dx * dx + dy * dy);

        // Clamp t to segment
        t = Math.max(0, Math.min(1, t));

        // Calculate closest point
        double projX = x1 + t * dx;
        double projY = y1 + t * dy;

        // Convert back to GeoCoordinate
        return GeoCoordinate.builder()
                .latitude(projX)
                .longitude(projY)
                .altitude(p.getAltitude())
                .build();
    }

    /**
     * Find deviation points to go around a weather cell
     *
     * @param cell  Weather cell to avoid
     * @param start Start point
     * @param end   End point
     * @return List of potential deviation points
     */
    private List<GeoCoordinate> findDeviationPoints(WeatherCell cell, GeoCoordinate start, GeoCoordinate end) {
        List<GeoCoordinate> points = new ArrayList<>();

        if (cell instanceof CircularWeatherCell) {
            CircularWeatherCell circularCell = (CircularWeatherCell) cell;
            GeoCoordinate center = circularCell.getCenter();

            // Calculate deviation distance (cell radius plus buffer)
            double deviationDist = circularCell.getRadius() + bufferDistance;

            // Calculate bearing from start to end
            double directBearing = start.initialBearingTo(end);

            // Calculate perpendicular bearings for deviation
            double leftBearing = (directBearing - 90 + 360) % 360;
            double rightBearing = (directBearing + 90) % 360;

            // Calculate the closest point on the direct path to the center
            GeoCoordinate closestPoint = center.closestPointOnPath(start, end);

            // Calculate deviation points on either side of the cell
            GeoCoordinate leftPoint = closestPoint.destinationPoint(deviationDist, leftBearing);
            GeoCoordinate rightPoint = closestPoint.destinationPoint(deviationDist, rightBearing);

            // Add both points
            points.add(leftPoint);
            points.add(rightPoint);

            // Check which side is shorter
            double leftDist = start.distanceTo(leftPoint) + leftPoint.distanceTo(end);
            double rightDist = start.distanceTo(rightPoint) + rightPoint.distanceTo(end);

            // Sort by distance (shortest first)
            if (leftDist <= rightDist) {
                points.add(0, leftPoint);
                points.add(rightPoint);
            } else {
                points.add(0, rightPoint);
                points.add(leftPoint);
            }
        } else {
            // For polygonal cells, use the vertices as potential waypoints
            List<GeoCoordinate> boundary = cell.getBoundaryPoints();

            // Sort vertices by total distance through the waypoint
            points = boundary.stream()
                    .map(p -> GeoCoordinate.builder()
                            .latitude(p.getLatitude())
                            .longitude(p.getLongitude())
                            .altitude((cell.getMinAltitude() + cell.getMaxAltitude()) / 2)
                            .build())
                    .sorted(Comparator.comparingDouble(p ->
                            start.distanceTo(p) + p.distanceTo(end)))
                    .limit(maxWaypoints)
                    .collect(Collectors.toList());
        }

        return points;
    }
}
