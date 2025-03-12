package org.tash.core.routing.search.astar;

import org.tash.spatial.SpatialPoint;
import org.tash.data.GeoCoordinate;

import java.util.List;
import java.util.ArrayList;
import java.util.UUID;

/**
 * Expansion strategy using a fixed grid
 */
public class GridExpansionStrategy implements ExpansionStrategy {
    @Override
    public List<SpatialPoint> expandFrom(SpatialPoint current, SpatialPoint goal, AStarConfig config) {
        List<SpatialPoint> candidates = new ArrayList<>();

        // Get the current coordinate
        GeoCoordinate currentCoord = current.getCoordinate();
        double stepSizeNM = config.getNodeSpacingNM();
        double altStepFt = config.getAltitudeStepFt();

        // Horizontal directions (8 directions)
        double[] bearings = {0, 45, 90, 135, 180, 225, 270, 315};

        // Generate candidates in each horizontal direction
        for (double bearing : bearings) {
            // Create a point at the specified distance and bearing
            GeoCoordinate newCoord = currentCoord.destinationPoint(stepSizeNM, bearing);

            // Create a candidate at the same altitude
            candidates.add(SpatialPoint.builder()
                .id("candidate-" + UUID.randomUUID().toString())
                .coordinate(newCoord)
                .build());

            // If we're not at the target altitude, add candidates with altitude changes
            double targetAlt = goal.getCoordinate().getAltitude();
            if (Math.abs(currentCoord.getAltitude() - targetAlt) > altStepFt) {
                // Determine climb or descent direction
                double altChange = currentCoord.getAltitude() < targetAlt ? altStepFt : -altStepFt;

                // Create a point with altitude change
                GeoCoordinate altCoord = GeoCoordinate.builder()
                    .latitude(newCoord.getLatitude())
                    .longitude(newCoord.getLongitude())
                    .altitude(currentCoord.getAltitude() + altChange)
                    .build();

                candidates.add(SpatialPoint.builder()
                    .id("candidate-alt-" + UUID.randomUUID().toString())
                    .coordinate(altCoord)
                    .build());
            }
        }

        // Also add a direct candidate towards the goal for more efficient routing
        double bearing = currentCoord.initialBearingTo(goal.getCoordinate());
        GeoCoordinate directCoord = currentCoord.destinationPoint(stepSizeNM, bearing);

        // Adjust altitude towards goal altitude
        double altDiff = goal.getCoordinate().getAltitude() - currentCoord.getAltitude();
        double altChange = Math.signum(altDiff) * Math.min(Math.abs(altDiff), altStepFt);

        directCoord = GeoCoordinate.builder()
            .latitude(directCoord.getLatitude())
            .longitude(directCoord.getLongitude())
            .altitude(currentCoord.getAltitude() + altChange)
            .build();

        candidates.add(SpatialPoint.builder()
            .id("candidate-direct-" + UUID.randomUUID().toString())
            .coordinate(directCoord)
            .build());

        return candidates;
    }
}