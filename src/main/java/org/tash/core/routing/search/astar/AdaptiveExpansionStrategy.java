package org.tash.core.routing.search.astar;

import org.tash.data.GeoCoordinate;
import org.tash.spatial.SpatialPoint;

import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

/**
 * Expansion strategy that adjusts node spacing based on distance to goal
 */
public class AdaptiveExpansionStrategy implements ExpansionStrategy {
    @Override
    public List<SpatialPoint> expandFrom(SpatialPoint current, SpatialPoint goal, AStarConfig config) {
        List<SpatialPoint> candidates = new ArrayList<>();

        // Get the current coordinate
        GeoCoordinate currentCoord = current.getCoordinate();
        double distanceToGoal = currentCoord.distanceTo(goal.getCoordinate());

        // Adjust node spacing based on distance to goal
        double stepSizeNM;
        if (distanceToGoal > 100) {
            // Far from goal, use larger steps
            stepSizeNM = config.getNodeSpacingNM() * 2.0;
        } else if (distanceToGoal > 50) {
            // Medium distance, use normal steps
            stepSizeNM = config.getNodeSpacingNM();
        } else if (distanceToGoal > 10) {
            // Getting closer, use smaller steps
            stepSizeNM = config.getNodeSpacingNM() * 0.5;
        } else {
            // Very close, use precise steps
            stepSizeNM = config.getNodeSpacingNM() * 0.25;
        }

        double altStepFt = config.getAltitudeStepFt();

        // Horizontal directions (varying number based on distance)
        List<Double> bearings = new ArrayList<>();
        if (distanceToGoal > 50) {
            // Far away, use 8 directions
            for (int i = 0; i < 8; i++) {
                bearings.add(i * 45.0);
            }
        } else {
            // Closer, use 16 directions for more precision
            for (int i = 0; i < 16; i++) {
                bearings.add(i * 22.5);
            }
        }

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

        // Always add a direct candidate towards the goal
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