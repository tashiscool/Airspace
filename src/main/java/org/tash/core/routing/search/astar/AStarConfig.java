package org.tash.core.routing.search.astar;

import org.tash.core.routing.search.PlanningConfig;

/**
 * Configuration for A* algorithm
 */
public class AStarConfig implements PlanningConfig {
    private double nodeSpacingNM = 5.0;
    private double altitudeStepFt = 1000.0;
    private double goalReachedThresholdNM = 1.0;
    private int maxIterations = 5000;
    private boolean pathSmoothingEnabled = true;
    private double pathSmoothingToleranceNM = 1.0;
    private double gridSizeNM = 5.0;
    private double altitudeGridFt = 1000.0;
    private double speedKnots = 450.0;
    private ExpansionStrategy expansionStrategy;

    public AStarConfig() {
        this.expansionStrategy = new GridExpansionStrategy();
    }

    // Getters and setters
    public double getNodeSpacingNM() {
        return nodeSpacingNM;
    }

    public void setNodeSpacingNM(double nodeSpacingNM) {
        this.nodeSpacingNM = nodeSpacingNM;
    }

    public double getAltitudeStepFt() {
        return altitudeStepFt;
    }

    public void setAltitudeStepFt(double altitudeStepFt) {
        this.altitudeStepFt = altitudeStepFt;
    }

    public double getGoalReachedThresholdNM() {
        return goalReachedThresholdNM;
    }

    public void setGoalReachedThresholdNM(double goalReachedThresholdNM) {
        this.goalReachedThresholdNM = goalReachedThresholdNM;
    }

    public int getMaxIterations() {
        return maxIterations;
    }

    public void setMaxIterations(int maxIterations) {
        this.maxIterations = maxIterations;
    }

    public boolean isPathSmoothingEnabled() {
        return pathSmoothingEnabled;
    }

    public void setPathSmoothingEnabled(boolean pathSmoothingEnabled) {
        this.pathSmoothingEnabled = pathSmoothingEnabled;
    }

    public double getPathSmoothingToleranceNM() {
        return pathSmoothingToleranceNM;
    }

    public void setPathSmoothingToleranceNM(double pathSmoothingToleranceNM) {
        this.pathSmoothingToleranceNM = pathSmoothingToleranceNM;
    }

    public double getGridSizeNM() {
        return gridSizeNM;
    }

    public void setGridSizeNM(double gridSizeNM) {
        this.gridSizeNM = gridSizeNM;
    }

    public double getAltitudeGridFt() {
        return altitudeGridFt;
    }

    public void setAltitudeGridFt(double altitudeGridFt) {
        this.altitudeGridFt = altitudeGridFt;
    }

    public double getSpeedKnots() {
        return speedKnots;
    }

    public void setSpeedKnots(double speedKnots) {
        this.speedKnots = speedKnots;
    }

    public ExpansionStrategy getExpansionStrategy() {
        return expansionStrategy;
    }

    public void setExpansionStrategy(ExpansionStrategy expansionStrategy) {
        this.expansionStrategy = expansionStrategy;
    }

    /**
     * Create a builder for fluent configuration
     */
    public static Builder builder() {
        return new Builder();
    }

    /**
     * Builder for AStarConfig
     */
    public static class Builder {
        private final AStarConfig config = new AStarConfig();

        public Builder nodeSpacingNM(double nodeSpacingNM) {
            config.setNodeSpacingNM(nodeSpacingNM);
            return this;
        }

        public Builder altitudeStepFt(double altitudeStepFt) {
            config.setAltitudeStepFt(altitudeStepFt);
            return this;
        }

        public Builder goalReachedThresholdNM(double goalReachedThresholdNM) {
            config.setGoalReachedThresholdNM(goalReachedThresholdNM);
            return this;
        }

        public Builder maxIterations(int maxIterations) {
            config.setMaxIterations(maxIterations);
            return this;
        }

        public Builder pathSmoothingEnabled(boolean pathSmoothingEnabled) {
            config.setPathSmoothingEnabled(pathSmoothingEnabled);
            return this;
        }

        public Builder pathSmoothingToleranceNM(double pathSmoothingToleranceNM) {
            config.setPathSmoothingToleranceNM(pathSmoothingToleranceNM);
            return this;
        }

        public Builder gridSizeNM(double gridSizeNM) {
            config.setGridSizeNM(gridSizeNM);
            return this;
        }

        public Builder altitudeGridFt(double altitudeGridFt) {
            config.setAltitudeGridFt(altitudeGridFt);
            return this;
        }

        public Builder speedKnots(double speedKnots) {
            config.setSpeedKnots(speedKnots);
            return this;
        }

        public Builder expansionStrategy(ExpansionStrategy expansionStrategy) {
            config.setExpansionStrategy(expansionStrategy);
            return this;
        }

        public AStarConfig build() {
            return config;
        }
    }
}