package org.tash.core.routing.search.rrt;
/**
 * Rapidly-exploring Random Trees (RRT) algorithm for complex airspace navigation
 * This implementation includes RRT* optimizations for path quality
 */


import org.tash.core.routing.search.PlanningConfig;

/**
     * Configuration for RRT algorithm
     */
    public class RRTConfig implements PlanningConfig {
        private double stepSizeNM = 5.0;
        private double goalReachedThresholdNM = 1.0;
        private int maxIterations = 5000;
        private double goalBias = 0.1;
        private boolean useRRTStar = true;
        private boolean pathSmoothingEnabled = true;
        private double pathSmoothingToleranceNM = 1.0;
        private double searchSpacePaddingDeg = 0.5;
        private double searchSpaceAltPaddingFt = 5000.0;
        private double speedKnots = 450.0;

        // Getters and setters
        public double getStepSizeNM() {
            return stepSizeNM;
        }

        public void setStepSizeNM(double stepSizeNM) {
            this.stepSizeNM = stepSizeNM;
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

        public double getGoalBias() {
            return goalBias;
        }

        public void setGoalBias(double goalBias) {
            this.goalBias = goalBias;
        }

        public boolean isUseRRTStar() {
            return useRRTStar;
        }

        public void setUseRRTStar(boolean useRRTStar) {
            this.useRRTStar = useRRTStar;
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

        public double getSearchSpacePaddingDeg() {
            return searchSpacePaddingDeg;
        }

        public void setSearchSpacePaddingDeg(double searchSpacePaddingDeg) {
            this.searchSpacePaddingDeg = searchSpacePaddingDeg;
        }

        public double getSearchSpaceAltPaddingFt() {
            return searchSpaceAltPaddingFt;
        }

        public void setSearchSpaceAltPaddingFt(double searchSpaceAltPaddingFt) {
            this.searchSpaceAltPaddingFt = searchSpaceAltPaddingFt;
        }

        public double getSpeedKnots() {
            return speedKnots;
        }

        public void setSpeedKnots(double speedKnots) {
            this.speedKnots = speedKnots;
        }

        /**
         * Create a builder for fluent configuration
         */
        public static Builder builder() {
            return new Builder();
        }

        /**
         * Builder for RRTConfig
         */
        public static class Builder {
            private final RRTConfig config = new RRTConfig();

            public Builder stepSizeNM(double stepSizeNM) {
                config.setStepSizeNM(stepSizeNM);
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

            public Builder goalBias(double goalBias) {
                config.setGoalBias(goalBias);
                return this;
            }

            public Builder useRRTStar(boolean useRRTStar) {
                config.setUseRRTStar(useRRTStar);
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

            public Builder searchSpacePaddingDeg(double searchSpacePaddingDeg) {
                config.setSearchSpacePaddingDeg(searchSpacePaddingDeg);
                return this;
            }

            public Builder searchSpaceAltPaddingFt(double searchSpaceAltPaddingFt) {
                config.setSearchSpaceAltPaddingFt(searchSpaceAltPaddingFt);
                return this;
            }

            public Builder speedKnots(double speedKnots) {
                config.setSpeedKnots(speedKnots);
                return this;
            }

            public RRTConfig build() {
                return config;
            }
        }
    }
