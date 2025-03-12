package org.tash.time;

import org.tash.data.GeoCoordinate;

import java.time.ZonedDateTime;

/**
     * Interface for motion models
     */
    public interface MotionModel {
        /**
         * Get position at a specific time
         */
        GeoCoordinate getPositionAt(ZonedDateTime time);
        
        /**
         * Get the time bounds for this motion model
         */
        TimeInterval getTimeInterval();
    }
