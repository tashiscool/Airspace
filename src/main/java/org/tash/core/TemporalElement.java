package org.tash.core;

import java.time.ZonedDateTime;

/**
     * Interface for elements with a time dimension
     */
    public interface TemporalElement extends AirspaceElement {
        /**
         * Get the start time of this element
         */
        ZonedDateTime getStartTime();
        
        /**
         * Get the end time of this element
         */
        ZonedDateTime getEndTime();
        
        /**
         * Check if this element's time window overlaps with another
         */
        default boolean timeOverlaps(TemporalElement other) {
            return !(getEndTime().isBefore(other.getStartTime()) || 
                   other.getEndTime().isBefore(getStartTime()));
        }
        
        /**
         * Check if this element's time window contains a specific time
         */
        default boolean containsTime(ZonedDateTime time) {
            return !time.isBefore(getStartTime()) && !time.isAfter(getEndTime());
        }
    }