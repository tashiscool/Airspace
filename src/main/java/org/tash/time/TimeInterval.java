package org.tash.time;

import lombok.*;

import java.time.ZonedDateTime;

@Data
    @Builder
    @NoArgsConstructor
    @AllArgsConstructor
    public class TimeInterval {
        private ZonedDateTime startTime;
        private ZonedDateTime endTime;
        
        /**
         * Check if this interval contains a time
         */
        public boolean contains(ZonedDateTime time) {
            return !time.isBefore(startTime) && !time.isAfter(endTime);
        }
        
        /**
         * Check if this interval overlaps with another
         */
        public boolean overlaps(TimeInterval other) {
            return !(endTime.isBefore(other.startTime) || 
                   startTime.isAfter(other.endTime));
        }
        
        /**
         * Get the duration of this interval
         */
        public java.time.Duration getDuration() {
            return java.time.Duration.between(startTime, endTime);
        }
    }