package org.tash.event.conflict;

import lombok.*;
import org.tash.trajectory.TrajectorySegment;

import java.time.ZonedDateTime;

/**
     * Represents a separation conflict between two trajectory segments
     */
    @Data
    @Builder
    @NoArgsConstructor
    @AllArgsConstructor
    public class SeparationConflict {
        private TrajectorySegment segment1;
        private TrajectorySegment segment2;
        private ZonedDateTime time;
        private double horizontalSeparation; // nautical miles
        private double verticalSeparation;   // feet
    }
