package org.tash.event;


import lombok.*;
import org.tash.trajectory.TrajectorySegment;

import java.time.ZonedDateTime;

/**
     * Event when trajectories conflict
     */
    @Data
    @EqualsAndHashCode(callSuper = true)
    public class TrajectoryConflictEvent extends AirspaceEvent {
        private TrajectorySegment segment1;
        private TrajectorySegment segment2;
        private double horizontalSeparation;
        private double verticalSeparation;
        
        public TrajectoryConflictEvent(TrajectorySegment segment1, TrajectorySegment segment2,
                                       ZonedDateTime timestamp, double horizontalSeparation,
                                       double verticalSeparation) {
            super("TRAJECTORY_CONFLICT", timestamp);
            this.segment1 = segment1;
            this.segment2 = segment2;
            this.horizontalSeparation = horizontalSeparation;
            this.verticalSeparation = verticalSeparation;
        }
    }