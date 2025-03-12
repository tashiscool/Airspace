package org.tash.event.conflict;

import org.tash.trajectory.TrajectorySegment;

import java.util.List;

/**
     * Base interface for conflict detection strategies (Strategy pattern)
     */
    public interface ConflictDetectionStrategy {
        /**
         * Check for conflicts in a set of trajectory segments
         */
        List<SeparationConflict> detectConflicts(List<TrajectorySegment> segments);
        
        /**
         * Check for conflicts between two trajectory segments
         */
        SeparationConflict detectConflict(TrajectorySegment segment1, TrajectorySegment segment2);
    }