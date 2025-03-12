package org.tash.event;

import lombok.*;
import lombok.experimental.SuperBuilder;
import org.tash.spatial.SpatialVolume;
import org.tash.trajectory.TrajectorySegment;

import java.time.ZonedDateTime;

/**
     * Event when a trajectory enters reserved airspace
     */
    @SuperBuilder
    @Data
    @EqualsAndHashCode(callSuper = true)
    public class AirspaceInfringementEvent extends AirspaceEvent {
        private TrajectorySegment segment;
        private SpatialVolume volume;
        
        public AirspaceInfringementEvent(TrajectorySegment segment, SpatialVolume volume, 
                                       ZonedDateTime timestamp) {
            super("AIRSPACE_INFRINGEMENT", timestamp);
            this.segment = segment;
            this.volume = volume;
        }
    }