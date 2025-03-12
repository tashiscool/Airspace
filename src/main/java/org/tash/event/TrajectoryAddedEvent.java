package org.tash.event;

import lombok.*;
import org.tash.flight.FlightTrajectory;

import java.time.ZonedDateTime;

/**
     * Event when a new trajectory is added
     */
    @Data
    @EqualsAndHashCode(callSuper = true)
    public class TrajectoryAddedEvent extends AirspaceEvent {
        private FlightTrajectory trajectory;
        
        public TrajectoryAddedEvent(FlightTrajectory trajectory, ZonedDateTime timestamp) {
            super("TRAJECTORY_ADDED", timestamp);
            this.trajectory = trajectory;
        }
    }