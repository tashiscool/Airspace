package org.tash.event.conflict;

import lombok.*;

import java.time.ZonedDateTime;

/**
     * Represents the closest point of approach between two trajectories
     */
    @Data
    @Builder
    @NoArgsConstructor
    @AllArgsConstructor
    public class ClosestPointOfApproach {
        private ZonedDateTime time;
        private double horizontalDistance; // nautical miles
        private double verticalDistance;   // feet
    }