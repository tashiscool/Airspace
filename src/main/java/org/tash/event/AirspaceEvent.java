package org.tash.event;

import lombok.*;
import lombok.experimental.SuperBuilder;

import java.time.ZonedDateTime;

/**
     * Base class for airspace events
     */
    @Data
    @AllArgsConstructor
    @NoArgsConstructor
    @SuperBuilder
    public abstract class AirspaceEvent {
        private String eventType;
        private ZonedDateTime timestamp;
    }