package org.tash.spatial.threedee;

import lombok.*;
import org.tash.core.SpatialElement;

/**
     * Represents a single operation in the construction of the compound volume
     */
    @Data
    @Builder
    @NoArgsConstructor
    @AllArgsConstructor
    public class VolumeOperation {
        public OperationType operation;
        public SpatialElement volume;
    }