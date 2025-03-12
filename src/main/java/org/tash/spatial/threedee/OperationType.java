package org.tash.spatial.threedee;

/**
     * Operation type for boolean operations
     */
    public enum OperationType {
        BASE,       // First volume (no operation)
        UNION,      // Add volumes
        INTERSECT,  // Keep only common volume
        SUBTRACT    // Remove second volume from first
    }