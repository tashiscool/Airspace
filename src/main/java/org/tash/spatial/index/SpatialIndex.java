package org.tash.spatial.index;

import org.tash.core.SpatialElement;
import org.tash.data.BoundingBox;

import java.util.List;

/**
     * Interface for spatial indexes
     */
    public interface SpatialIndex {
        /**
         * Add a spatial element to the index
         */
        void add(SpatialElement element);
        
        /**
         * Remove a spatial element from the index
         */
        void remove(SpatialElement element);
        
        /**
         * Query elements within a bounding box
         */
        List<SpatialElement> query(BoundingBox bbox);
        
        /**
         * Query elements that intersect with a spatial element
         */
        List<SpatialElement> query(SpatialElement element);
        
        /**
         * Clear the index
         */
        void clear();
    }