package org.tash.core;

import org.tash.data.BoundingBox;
import org.tash.spatial.SpatialPoint;
import org.tash.visitor.IntersectionVisitor;

/**
     * Interface for elements that exist in space
     */
    public interface SpatialElement extends AirspaceElement, Visitable {
        /**
         * Accept a spatial visitor
         */
        void accept(SpatialVisitor visitor);
        
        /**
         * Get the bounding box
         */
        BoundingBox getBoundingBox();
        
        /**
         * Check if this element intersects with another
         */
        default boolean intersects(SpatialElement other) {
            IntersectionVisitor visitor = new IntersectionVisitor(this);
            other.accept(visitor);
            return visitor.isIntersecting();
        }

        /**
         * Check if this element contains a point
         */
        default boolean contains(double lat, double lon, double alt) {
            return getBoundingBox().contains(lat, lon, alt);
        }

    SpatialPoint getPointAtFraction(double v);
}