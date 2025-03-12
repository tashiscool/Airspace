package org.tash.core;

import org.tash.spatial.*;
import org.tash.trajectory.TrajectorySegment;

/**
     * Visitor specifically for spatial elements
     */
    public interface SpatialVisitor extends Visitor {
        void visit(SpatialPoint point);
        void visit(SpatialLine line);
        void visit(SpatialPolygon polygon);
        void visit(TrajectorySegment segment);
        void visit(SpatialVolume volume);
        void visit(SpatialCircle circle);
        void visit(SpatialArc arc);
    }