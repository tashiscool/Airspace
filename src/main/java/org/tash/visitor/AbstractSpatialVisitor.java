package org.tash.visitor;

import org.tash.core.AirspaceElement;
import org.tash.core.SpatialVisitor;
import org.tash.spatial.*;
import org.tash.trajectory.TrajectorySegment;

/**
     * Abstract base visitor for spatial elements
     */
    public abstract class AbstractSpatialVisitor implements SpatialVisitor {
        @Override
        public void visit(AirspaceElement element) {
            if (element instanceof SpatialPoint) {
                visit((SpatialPoint) element);
            } else if (element instanceof SpatialLine) {
                visit((SpatialLine) element);
            } else if (element instanceof SpatialPolygon) {
                visit((SpatialPolygon) element);
            } else if (element instanceof SpatialVolume) {
                visit((SpatialVolume) element);
            } else if (element instanceof TrajectorySegment) {
                visit((TrajectorySegment) element);
            } else if (element instanceof SpatialCircle) {
                visit((SpatialCircle) element);
            } else if (element instanceof SpatialArc) {
                visit((SpatialArc) element);
            }
        }
    }