package org.tash.visitor;

import org.tash.core.AirspaceElement;
import org.tash.core.SpatialVisitor;
import org.tash.spatial.*;
import org.tash.trajectory.TrajectorySegment;

/**
     * Example visitor that counts elements by type
     */
    public class CountingVisitor implements SpatialVisitor {
        private int pointCount = 0;
        private int lineCount = 0;
        private int polygonCount = 0;
        private int volumeCount = 0;
        private int trajectoryCount = 0;
        
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
            }
        }
        
        @Override
        public void visit(SpatialPoint point) {
            pointCount++;
        }
        
        @Override
        public void visit(SpatialLine line) {
            lineCount++;
        }
        
        @Override
        public void visit(SpatialPolygon polygon) {
            polygonCount++;
        }
        
        @Override
        public void visit(SpatialVolume volume) {
            volumeCount++;
        }

    @Override
    public void visit(SpatialCircle circle) {
            // Not relevant to counting
    }

    @Override
    public void visit(SpatialArc arc) {
            // Not relevant to counting
    }

    @Override
        public void visit(TrajectorySegment segment) {
            trajectoryCount++;
        }
        
        public int getPointCount() { return pointCount; }
        public int getLineCount() { return lineCount; }
        public int getPolygonCount() { return polygonCount; }
        public int getVolumeCount() { return volumeCount; }
        public int getTrajectoryCount() { return trajectoryCount; }
    }
    
   