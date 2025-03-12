package org.tash.visitor;

import org.tash.core.AirspaceElement;
import org.tash.core.SpatialVisitor;
import org.tash.spatial.*;
import org.tash.trajectory.TrajectorySegment;

/**
     * Example visitor that finds an element with a specific ID substring
     */
    public class FinderVisitor implements SpatialVisitor {
        private String idSubstring;
        private AirspaceElement foundElement = null;
        
        public FinderVisitor(String idSubstring) {
            this.idSubstring = idSubstring;
        }
        
        @Override
        public void visit(AirspaceElement element) {
            if (element.getId().contains(idSubstring)) {
                foundElement = element;
            }
        }
        
        @Override
        public void visit(SpatialPoint point) {
            visit((AirspaceElement) point);
        }
        
        @Override
        public void visit(SpatialLine line) {
            visit((AirspaceElement) line);
        }
        
        @Override
        public void visit(SpatialPolygon polygon) {
            visit((AirspaceElement) polygon);
        }
        
        @Override
        public void visit(SpatialVolume volume) {
            visit((AirspaceElement) volume);
        }

    @Override
    public void visit(SpatialCircle circle) {
            // Not relevant to finding
    }

    @Override
    public void visit(SpatialArc arc) {
            // Not relevant to finding
    }

    @Override
        public void visit(TrajectorySegment segment) {
            visit((AirspaceElement) segment);
        }
        
        public AirspaceElement getFoundElement() {
            return foundElement;
        }
    }