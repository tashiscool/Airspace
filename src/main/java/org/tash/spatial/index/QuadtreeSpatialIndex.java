package org.tash.spatial.index;

import org.tash.core.SpatialElement;
import org.tash.data.BoundingBox;

import java.util.ArrayList;
import java.util.List;

/**
     * Simple spatial index implementation using a quadtree
     */
    public class QuadtreeSpatialIndex implements SpatialIndex {
        private QuadNode root;
        public static final int MAX_ELEMENTS = 10;
        public static final int MAX_DEPTH = 10;
        
        public QuadtreeSpatialIndex() {
            // Create a root node covering the entire globe
            BoundingBox globalBBox = BoundingBox.builder()
                .minLat(-90)
                .maxLat(90)
                .minLon(-180)
                .maxLon(180)
                .minAlt(-1000)
                .maxAlt(100000)
                .build();
            
            root = new QuadNode(globalBBox, 0);
        }
        
        @Override
        public void add(SpatialElement element) {
            root.add(element);
        }
        
        @Override
        public void remove(SpatialElement element) {
            root.remove(element);
        }
        
        @Override
        public List<SpatialElement> query(BoundingBox bbox) {
            List<SpatialElement> results = new ArrayList<>();
            root.query(bbox, results);
            return results;
        }
        
        @Override
        public List<SpatialElement> query(SpatialElement element) {
            return query(element.getBoundingBox());
        }
        
        @Override
        public void clear() {
            root.clear();
        }

        public List<SpatialElement> findElementsWithin(BoundingBox bbox) {
            return query(bbox);
        }
    }