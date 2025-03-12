package org.tash.spatial.index;

import org.tash.core.SpatialElement;
import org.tash.data.BoundingBox;

import java.util.ArrayList;
import java.util.List;

import static org.tash.spatial.index.QuadtreeSpatialIndex.MAX_DEPTH;
import static org.tash.spatial.index.QuadtreeSpatialIndex.MAX_ELEMENTS;

/**
         * Quadtree node
         */
        class QuadNode {
            private BoundingBox bbox;
            private int depth;
            private List<SpatialElement> elements;
            private QuadNode[] children;
            
            public QuadNode(BoundingBox bbox, int depth) {
                this.bbox = bbox;
                this.depth = depth;
                this.elements = new ArrayList<>();
                this.children = null; // Lazy initialization
            }
            
            /**
             * Add an element to this node
             */
            public void add(SpatialElement element) {
                // Check if element's bounding box intersects with this node
                if (!bbox.intersects(element.getBoundingBox())) {
                    return;
                }
                
                // If we have children, add to appropriate children
                if (children != null) {
                    for (QuadNode child : children) {
                        child.add(element);
                    }
                    return;
                }
                
                // Add to this node
                elements.add(element);
                
                // Check if we need to split
                if (elements.size() > MAX_ELEMENTS && depth < MAX_DEPTH) {
                    split();
                }
            }
            
            /**
             * Split this node into four children
             */
            private void split() {
                double midLat = (bbox.getMinLat() + bbox.getMaxLat()) / 2;
                double midLon = (bbox.getMinLon() + bbox.getMaxLon()) / 2;
                
                children = new QuadNode[4];
                
                // Create four quadrants
                children[0] = new QuadNode(
                    BoundingBox.builder()
                        .minLat(midLat)
                        .maxLat(bbox.getMaxLat())
                        .minLon(bbox.getMinLon())
                        .maxLon(midLon)
                        .minAlt(bbox.getMinAlt())
                        .maxAlt(bbox.getMaxAlt())
                        .build(),
                    depth + 1
                );
                
                children[1] = new QuadNode(
                    BoundingBox.builder()
                        .minLat(midLat)
                        .maxLat(bbox.getMaxLat())
                        .minLon(midLon)
                        .maxLon(bbox.getMaxLon())
                        .minAlt(bbox.getMinAlt())
                        .maxAlt(bbox.getMaxAlt())
                        .build(),
                    depth + 1
                );
                
                children[2] = new QuadNode(
                    BoundingBox.builder()
                        .minLat(bbox.getMinLat())
                        .maxLat(midLat)
                        .minLon(bbox.getMinLon())
                        .maxLon(midLon)
                        .minAlt(bbox.getMinAlt())
                        .maxAlt(bbox.getMaxAlt())
                        .build(),
                    depth + 1
                );
                
                children[3] = new QuadNode(
                    BoundingBox.builder()
                        .minLat(bbox.getMinLat())
                        .maxLat(midLat)
                        .minLon(midLon)
                        .maxLon(bbox.getMaxLon())
                        .minAlt(bbox.getMinAlt())
                        .maxAlt(bbox.getMaxAlt())
                        .build(),
                    depth + 1
                );
                
                // Redistribute elements to children
                for (SpatialElement element : elements) {
                    for (QuadNode child : children) {
                        child.add(element);
                    }
                }
                
                // Clear this node's elements
                elements.clear();
            }
            
            /**
             * Remove an element from this node
             */
            public void remove(SpatialElement element) {
                // Check if element's bounding box intersects with this node
                if (!bbox.intersects(element.getBoundingBox())) {
                    return;
                }
                
                // If we have children, remove from appropriate children
                if (children != null) {
                    for (QuadNode child : children) {
                        child.remove(element);
                    }
                    
                    // Check if we should merge
                    boolean shouldMerge = true;
                    int totalElements = 0;
                    for (QuadNode child : children) {
                        if (child.children != null) {
                            shouldMerge = false;
                            break;
                        }
                        totalElements += child.elements.size();
                    }
                    
                    if (shouldMerge && totalElements <= MAX_ELEMENTS) {
                        merge();
                    }
                    
                    return;
                }
                
                // Remove from this node
                elements.removeIf(e -> e.getId().equals(element.getId()));
            }
            
            /**
             * Merge children back into this node
             */
            private void merge() {
                // Collect all elements from children
                for (QuadNode child : children) {
                    elements.addAll(child.elements);
                }
                
                // Remove children
                children = null;
            }
            
            /**
             * Query elements within a bounding box
             */
            public void query(BoundingBox queryBBox, List<SpatialElement> results) {
                // Check if queryBBox intersects with this node
                if (!bbox.intersects(queryBBox)) {
                    return;
                }
                
                // If we have children, query appropriate children
                if (children != null) {
                    for (QuadNode child : children) {
                        child.query(queryBBox, results);
                    }
                    return;
                }
                
                // Add elements that intersect with the query bbox
                for (SpatialElement element : elements) {
                    if (element.getBoundingBox().intersects(queryBBox)) {
                        results.add(element);
                    }
                }
            }
            
            /**
             * Clear this node
             */
            public void clear() {
                elements.clear();
                if (children != null) {
                    for (QuadNode child : children) {
                        child.clear();
                    }
                    children = null;
                }
            }
        }