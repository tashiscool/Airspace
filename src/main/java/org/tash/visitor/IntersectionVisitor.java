package org.tash.visitor;

import org.tash.core.SpatialElement;
import org.tash.core.TemporalElement;
import org.tash.event.conflict.ClosestPointOfApproach;
import org.tash.spatial.*;
import org.tash.trajectory.TrajectorySegment;

import org.tash.data.GeoCoordinate;

import java.time.ZonedDateTime;
import java.util.List;

import static edu.uci.ics.jung.algorithms.layout.PolarPoint.polarToCartesian;
import static org.tash.AirspaceModel.*;

/**
     * Visitor that checks for intersections between spatial elements
     */
    public class IntersectionVisitor extends AbstractSpatialVisitor {
        private SpatialElement source;
        private boolean intersecting;
        
        public IntersectionVisitor(SpatialElement source) {
            this.source = source;
            this.intersecting = false;
        }
        
        public boolean isIntersecting() {
            return intersecting;
        }
        
        @Override
        public void visit(SpatialPoint point) {
            if (source instanceof SpatialPoint) {
                // Point-Point intersection
                SpatialPoint sourcePoint = (SpatialPoint) source;
                intersecting = point.getCoordinate().equals(sourcePoint.getCoordinate());
            } else if (source instanceof SpatialLine) {
                // Point-Line intersection
                SpatialLine line = (SpatialLine) source;
                intersecting = line.containsPoint(point.getCoordinate());
            } else if (source instanceof SpatialPolygon) {
                // Point-Polygon intersection
                SpatialPolygon polygon = (SpatialPolygon) source;
                intersecting = polygon.containsPoint(point.getCoordinate());
            } else if (source instanceof SpatialVolume) {
                // Point-Volume intersection
                SpatialVolume volume = (SpatialVolume) source;
                intersecting = volume.containsPoint(point.getCoordinate());
            }
        }
        
        @Override
        public void visit(SpatialLine line) {
            if (source instanceof SpatialPoint) {
                // Line-Point intersection (commutative)
                visit((SpatialPoint) source);
            } else if (source instanceof SpatialLine) {
                // Line-Line intersection
                SpatialLine sourceLine = (SpatialLine) source;
                intersecting = checkLineLineIntersection(sourceLine, line);
            } else if (source instanceof SpatialPolygon) {
                // Line-Polygon intersection
                SpatialPolygon polygon = (SpatialPolygon) source;
                for (SpatialLine edge : polygon.getEdges()) {
                    if (checkLineLineIntersection(edge, line)) {
                        intersecting = true;
                        return;
                    }
                }
                
                // Also check if the line is contained in the polygon
                SpatialPoint midpoint = line.getPointAtFraction(0.5);
                intersecting = polygon.containsPoint(midpoint.getCoordinate());
            } else if (source instanceof SpatialVolume) {
                // Line-Volume intersection
                SpatialVolume volume = (SpatialVolume) source;
                // Check for intersection with the base polygon
                SpatialPolygon base = volume.getBasePolygon();
                for (SpatialLine edge : base.getEdges()) {
                    if (checkLineLineIntersection(edge, line)) {
                        intersecting = true;
                        return;
                    }
                }
                
                // Also check if the line is contained in the volume
                SpatialPoint midpoint = line.getPointAtFraction(0.5);
                intersecting = volume.containsPoint(midpoint.getCoordinate());
            }
        }
        
        @Override
        public void visit(SpatialPolygon polygon) {
            if (source instanceof SpatialPoint) {
                // Polygon-Point intersection (commutative)
                visit((SpatialPoint) source);
            } else if (source instanceof SpatialLine) {
                // Polygon-Line intersection (commutative)
                visit((SpatialLine) source);
            } else if (source instanceof SpatialPolygon) {
                // Polygon-Polygon intersection
                SpatialPolygon sourcePolygon = (SpatialPolygon) source;
                
                // Check for edge intersections
                for (SpatialLine edge1 : sourcePolygon.getEdges()) {
                    for (SpatialLine edge2 : polygon.getEdges()) {
                        if (checkLineLineIntersection(edge1, edge2)) {
                            intersecting = true;
                            return;
                        }
                    }
                }
                
                // Check if one polygon contains a point from the other
                SpatialPoint samplePoint = sourcePolygon.getSamplePoint();
                if (polygon.containsPoint(samplePoint.getCoordinate())) {
                    intersecting = true;
                    return;
                }
                
                samplePoint = polygon.getSamplePoint();
                if (sourcePolygon.containsPoint(samplePoint.getCoordinate())) {
                    intersecting = true;
                }
            } else if (source instanceof SpatialVolume) {
                // Polygon-Volume intersection
                SpatialVolume volume = (SpatialVolume) source;
                
                // Check if polygon intersects with the base polygon of the volume
                SpatialPolygon sourcePolygon = volume.getBasePolygon();
                
                // First check edges
                for (SpatialLine edge1 : sourcePolygon.getEdges()) {
                    for (SpatialLine edge2 : polygon.getEdges()) {
                        if (checkLineLineIntersection(edge1, edge2)) {
                            intersecting = true;
                            return;
                        }
                    }
                }
                
                // Check if one polygon contains a point from the other
                SpatialPoint samplePoint = sourcePolygon.getSamplePoint();
                if (polygon.containsPoint(samplePoint.getCoordinate())) {
                    intersecting = true;
                    return;
                }
                
                samplePoint = polygon.getSamplePoint();
                if (sourcePolygon.containsPoint(samplePoint.getCoordinate())) {
                    intersecting = true;
                }
            }
        }
        
        @Override
        public void visit(SpatialVolume volume) {
            if (source instanceof SpatialPoint) {
                // Volume-Point intersection (commutative)
                visit((SpatialPoint) source);
            } else if (source instanceof SpatialLine) {
                // Volume-Line intersection (commutative)
                visit((SpatialLine) source);
            } else if (source instanceof SpatialPolygon) {
                // Volume-Polygon intersection (commutative)
                visit((SpatialPolygon) source);
            } else if (source instanceof SpatialVolume) {
                // Volume-Volume intersection
                SpatialVolume sourceVolume = (SpatialVolume) source;
                
                // First check if the base polygons intersect
                SpatialPolygon base1 = sourceVolume.getBasePolygon();
                SpatialPolygon base2 = volume.getBasePolygon();
                
                // Use the polygon-polygon intersection check
                IntersectionVisitor polygonVisitor = new IntersectionVisitor(base1);
                base2.accept(polygonVisitor);
                
                if (polygonVisitor.isIntersecting()) {
                    // Check if the height ranges overlap
                    if (sourceVolume.getLowerAltitude() <= volume.getUpperAltitude() &&
                        sourceVolume.getUpperAltitude() >= volume.getLowerAltitude()) {
                        intersecting = true;
                    }
                }
            }
        }
        
        @Override
        public void visit(TrajectorySegment segment) {
            // Trajectory segments need to be checked against other elements
            if (source instanceof SpatialPoint) {
                // Segment-Point intersection
                SpatialPoint point = (SpatialPoint) source;
                
                // Convert the segment to a spatial line
                SpatialLine line = segment.toSpatialLine();
                intersecting = line.containsPoint(point.getCoordinate());
            } else if (source instanceof SpatialLine) {
                // Segment-Line intersection
                SpatialLine line = (SpatialLine) source;
                SpatialLine segmentLine = segment.toSpatialLine();
                
                intersecting = checkLineLineIntersection(line, segmentLine);
            } else if (source instanceof SpatialPolygon) {
                // Segment-Polygon intersection
                SpatialPolygon polygon = (SpatialPolygon) source;
                SpatialLine segmentLine = segment.toSpatialLine();
                
                // Check if segment intersects with any edge
                for (SpatialLine edge : polygon.getEdges()) {
                    if (checkLineLineIntersection(edge, segmentLine)) {
                        intersecting = true;
                        return;
                    }
                }
                
                // Check if segment is contained in the polygon
                SpatialPoint midpoint = segmentLine.getPointAtFraction(0.5);
                intersecting = polygon.containsPoint(midpoint.getCoordinate());
            } else if (source instanceof SpatialVolume) {
                // Segment-Volume intersection
                SpatialVolume volume = (SpatialVolume) source;
                
                // First check temporal overlap for efficiency
                if (source instanceof TemporalElement && segment instanceof TemporalElement) {
                    TemporalElement sourceTemp = (TemporalElement) source;
                    TemporalElement segmentTemp = (TemporalElement) segment;
                    
                    if (!sourceTemp.timeOverlaps(segmentTemp)) {
                        intersecting = false;
                        return;
                    }
                }
                
                // Check a few sample points along the segment
                int numSamples = 10;
                for (int i = 0; i <= numSamples; i++) {
                    double fraction = (double) i / numSamples;
                    SpatialPoint point = segment.getPointAtFraction(fraction);
                    
                    if (volume.containsPoint(point.getCoordinate())) {
                        intersecting = true;
                        return;
                    }
                }
                
                intersecting = false;
            } else if (source instanceof TrajectorySegment) {
                // Segment-Segment intersection
                TrajectorySegment sourceSegment = (TrajectorySegment) source;
                
                // First check temporal overlap for efficiency
                if (source instanceof TemporalElement && segment instanceof TemporalElement) {
                    TemporalElement sourceTemp = (TemporalElement) source;
                    TemporalElement segmentTemp = (TemporalElement) segment;
                    
                    if (!sourceTemp.timeOverlaps(segmentTemp)) {
                        intersecting = false;
                        return;
                    }
                }
                
                // Use specialized trajectory-trajectory intersection check
                ClosestPointOfApproach cpa = calculateClosestPointOfApproach(sourceSegment, segment);
                
                // Check if horizontal separation is violated
                if (cpa.getHorizontalDistance() < MIN_HORIZONTAL_SEPARATION_NM) {
                    // Check if vertical separation is also violated
                    if (Math.abs(cpa.getVerticalDistance()) < MIN_VERTICAL_SEPARATION_FT) {
                        intersecting = true;
                        return;
                    }
                }
                
                intersecting = false;
            }
        }

    @Override
    public void visit(SpatialCircle circle) {
        if (source instanceof SpatialPoint) {
            // Circle-Point intersection
            SpatialPoint point = (SpatialPoint) source;
            intersecting = circle.containsPoint(point.getCoordinate());
        } else if (source instanceof SpatialLine) {
            // Circle-Line intersection (simplified)
            SpatialLine line = (SpatialLine) source;

            // Check if either endpoint is inside the circle
            if (circle.containsPoint(line.getStartPoint().getCoordinate()) ||
                    circle.containsPoint(line.getEndPoint().getCoordinate())) {
                intersecting = true;
                return;
            }

            // Check if line intersects the circle
            // Use a simplified algorithm: distance from line to center < radius
            GeoCoordinate p1 = line.getStartPoint().getCoordinate();
            GeoCoordinate p2 = line.getEndPoint().getCoordinate();
            GeoCoordinate c = circle.getCenter().getCoordinate();

            // Convert to a local Cartesian reference frame for simplicity
            // Use the circle center as the origin
            Vector2D v1 = polarToCartesian(c.distanceTo(p1),
                    angleBetween(c, p1));
            Vector2D v2 = polarToCartesian(c.distanceTo(p2),
                    angleBetween(c, p2));

            // Calculate distance from line to origin (circle center)
            // Line equation: ax + by + c = 0
            double a = v2.getY() - v1.getY();
            double b = v1.getX() - v2.getX();
            double c2 = v2.getX() * v1.getY() - v1.getX() * v2.getY();

            double distanceToLine = Math.abs(c2) / Math.sqrt(a*a + b*b);

            intersecting = distanceToLine <= circle.getRadius();
        } else if (source instanceof SpatialCircle) {
            // Circle-Circle intersection
            SpatialCircle sourceCircle = (SpatialCircle) source;

            double centerDistance = sourceCircle.getCenter().getCoordinate()
                    .distanceTo(circle.getCenter().getCoordinate());
            double sumRadii = sourceCircle.getRadius() + circle.getRadius();

            intersecting = centerDistance <= sumRadii;
        } else if (source instanceof SpatialArc) {
            // Circle-Arc intersection (simplified)
            SpatialArc arc = (SpatialArc) source;

            // Treat the arc as a part of a circle
            SpatialCircle arcCircle = SpatialCircle.builder()
                    .id(arc.getId() + "-circle")
                    .center(arc.getCenter())
                    .radius(arc.getRadius())
                    .build();

            // Check if the circles intersect
            double centerDistance = arcCircle.getCenter().getCoordinate()
                    .distanceTo(circle.getCenter().getCoordinate());
            double sumRadii = arcCircle.getRadius() + circle.getRadius();

            if (centerDistance > sumRadii) {
                intersecting = false;
                return;
            }

            // Find intersection points of the circles
            // Then check if any intersection point falls within the arc
            // For simplicity, check a few sample points along the arc
            List<SpatialPoint> samplePoints = arc.getSamplePoints(12);

            for (SpatialPoint point : samplePoints) {
                if (circle.containsPoint(point.getCoordinate())) {
                    intersecting = true;
                    return;
                }
            }

            // Check if any sample points along the circle fall within the arc
            for (int i = 0; i < 16; i++) {
                double angle = 360.0 * i / 16;
                SpatialPoint circlePoint = circle.getPointAtAngle(angle);

                double distToArcCenter = circlePoint.getCoordinate()
                        .distanceTo(arc.getCenter().getCoordinate());

                // If point is on arc's circle and within arc's angle
                if (Math.abs(distToArcCenter - arc.getRadius()) < 0.01 &&
                        arc.containsAngle(angle)) {
                    intersecting = true;
                    return;
                }
            }

            intersecting = false;
        }
        else if (source instanceof SpatialPolygon) {
            // Circle-Polygon intersection
            SpatialPolygon polygon = (SpatialPolygon) source;

            // Check if any polygon edge intersects the circle
            for (SpatialLine edge : polygon.getEdges()) {
                if (checkLineCircleIntersection(edge, circle)) {
                    intersecting = true;
                    return;
                }
            }

            // Check if any polygon vertex is inside the circle
            for (SpatialPoint vertex : polygon.getVertices()) {
                if (circle.containsPoint(vertex.getCoordinate())) {
                    intersecting = true;
                    return;
                }
            }

            intersecting = false;
        }
        else if (source instanceof SpatialVolume) {
            // Circle-Volume intersection
            SpatialVolume volume = (SpatialVolume) source;

            // Check if the circle intersects the base polygon of the volume
            SpatialPolygon base = volume.getBasePolygon();

            // First check edges
            for (SpatialLine edge : base.getEdges()) {
                if (checkLineCircleIntersection(edge, circle)) {
                    intersecting = true;
                    return;
                }
            }

            // Check if any vertex is inside the circle
            for (SpatialPoint vertex : base.getVertices()) {
                if (circle.containsPoint(vertex.getCoordinate())) {
                    intersecting = true;
                    return;
                }
            }

            intersecting = false;
        }
        else {
            // Handle other spatial types
            throw new IllegalArgumentException("Unsupported spatial type: " + source.getClass().getName());
        }
    }

    private boolean checkLineCircleIntersection(SpatialLine edge, SpatialCircle circle) {
        // Check if either endpoint is inside the circle
        if (circle.containsPoint(edge.getStartPoint().getCoordinate()) ||
                circle.containsPoint(edge.getEndPoint().getCoordinate())) {
            return true;
        }

        // Check if line intersects the circle
        // Use a simplified algorithm: distance from line to center < radius
        GeoCoordinate p1 = edge.getStartPoint().getCoordinate();
        GeoCoordinate p2 = edge.getEndPoint().getCoordinate();
        GeoCoordinate c = circle.getCenter().getCoordinate();

        // Convert to a local Cartesian reference frame for simplicity
        // Use the circle center as the origin
        Vector2D v1 = polarToCartesian(c.distanceTo(p1),
                angleBetween(c, p1));
        Vector2D v2 = polarToCartesian(c.distanceTo(p2),
                angleBetween(c, p2));

        // Calculate distance from line to origin (circle center)
        // Line equation: ax + by + c = 0
        double a = v2.getY() - v1.getY();
        double b = v1.getX() - v2.getX();
        double c2 = v2.getX() * v1.getY() - v1.getX() * v2.getY();

        double distanceToLine = Math.abs(c2) / Math.sqrt(a*a + b*b);

        return distanceToLine <= circle.getRadius();
    }

    @Override
    public void visit(SpatialArc arc) {
        if (source instanceof SpatialPoint) {
            // Arc-Point intersection
            SpatialPoint point = (SpatialPoint) source;
            intersecting = arc.containsPoint(point.getCoordinate());
        } else if (source instanceof SpatialCircle) {
            // Arc-Circle intersection (commutative)
            visit((SpatialCircle) source);
        } else if (source instanceof SpatialArc) {
            // Arc-Arc intersection (simplified)
            SpatialArc sourceArc = (SpatialArc) source;

            // First check if the underlying circles intersect
            double centerDistance = sourceArc.getCenter().getCoordinate()
                    .distanceTo(arc.getCenter().getCoordinate());
            double sumRadii = sourceArc.getRadius() + arc.getRadius();

            if (centerDistance > sumRadii) {
                intersecting = false;
                return;
            }

            // Check if any point on one arc lies on the other
            List<SpatialPoint> arcSamplePoints = arc.getSamplePoints(12);
            List<SpatialPoint> sourceArcSamplePoints = sourceArc.getSamplePoints(12);

            for (SpatialPoint point : arcSamplePoints) {
                if (sourceArc.containsPoint(point.getCoordinate())) {
                    intersecting = true;
                    return;
                }
            }

            for (SpatialPoint point : sourceArcSamplePoints) {
                if (arc.containsPoint(point.getCoordinate())) {
                    intersecting = true;
                    return;
                }
            }

            intersecting = false;
        }
        // Add more cases for other element types as needed
    }

    /**
     * Helper class for 2D Cartesian coordinates
     */
    private static class Vector2D {
        private double x;
        private double y;

        public Vector2D(double x, double y) {
            this.x = x;
            this.y = y;
        }

        public double getX() { return x; }
        public double getY() { return y; }
    }

    /**
     * Convert polar coordinates to Cartesian
     */
    private Vector2D polarToCartesian(double radius, double angleRad) {
        return new Vector2D(
                radius * Math.cos(angleRad),
                radius * Math.sin(angleRad)
        );
    }

    /**
     * Calculate angle between two geographic coordinates
     */
    private double angleBetween(GeoCoordinate center, GeoCoordinate point) {
        double dLon = point.getLongitude() - center.getLongitude();
        double dLat = point.getLatitude() - center.getLatitude();

        // Adjust for Earth's curvature
        dLon *= Math.cos(Math.toRadians(center.getLatitude()));
        double angleRad = Math.atan2(dLat, dLon);
        return angleRad;
    }

    private ClosestPointOfApproach calculateClosestPointOfApproach(TrajectorySegment sourceSegment, TrajectorySegment segment) {
        // Initialize variables to store the closest point of approach
        double minHorizontalDistance = Double.MAX_VALUE;
        double minVerticalDistance = Double.MAX_VALUE;
        ZonedDateTime closestTime = null;

        // Sample points along the segments to find the closest approach
        int numSamples = 100;
        for (int i = 0; i <= numSamples; i++) {
            double fraction = (double) i / numSamples;
            GeoCoordinate sourcePoint = sourceSegment.getPointAtFraction(fraction).getCoordinate();
            GeoCoordinate segmentPoint = segment.getPointAtFraction(fraction).getCoordinate();

            // Calculate horizontal and vertical distances
            double horizontalDistance = sourcePoint.distanceTo(segmentPoint);
            double verticalDistance = Math.abs(sourcePoint.getAltitude() - segmentPoint.getAltitude());

            // Update the closest point of approach if this point is closer
            if (horizontalDistance < minHorizontalDistance) {
                minHorizontalDistance = horizontalDistance;
                minVerticalDistance = verticalDistance;
                SpatialPoint spatialPoint = sourceSegment.getPointAtFraction(fraction);
//                closestTime = spatialPoint.getTime();

                // Check if the segments are close enough to intersect
                if (minHorizontalDistance < MIN_HORIZONTAL_SEPARATION_NM) {
                    if (minVerticalDistance < MIN_VERTICAL_SEPARATION_FT) {
                        break;
                    }
                }

                // Check if the segments are close enough to intersect
                if (minHorizontalDistance < MIN_HORIZONTAL_SEPARATION_NM && minVerticalDistance < MIN_VERTICAL_SEPARATION_FT) {
                    break;
                }
            }
        }

        // Create and return the ClosestPointOfApproach object
        return ClosestPointOfApproach.builder()
                .time(closestTime)
                .horizontalDistance(minHorizontalDistance)
                .verticalDistance(minVerticalDistance)
                .build();
    }

    /**
         * Check if two lines intersect
         */
        private boolean checkLineLineIntersection(SpatialLine line1, SpatialLine line2) {
            // Implementation of line segment intersection algorithm
            GeoCoordinate p1 = line1.getStartPoint().getCoordinate();
            GeoCoordinate p2 = line1.getEndPoint().getCoordinate();
            GeoCoordinate p3 = line2.getStartPoint().getCoordinate();
            GeoCoordinate p4 = line2.getEndPoint().getCoordinate();
            
            // Convert to simple 2D coordinates
            double x1 = p1.getLongitude(), y1 = p1.getLatitude();
            double x2 = p2.getLongitude(), y2 = p2.getLatitude();
            double x3 = p3.getLongitude(), y3 = p3.getLatitude();
            double x4 = p4.getLongitude(), y4 = p4.getLatitude();
            
            // Calculate determinants
            double det = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
            if (Math.abs(det) < 1e-10) {
                // Lines are parallel, check if they overlap
                // This is a simplification - a more robust check would be needed for colinear segments
                return false;
            }
            
            double detA = (x1 * y2 - y1 * x2);
            double detB = (x3 * y4 - y3 * x4);
            
            double intersectX = (detA * (x3 - x4) - (x1 - x2) * detB) / det;
            double intersectY = (detA * (y3 - y4) - (y1 - y2) * detB) / det;
            
            // Check if the intersection point is within both line segments
            boolean withinSegment1 = (
                (Math.min(x1, x2) <= intersectX && intersectX <= Math.max(x1, x2)) &&
                (Math.min(y1, y2) <= intersectY && intersectY <= Math.max(y1, y2))
            );
            
            boolean withinSegment2 = (
                (Math.min(x3, x4) <= intersectX && intersectX <= Math.max(x3, x4)) &&
                (Math.min(y3, y4) <= intersectY && intersectY <= Math.max(y3, y4))
            );
            
            return withinSegment1 && withinSegment2;
        }
    }