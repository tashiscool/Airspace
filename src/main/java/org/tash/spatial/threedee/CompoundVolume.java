package org.tash.spatial.threedee;

import eu.mihosoft.jcsg.Polygon;
import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.EqualsAndHashCode;
import lombok.NoArgsConstructor;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

import org.tash.core.ElementType;
import org.tash.core.SpatialElement;
import org.tash.core.SpatialVisitor;
import org.tash.core.TemporalElement;
import org.tash.core.Visitor;
import org.tash.data.BoundingBox;
import org.tash.data.GeoCoordinate;
import eu.mihosoft.jcsg.CSG;
import org.tash.spatial.SpatialPoint;
import org.tash.spatial.SpatialVolume;

/**
 * Represents a compound airspace volume created from boolean operations
 * on other volumes
 */
@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
@EqualsAndHashCode(onlyExplicitlyIncluded = true)
public class CompoundVolume implements SpatialElement, TemporalElement {
    @EqualsAndHashCode.Include
    private String id;
    private List<VolumeOperation> operations;
    private ZonedDateTime startTime;
    private ZonedDateTime endTime;
    
    // Cached CSG representation (lazily initialized)
    private transient JCSGVolume cachedJcsgVolume;

    public CompoundVolume(OperationType operationType, SpatialElement[] volumes) {
        this.operations = new ArrayList<>();
        for (SpatialElement volume : volumes) {
            operations.add(VolumeOperation.builder()
                .operation(operationType)
                .volume(volume)
                .build());
        }
    }


    @Override
    public ElementType getElementType() {
        return ElementType.VOLUME;
    }
    
    @Override
    public void accept(Visitor visitor) {
        if (visitor instanceof SpatialVisitor) {
            ((SpatialVisitor) visitor).visit(this);
        } else {
            visitor.visit(this);
        }
    }
    
    @Override
    public void accept(SpatialVisitor visitor) {
        visitor.visit(this);
    }
    
    @Override
    public BoundingBox getBoundingBox() {
        if (operations == null || operations.isEmpty()) {
            throw new IllegalStateException("Compound volume has no operations");
        }
        
        // Get JCSGVolume representation
        JCSGVolume jcsgVolume = toJCSGVolume();
        
        // Get bounding box from JCSGVolume
        return jcsgVolume.getBoundingBox();
    }

    @Override
    public SpatialPoint getPointAtFraction(double v) {
        JCSGVolume jcsgVolume = toJCSGVolume();
        return jcsgVolume.getPointAtFraction(v);
    }

    /**
     * Convert to JCSGVolume representation
     * @return JCSGVolume representation
     */
    public JCSGVolume toJCSGVolume() {
        if (cachedJcsgVolume != null) {
            return cachedJcsgVolume;
        }
        
        if (operations == null || operations.isEmpty()) {
            throw new IllegalStateException("Compound volume has no operations");
        }
        
        // Start with the first volume
        VolumeOperation firstOp = operations.get(0);
        if (firstOp.operation != OperationType.BASE) {
            throw new IllegalStateException("First operation must be BASE");
        }
        
        // Convert first volume to JCSGVolume
        JCSGVolume result = convertToJCSGVolume(firstOp.volume);
        
        // Apply subsequent operations
        for (int i = 1; i < operations.size(); i++) {
            VolumeOperation op = operations.get(i);
            JCSGVolume nextVolume = convertToJCSGVolume(op.volume);
            
            switch (op.operation) {
                case UNION:
                    result = result.union(nextVolume);
                    break;
                case INTERSECT:
                    result = result.intersection(nextVolume);
                    break;
                case SUBTRACT:
                    result = result.subtract(nextVolume);
                    break;
                default:
                    throw new IllegalStateException("Invalid operation type: " + op.operation);
            }
        }
        
        // Cache the result
        cachedJcsgVolume = result;
        return result;
    }
    
    /**
     * Convert a spatial element to JCSGVolume
     */
    private JCSGVolume convertToJCSGVolume(SpatialElement element) {
        if (element instanceof ConicalVolume) {
            return new JCSGVolume((ConicalVolume) element);
        } else if (element instanceof FrustumVolume) {
            return new JCSGVolume((FrustumVolume) element);
        } else if (element instanceof CylindricalVolume) {
            return new JCSGVolume((CylindricalVolume) element);
        } else if (element instanceof SpatialVolume) {
            return new JCSGVolume((SpatialVolume) element);
        } else if (element instanceof CompoundVolume) {
            return ((CompoundVolume) element).toJCSGVolume();
        } else {
            throw new IllegalArgumentException("Unsupported volume type: " + element.getClass().getName());
        }
    }
    
    /**
     * Check if a point is inside this compound volume
     */
    public boolean containsPoint(GeoCoordinate point) {
        // Check time bounds
        if (startTime != null && endTime != null
//                &&
//            (point.getTime().isBefore(startTime) || point.getTime().isAfter(endTime))
        ) {
            return false;
        }
        
        // Use the JCSGVolume to check containment
        return toJCSGVolume().containsPoint(point);
    }
    
    /**
     * Build a compound volume with the builder pattern
     */
    public static class Builder {
        private String id;
        private List<VolumeOperation> operations = new ArrayList<>();
        private ZonedDateTime startTime;
        private ZonedDateTime endTime;
        
        /**
         * Set the ID of the compound volume
         */
        public Builder id(String id) {
            this.id = id;
            return this;
        }
        
        /**
         * Set the base volume
         */
        public Builder base(SpatialElement volume) {
            operations.add(VolumeOperation.builder()
                .operation(OperationType.BASE)
                .volume(volume)
                .build());
            return this;
        }
        
        /**
         * Add a volume with union operation
         */
        public Builder union(SpatialElement volume) {
            operations.add(VolumeOperation.builder()
                .operation(OperationType.UNION)
                .volume(volume)
                .build());
            return this;
        }
        
        /**
         * Add a volume with intersection operation
         */
        public Builder intersect(SpatialElement volume) {
            operations.add(VolumeOperation.builder()
                .operation(OperationType.INTERSECT)
                .volume(volume)
                .build());
            return this;
        }
        
        /**
         * Add a volume with subtraction operation
         */
        public Builder subtract(SpatialElement volume) {
            operations.add(VolumeOperation.builder()
                .operation(OperationType.SUBTRACT)
                .volume(volume)
                .build());
            return this;
        }
        
        /**
         * Set the start time
         */
        public Builder startTime(ZonedDateTime startTime) {
            this.startTime = startTime;
            return this;
        }
        
        /**
         * Set the end time
         */
        public Builder endTime(ZonedDateTime endTime) {
            this.endTime = endTime;
            return this;
        }
        
        /**
         * Build the compound volume
         */
        public CompoundVolume build() {
            if (operations.isEmpty()) {
                throw new IllegalStateException("Compound volume must have at least one operation");
            }
            
            if (operations.get(0).operation != OperationType.BASE) {
                throw new IllegalStateException("First operation must be BASE");
            }
            
            if (id == null) {
                id = "COMPOUND-" + System.currentTimeMillis();
            }
            
            return new CompoundVolume(id, operations, startTime, endTime, null);
        }
    }
    
    /**
     * Create a new builder
     */
    public static Builder builder() {
        return new Builder();
    }
    
    /**
     * Export to STL file for visualization
     * 
     * @param filePath Path to save the STL file
     */
    public void exportToStl(String filePath) {
        toJCSGVolume().exportToStl(filePath);
    }
    
    /**
     * Calculate the approximate volume in cubic nautical miles
     * 
     * @return Volume in cubic nautical miles
     */
    public double calculateVolume() {
        // Get the CSG volume in cubic meters
        double volumeCubicMeters = toJCSGVolume().getCsg().getPolygons()
            .stream()
            .mapToDouble(polygon -> getArea(polygon) * polygon.getPlane().getNormal().dot(polygon.vertices.get(0).pos))
            .sum();
        
        // Convert to cubic nautical miles (1 NM = 1852 meters)
        return volumeCubicMeters / Math.pow(1852, 3);
    }

    private double getArea(Polygon polygon) {
        // Calculate the area of a polygon using the shoelace formula
        double area = 0.0;
        for (int i = 0; i < polygon.vertices.size(); i++) {
            eu.mihosoft.jcsg.Vertex v1 = polygon.vertices.get(i);
            eu.mihosoft.jcsg.Vertex v2 = polygon.vertices.get((i + 1) % polygon.vertices.size());
            area += v1.pos.x() * v2.pos.y() - v2.pos.x() * v1.pos.y();
        }
        return Math.abs(area) / 2.0;
    }

    /**
     * Get a visualization mesh representation
     * 
     * @return List of triangles representing the mesh
     */
    public List<Triangle3D> toMesh() {
        CSG csg = toJCSGVolume().getCsg();
        
        // Convert CSG polygons to triangles
        return csg.getPolygons().stream()
            .flatMap(polygon -> {
                List<Triangle3D> triangles = new ArrayList<>();
                eu.mihosoft.jcsg.Vertex firstVertex = polygon.vertices.get(0);
                
                // Triangulate the polygon
                for (int i = 1; i < polygon.vertices.size() - 1; i++) {
                    eu.mihosoft.jcsg.Vertex v1 = firstVertex;
                    eu.mihosoft.jcsg.Vertex v2 = polygon.vertices.get(i);
                    eu.mihosoft.jcsg.Vertex v3 = polygon.vertices.get(i + 1);
                    
                    // Create a triangle
                    triangles.add(new Triangle3D(
                        convertToGeoCoordinate(v1),
                        convertToGeoCoordinate(v2),
                        convertToGeoCoordinate(v3)
                    ));
                }
                
                return triangles.stream();
            })
            .collect(Collectors.toList());
    }
    
    /**
     * Convert a CSG vertex to GeoCoordinate
     * This is an approximation and would need proper inverse transformation
     */
    private GeoCoordinate convertToGeoCoordinate(eu.mihosoft.jcsg.Vertex vertex) {
        // This would need to apply the inverse of the transformation used to create the CSG
        // For a proper implementation, you'd need to store the transformation parameters
        
        // For now, use a very simplified conversion (assumes flat Earth and arbitrary positioning)
        return GeoCoordinate.builder()
            .latitude(vertex.pos.y() / 111.0)  // Convert from km to degrees
            .longitude(vertex.pos.x() / 111.0) // Simplified conversion
            .altitude(vertex.pos.z() * 328.084) // Convert 100m to feet
            .build();
    }
    
    /**
     * Helper class for 3D triangles
     */
    @Data
    @AllArgsConstructor
    public static class Triangle3D {
        private GeoCoordinate v1;
        private GeoCoordinate v2;
        private GeoCoordinate v3;
    }
}

