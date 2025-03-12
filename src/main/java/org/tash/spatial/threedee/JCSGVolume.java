package org.tash.spatial.threedee;

import eu.mihosoft.jcsg.*;
import eu.mihosoft.vvecmath.StoredVector3dImpl;
import eu.mihosoft.vvecmath.Transform;
import eu.mihosoft.vvecmath.Vector3d;
import lombok.Data;
import org.tash.core.SpatialElement;
import org.tash.data.BoundingBox;
import org.tash.data.GeoCoordinate;
import org.tash.spatial.SpatialPoint;
import org.tash.spatial.SpatialPolygon;
import org.tash.spatial.SpatialVolume;

import java.util.ArrayList;
import java.util.List;

/**
 * Wrapper class for CSG to enable
 * boolean operations on airspace volumes
 */
@Data
public class JCSGVolume {
    private CSG csg;
    private SpatialElement originalVolume;

    /**
     * Create from a ConicalVolume
     */
    public JCSGVolume(ConicalVolume volume) {
        this.originalVolume = volume;

        // Get apex coordinates
        GeoCoordinate apexCoord = volume.getApex().getCoordinate();

        // Create a cone with eu.mihosoft.jcsg
        double height = volume.getBaseAltitude() - apexCoord.getAltitude();
        height = height / 328.084; // Convert feet to 100 meters for better precision

        // Base radius in 100 meters
        double radius = volume.getBaseRadius() * 1.852; // Convert NM to km, then to 100 meters

        // Create cylinder for the cone
        Cylinder cylinder = new Cylinder(
                radius,
            height,
            32 // resolution
        );

        this.csg = cylinder.toCSG();

        // If the cone is tilted, apply a transformation
        if (volume.getSlopeAngle() > 0) {
            double slopeRad = Math.toRadians(volume.getSlopeAngle());
            double bearingRad = Math.toRadians(volume.getCentralAxisBearing());

            // Rotate around X-axis (tilt up) then around Z-axis (bearing)
            Transform transform = new Transform();

            // Rotation according to bearing (converts from geographic bearing to math angle)
            double rotationZ = 90 - volume.getCentralAxisBearing();
//            transform.rotate(new StoredVector3dImpl().add(0, 0, 1), Math.toRadians(rotationZ));
            transform.rot(0, 0, rotationZ); // Rotate around Z-axis

            // Tilt by slope angle
            transform.rot(1, 0, slopeRad); // Rotate around X-axis
//            transform.rotate(new StoredVector3dImpl().add(1, 0, 0), slopeRad);

            this.csg = this.csg.transformed(transform);
        }
    }

    /**
     * Create from a FrustumVolume
     */
    public JCSGVolume(FrustumVolume volume) {
        this.originalVolume = volume;

        // Create a cylinder with eu.mihosoft.jcsg
        double height = Math.abs(volume.getTopAltitude() - volume.getBaseAltitude());
        height = height / 328.084; // Convert feet to 100 meters for better precision

        // Base and top radius in 100 meters
        double baseRadius = volume.getBaseRadius() * 1.852; // Convert NM to km, then to 100 meters
        double topRadius = volume.getTopRadius() * 1.852;
        double radius  = (baseRadius + topRadius) / 2;
        // Create a cylinder (frustum)
        Cylinder cylinder = new Cylinder(
            radius,
                height,
            32 // resolution
        );

        this.csg = cylinder.toCSG();

        // Apply a transformation for bearing alignment
        Transform transform = new Transform();

        // Rotation according to bearing (converts from geographic bearing to math angle)
        double rotationZ = 90 - volume.getCentralAxisBearing();
        transform.rot(0, 0, rotationZ); // Rotate around Z-axis

        this.csg = this.csg.transformed(transform);
    }

    /**
     * Create from a SpatialVolume (polygonal volume)
     */
    public JCSGVolume(SpatialVolume volume) {
        this.originalVolume = volume;

        // Get the base polygon
        SpatialPolygon basePolygon = volume.getBasePolygon();
        List<SpatialPoint> vertices = basePolygon.getVertices();

        if (vertices.size() < 3) {
            throw new IllegalArgumentException("Polygon must have at least 3 vertices");
        }

        // Convert to JCSGVolume
        List<Polygon> polygons = new ArrayList<>();

        // Calculate height
        double height = volume.getUpperAltitude() - volume.getLowerAltitude();
        height = height / 328.084; // Convert feet to 100 meters for better precision

        // Create the base points in 2D
        List<Vector3d> basePoints = new ArrayList<>();

        // Reference point for local coordinate system
        GeoCoordinate refPoint = vertices.get(0).getCoordinate();

        for (SpatialPoint vertex : vertices) {
            GeoCoordinate coord = vertex.getCoordinate();

            // Calculate local X,Y coordinates (simplified flat Earth approximation)
            double latDiff = coord.getLatitude() - refPoint.getLatitude();
            double lonDiff = coord.getLongitude() - refPoint.getLongitude();

            // Convert to kilometers (1 degree latitude ≈ 111 km)
            double y = latDiff * 111.0;

            // Longitude degrees to kilometers (depends on latitude)
            double x = lonDiff * 111.0 * Math.cos(Math.toRadians(refPoint.getLatitude()));

            // Add to points list
            basePoints.add(new StoredVector3dImpl().add(x, y, 0));
        }

        // Create an extruded polygon
        this.csg = Extrude.points(
            new StoredVector3dImpl().add(0, 0, height),
            basePoints
        );
    }

    /**
     * Create from a CylindricalVolume
     */
    public JCSGVolume(CylindricalVolume volume) {
        this.originalVolume = volume;

        // Create a cylinder with eu.mihosoft.jcsg
        double height = volume.getUpperAltitude() - volume.getLowerAltitude();
        height = height / 328.084; // Convert feet to 100 meters for better precision

        // Radius in 100 meters
        double radius = volume.getBase().getRadius() * 1.852; // Convert NM to km, then to 100 meters

        // Create a cylinder
        Cylinder cylinder = new Cylinder(
            radius,
            height,
            32 // resolution
        );

        this.csg = cylinder.toCSG();
    }

    /**
     * Create from a JCSGVolume (copy constructor)
     */
    public JCSGVolume(JCSGVolume other) {
        this.csg = other.csg.clone();
        this.originalVolume = other.originalVolume;
    }

    /**
     * Create from a raw CSG object
     */
    public JCSGVolume(CSG csg, SpatialElement originalVolume) {
        this.csg = csg;
        this.originalVolume = originalVolume;
    }

    /**
     * Perform union operation with another volume
     *
     * @param other The other volume
     * @return A new volume representing the union
     */
    public JCSGVolume union(JCSGVolume other) {
        CSG result = this.csg.union(other.csg);
        return new JCSGVolume(result, null);
    }

    /**
     * Perform intersection operation with another volume
     *
     * @param other The other volume
     * @return A new volume representing the intersection
     */
    public JCSGVolume intersection(JCSGVolume other) {
        CSG result = this.csg.intersect(other.csg);
        return new JCSGVolume(result, null);
    }

    /**
     * Perform subtraction operation (this - other)
     *
     * @param other The volume to subtract
     * @return A new volume representing the difference
     */
    public JCSGVolume subtract(JCSGVolume other) {
        CSG result = this.csg.difference(other.csg);
        return new JCSGVolume(result, null);
    }

    /**
     * Check if a point is inside this volume
     *
     * @param point The point to check
     * @return True if the point is inside the volume
     */
    public boolean containsPoint(GeoCoordinate point) {
        // Convert to local coordinates
        Vector3d localPoint = convertToLocalCoordinates(point);
        // Convert to CSG coordinates
        localPoint = new StoredVector3dImpl().add(localPoint.x(), localPoint.y(), localPoint.z());

        // Use CSG to check if point is inside
        List<CSG> localPoint2 = new ArrayList<>();
        localPoint2.add(new Cube(0.1).toCSG().transformed(Transform.unity().translate(localPoint)));
        return csg.intersect(localPoint2) != null;
    }

    /**
     * Convert a geographic coordinate to local CSG coordinates
     */
    private Vector3d convertToLocalCoordinates(GeoCoordinate point) {
        // This would require knowing the transformation applied to create the CSG
        // For a proper implementation, you'd need to store the transformation parameters

        // Simplified implementation (assumes volume is centered at origin)
        GeoCoordinate reference;

        if (originalVolume instanceof ConicalVolume) {
            reference = ((ConicalVolume) originalVolume).getApex().getCoordinate();
        } else if (originalVolume instanceof FrustumVolume) {
            reference = ((FrustumVolume) originalVolume).getBaseCenter().getCoordinate();
        } else if (originalVolume instanceof CylindricalVolume) {
            reference = ((CylindricalVolume) originalVolume).getBase().getCenter().getCoordinate();
        } else if (originalVolume instanceof SpatialVolume) {
            // Use first vertex of base polygon as reference
            reference = ((SpatialVolume) originalVolume).getBasePolygon().getVertices().get(0).getCoordinate();
        } else {
            throw new IllegalStateException("Unknown volume type");
        }

        // Calculate local X,Y coordinates (simplified flat Earth approximation)
        double latDiff = point.getLatitude() - reference.getLatitude();
        double lonDiff = point.getLongitude() - reference.getLongitude();

        // Convert to kilometers (1 degree latitude ≈ 111 km)
        double y = latDiff * 111.0;

        // Longitude degrees to kilometers (depends on latitude)
        double x = lonDiff * 111.0 * Math.cos(Math.toRadians(reference.getLatitude()));

        // Altitude difference in 100 meters
        double z = (point.getAltitude() - reference.getAltitude()) / 328.084;

        return new StoredVector3dImpl().add(x, y, z);
    }

    /**
     * Export to STL format for visualization
     *
     * @param filePath Path to save the STL file
     */
    public void exportToStl(String filePath) {
        try {

        } catch (Exception e) {
            throw new RuntimeException("Failed to export to STL: " + e.getMessage(), e);
        }
    }

    /**
     * Get the bounding box of this CSG volume
     *
     * @return Bounding box
     */
    public BoundingBox getBoundingBox() {
        Bounds bounds = csg.getBounds();

        return BoundingBox.builder()
            .minLat(bounds.getMin().y() / 111.0) // Convert km to degrees
            .maxLat(bounds.getMax().y() / 111.0)
            .minLon(bounds.getMin().x() / (111.0 * Math.cos(Math.toRadians(0)))) // Rough approximation
            .maxLon(bounds.getMax().x() / (111.0 * Math.cos(Math.toRadians(0))))
            .minAlt(bounds.getMin().z() * 328.084) // Convert 100m to feet
            .maxAlt(bounds.getMax().z() * 328.084)
            .build();
    }

    public SpatialPoint getPointAtFraction(double v) {
        throw new UnsupportedOperationException("Operation not supported for a CSG volume.");
    }
}
