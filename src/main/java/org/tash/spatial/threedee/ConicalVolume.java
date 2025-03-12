package org.tash.spatial.threedee;

import lombok.AllArgsConstructor;
import lombok.Builder;
import lombok.Data;
import lombok.EqualsAndHashCode;
import lombok.NoArgsConstructor;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

import org.tash.core.ElementType;
import org.tash.core.SpatialElement;
import org.tash.core.SpatialVisitor;
import org.tash.core.TemporalElement;
import org.tash.core.Visitor;
import org.tash.data.BoundingBox;
import org.tash.data.GeoCoordinate;
import org.tash.spatial.SpatialPoint;
import org.tash.spatial.Triangle3D;

/**
 * Represents a conical airspace volume (e.g., for approach/departure paths)
 * The cone has its apex at the bottom and expands upward
 */
@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
@EqualsAndHashCode(onlyExplicitlyIncluded = true)
public class ConicalVolume implements SpatialElement, TemporalElement {
    @EqualsAndHashCode.Include
    private String id;
    private SpatialPoint apex;         // Apex of the cone (bottom point)
    private double baseAltitude;       // Altitude of the cone's base in feet
    private double baseRadius;         // Radius of the cone's base in nautical miles
    private double centralAxisBearing; // Bearing of the central axis in degrees
    private double slopeAngle;         // Slope angle from vertical in degrees (0-90)
    private ZonedDateTime startTime;
    private ZonedDateTime endTime;

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
        // Calculate cone height
        double apexAltitude = apex.getCoordinate().getAltitude();
        double coneHeight = baseAltitude - apexAltitude;

        // Calculate maximum horizontal distance from apex
        double maxDistance = baseRadius;

        // If the cone is tilted, adjust the bounding box
        if (slopeAngle > 0) {
            // Additional distance due to tilt
            double tiltOffset = coneHeight * Math.tan(Math.toRadians(slopeAngle));
            maxDistance += tiltOffset;
        }

        // Convert to lat/lon deltas
        double lat = apex.getCoordinate().getLatitude();
        double latDelta = maxDistance / 60.0; // 1 minute of latitude = 1 NM
        double lonDelta = maxDistance / (60.0 * Math.cos(Math.toRadians(lat)));

        // Calculate the center of the base
        double bearingRad = Math.toRadians(centralAxisBearing);
        double latOffset = 0;
        double lonOffset = 0;

        // If the cone is tilted, the base center is offset from directly above the apex
        if (slopeAngle > 0) {
            double offsetDistance = coneHeight * Math.tan(Math.toRadians(slopeAngle));
            latOffset = offsetDistance * Math.sin(bearingRad) / 60.0;
            lonOffset = offsetDistance * Math.cos(bearingRad) / (60.0 * Math.cos(Math.toRadians(lat)));
        }

        return BoundingBox.builder()
                .minLat(lat + latOffset - latDelta)
                .maxLat(lat + latOffset + latDelta)
                .minLon(apex.getCoordinate().getLongitude() + lonOffset - lonDelta)
                .maxLon(apex.getCoordinate().getLongitude() + lonOffset + lonDelta)
                .minAlt(apexAltitude)
                .maxAlt(baseAltitude)
                .startTime(startTime)
                .endTime(endTime)
                .build();
    }

    @Override
    public SpatialPoint getPointAtFraction(double v) {
        throw new UnsupportedOperationException("Operation not supported for a volume.");
    }

    /**
     * Check if a point is inside this conical volume
     */
    public boolean containsPoint(GeoCoordinate point) {
        // Check altitude bounds
        double apexAltitude = apex.getCoordinate().getAltitude();
        if (point.getAltitude() < apexAltitude || point.getAltitude() > baseAltitude) {
            return false;
        }

        // Check time bounds
        if (startTime != null && endTime != null
                // TODO: Fix this point has the time?
        ) {
            return false;
        }

        // Calculate horizontal distance from apex to point
        double distance = apex.getCoordinate().distanceTo(point);

        // Calculate bearing from apex to point
        double bearing = apex.getCoordinate().initialBearingTo(point);

        // Calculate angle between central axis and line to point
        double angleDiff = Math.abs(normalizeBearing(bearing - centralAxisBearing));
        if (angleDiff > 90) {
            // Point is behind the cone
            return false;
        }

        // Calculate height above apex
        double height = point.getAltitude() - apexAltitude;

        // Calculate the maximum allowed distance at this height
        double maxRadius = (height / (baseAltitude - apexAltitude)) * baseRadius;

        // If the cone is tilted, adjust the allowed distance based on the bearing difference
        if (slopeAngle > 0) {
            // Convert slopeAngle to radians
            double slopeRad = Math.toRadians(slopeAngle);

            // Additional offset due to tilt
            double tiltOffset = height * Math.tan(slopeRad) * Math.cos(Math.toRadians(angleDiff));

            // Adjust the center of the cone at this height
            double adjustedDistance = distance - tiltOffset;

            return adjustedDistance <= maxRadius;
        }

        // For vertical cone, simply check if point is within radius
        return distance <= maxRadius;
    }

    /**
     * Normalize bearing to 0-360 range
     */
    private double normalizeBearing(double bearing) {
        bearing = bearing % 360;
        if (bearing < 0) bearing += 360;
        return bearing;
    }

    /**
     * Generate a polygonal approximation of the cone at a specific altitude
     * @param altitude Altitude in feet
     * @param numPoints Number of points around the perimeter
     * @return List of points forming a circle at the specified altitude
     */
    public List<SpatialPoint> getPerimeterAtAltitude(double altitude, int numPoints) {
        List<SpatialPoint> points = new ArrayList<>();

        // Check if altitude is within cone bounds
        double apexAltitude = apex.getCoordinate().getAltitude();
        if (altitude < apexAltitude || altitude > baseAltitude) {
            return points;
        }

        // Calculate fraction of height
        double heightFraction = (altitude - apexAltitude) / (baseAltitude - apexAltitude);

        // Calculate radius at this height
        double radius = heightFraction * baseRadius;

        // Calculate center point at this height
        double length = (altitude - apexAltitude) / (baseAltitude - apexAltitude);
        double distanceAlongAxis = heightFraction * length;
        GeoCoordinate baseCenter = apex.getCoordinate().destinationPoint(distanceAlongAxis, centralAxisBearing);
        GeoCoordinate centerAtHeight = baseCenter.destinationPoint(distanceAlongAxis, centralAxisBearing);
        centerAtHeight.setAltitude(altitude);

        // Generate points around the perimeter
        for (int i = 0; i < numPoints; i++) {
            double angle = 360.0 * i / numPoints;
            double bearing = (centralAxisBearing + angle) % 360;

            GeoCoordinate pointCoord = centerAtHeight.destinationPoint(radius, bearing);
            pointCoord.setAltitude(altitude);

            points.add(SpatialPoint.builder()
                    .id(id + "-perim-" + altitude + "-" + i)
                    .coordinate(pointCoord)
                    .build());
        }

        return points;
    }

    /**
     * Convert to a mesh representation for visualization
     * @param verticalSamples Number of vertical samples
     * @param radialSamples Number of points around perimeter
     * @return List of triangles representing the frustum surface
     */
    public List<org.tash.spatial.Triangle3D> toMesh(int verticalSamples, int radialSamples) {
        List<org.tash.spatial.Triangle3D> triangles = new ArrayList<>();

        // Generate rings of points at different altitudes
        double length = baseAltitude - apex.getCoordinate().getAltitude();
        double topAltitude = baseAltitude + length;
        double altStep = (topAltitude - baseAltitude) / (verticalSamples - 1);

        // Generate rings of points
        List<List<SpatialPoint>> rings = new ArrayList<>();
        for (int v = 0; v < verticalSamples; v++) {
            double altitude = baseAltitude + v * altStep;
            rings.add(getPerimeterAtAltitude(altitude, radialSamples));
        }

        // Generate triangles connecting rings
        for (int v = 0; v < verticalSamples - 1; v++) {
            List<SpatialPoint> lowerRing = rings.get(v);
            List<SpatialPoint> upperRing = rings.get(v + 1);

            for (int i = 0; i < radialSamples; i++) {
                int nextI = (i + 1) % radialSamples;

                // Two triangles form a quad
                triangles.add(new org.tash.spatial.Triangle3D(
                        lowerRing.get(i).getCoordinate(),
                        upperRing.get(i).getCoordinate(),
                        lowerRing.get(nextI).getCoordinate()
                ));

                triangles.add(new Triangle3D(
                        upperRing.get(i).getCoordinate(),
                        upperRing.get(nextI).getCoordinate(),
                        lowerRing.get(nextI).getCoordinate()
                ));
            }
        }

        return triangles;
    }

    /**
     * Convert to JCSGVolume for boolean operations
     */
    public JCSGVolume toJCSGVolume() {
        return new JCSGVolume(this);
    }
}