package org.tash.spatial.threedee;

import lombok.*;
import org.tash.core.*;
import org.tash.data.BoundingBox;
import org.tash.data.GeoCoordinate;
import org.tash.spatial.SpatialLine;
import org.tash.spatial.SpatialPoint;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

/**
 * Represents a frustum (truncated cone) airspace volume
 * Useful for modeling approach corridors that widen with distance
 */
@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
@EqualsAndHashCode(onlyExplicitlyIncluded = true)
public class FrustumVolume implements SpatialElement, TemporalElement {
    @EqualsAndHashCode.Include
    private String id;
    private SpatialPoint baseCenter;    // Center of the smaller base
    private double baseRadius;          // Radius of the smaller base in nautical miles
    private double baseAltitude;        // Altitude of the smaller base in feet
    private double topRadius;           // Radius of the larger base in nautical miles
    private double topAltitude;         // Altitude of the larger base in feet
    private double centralAxisBearing;  // Bearing of the central axis in degrees
    private double length;              // Length of the frustum in nautical miles
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
        // Calculate top base center
        GeoCoordinate topCenter = baseCenter.getCoordinate().destinationPoint(length, centralAxisBearing);
        topCenter.setAltitude(topAltitude);

        // Find the outer bounds by calculating coordinates at maximum extent
        double maxRadius = Math.max(baseRadius, topRadius);

        // Calculate lat/lon deltas
        double baseLat = baseCenter.getCoordinate().getLatitude();
        double baseLatDelta = baseRadius / 60.0; // 1 minute of latitude = 1 NM
        double baseLonDelta = baseRadius / (60.0 * Math.cos(Math.toRadians(baseLat)));

        double topLat = topCenter.getLatitude();
        double topLatDelta = topRadius / 60.0;
        double topLonDelta = topRadius / (60.0 * Math.cos(Math.toRadians(topLat)));

        // Calculate combined bounds
        double minLat = Math.min(baseLat - baseLatDelta, topLat - topLatDelta);
        double maxLat = Math.max(baseLat + baseLatDelta, topLat + topLatDelta);
        double minLon = Math.min(baseCenter.getCoordinate().getLongitude() - baseLonDelta,
                topCenter.getLongitude() - topLonDelta);
        double maxLon = Math.max(baseCenter.getCoordinate().getLongitude() + baseLonDelta,
                topCenter.getLongitude() + topLonDelta);
        double minAlt = Math.min(baseAltitude, topAltitude);
        double maxAlt = Math.max(baseAltitude, topAltitude);

        return BoundingBox.builder()
                .minLat(minLat)
                .maxLat(maxLat)
                .minLon(minLon)
                .maxLon(maxLon)
                .minAlt(minAlt)
                .maxAlt(maxAlt)
                .startTime(startTime)
                .endTime(endTime)
                .build();
    }

    /**
     * Get the center of the top base
     */
    public GeoCoordinate getTopCenter() {
        GeoCoordinate topCenter = baseCenter.getCoordinate().destinationPoint(length, centralAxisBearing);
        topCenter.setAltitude(topAltitude);
        return topCenter;
    }

    /**
     * Check if a point is inside this frustum volume
     */
    public boolean containsPoint(GeoCoordinate point) {
        // Check altitude bounds
        double minAlt = Math.min(baseAltitude, topAltitude);
        double maxAlt = Math.max(baseAltitude, topAltitude);
        if (point.getAltitude() < minAlt || point.getAltitude() > maxAlt) {
            return false;
        }

        // Check time bounds
        if (startTime != null && endTime != null
         // TODO: Fix this point has the time?
        ) {
            return false;
        }

        // Find nearest point on central axis
        GeoCoordinate topCenter = getTopCenter();

        // Create a line segment from base to top
        SpatialLine centralAxis = SpatialLine.builder()
                .id("temp-axis")
                .startPoint(SpatialPoint.builder().id("temp-base").coordinate(baseCenter.getCoordinate()).build())
                .endPoint(SpatialPoint.builder().id("temp-top").coordinate(topCenter).build())
                .build();

        // Find closest point on the axis to the test point
        GeoCoordinate closestOnAxis = point.closestPointOnPath(baseCenter.getCoordinate(), topCenter);

        // Calculate distance from point to axis
        double distanceToAxis = point.distanceTo(closestOnAxis);

        // Calculate the fraction along the axis
        double axisLength = baseCenter.distanceTo(SpatialPoint.builder().coordinate(topCenter).build());
        double baseDistance = baseCenter.distanceTo(SpatialPoint.builder().coordinate(closestOnAxis).build());
        double fraction = axisLength > 0 ? baseDistance / axisLength : 0;

        // Calculate allowable radius at this position along the axis
        double allowableRadius = baseRadius + fraction * (topRadius - baseRadius);

        // Check if point is within the allowed distance
        return distanceToAxis <= allowableRadius;
    }

    /**
     * Generate a polygonal approximation of the frustum at a specific altitude
     *
     * @param altitude  Altitude in feet
     * @param numPoints Number of points around the perimeter
     * @return List of points forming a circle at the specified altitude
     */
    public List<SpatialPoint> getPerimeterAtAltitude(double altitude, int numPoints) {
        List<SpatialPoint> points = new ArrayList<>();

        // Check if altitude is within frustum bounds
        double minAlt = Math.min(baseAltitude, topAltitude);
        double maxAlt = Math.max(baseAltitude, topAltitude);
        if (altitude < minAlt || altitude > maxAlt) {
            return points;
        }

        // Calculate fraction of height
        double heightFraction = (altitude - baseAltitude) / (topAltitude - baseAltitude);

        // Calculate radius at this height
        double radius = baseRadius + heightFraction * (topRadius - baseRadius);

        // Calculate center point at this height
        GeoCoordinate center = baseCenter.getCoordinate().destinationPoint(heightFraction * length, centralAxisBearing);
        center.setAltitude(altitude);
        points.add(SpatialPoint.builder().id("temp-center").coordinate(center).build());
        center = center.destinationPoint(radius, centralAxisBearing + 180);
        center.setAltitude(altitude);


        // Generate points around the perimeter
        for (int i = 0; i < numPoints; i++) {
            double angle = 360.0 * i / numPoints;
            GeoCoordinate point = center.destinationPoint(radius, angle);
            point.setAltitude(altitude);
            points.add(SpatialPoint.builder().id("temp-point-" + i).coordinate(point).build());
        }
        return points;
    }

    @Override
    public boolean intersects(SpatialElement other) {
        return SpatialElement.super.intersects(other);
    }

    @Override
    public boolean contains(double lat, double lon, double alt) {
        return SpatialElement.super.contains(lat, lon, alt);
    }

    @Override
    public SpatialPoint getPointAtFraction(double v) {
        throw new UnsupportedOperationException("Operation not supported for a volume.");
    }
}