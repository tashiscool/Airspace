package org.tash.extensions.geodetic;

import org.tash.data.GeoCoordinate;
import org.tash.extensions.geodetic.EarthModel;
import org.tash.spatial.SpatialLine;
import org.tash.spatial.SpatialPoint;
import org.tash.spatial.SpatialPolygon;
import org.tash.spatial.SpatialVolume;

import java.util.ArrayList;
import java.util.List;

/**
 * Provides accurate geodetic calculations based on the WGS84 Earth model
 */
public class GeodeticCalculator {

    /**
     * Calculate the great circle distance between two points using Vincenty's formula
     * This is much more accurate than simpler formulas, especially for long distances
     *
     * @param point1 First point
     * @param point2 Second point
     * @return Distance in nautical miles
     */
    public static double vincentyDistance(GeoCoordinate point1, GeoCoordinate point2) {
        // Convert to radians
        double lat1 = Math.toRadians(point1.getLatitude());
        double lon1 = Math.toRadians(point1.getLongitude());
        double lat2 = Math.toRadians(point2.getLatitude());
        double lon2 = Math.toRadians(point2.getLongitude());

        // WGS84 ellipsoid parameters
        double a = EarthModel.WGS84_SEMI_MAJOR_AXIS;
        double b = EarthModel.WGS84_SEMI_MINOR_AXIS;
        double f = EarthModel.WGS84_FLATTENING;

        // Calculate L (difference in longitude)
        double L = lon2 - lon1;

        // Calculate reduced latitudes (U)
        double tanU1 = (1 - f) * Math.tan(lat1);
        double cosU1 = 1 / Math.sqrt(1 + tanU1 * tanU1);
        double sinU1 = tanU1 * cosU1;

        double tanU2 = (1 - f) * Math.tan(lat2);
        double cosU2 = 1 / Math.sqrt(1 + tanU2 * tanU2);
        double sinU2 = tanU2 * cosU2;

        // Iterative calculation
        double lambda = L;
        double lambdaP;
        double sinLambda, cosLambda;
        double sinSigma, cosSigma, sigma, sinAlpha, cos2Alpha, cos2SigmaM;

        int iterations = 0;
        do {
            sinLambda = Math.sin(lambda);
            cosLambda = Math.cos(lambda);

            sinSigma = Math.sqrt(
                    (cosU2 * sinLambda) * (cosU2 * sinLambda) +
                            (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda) * (cosU1 * sinU2 - sinU1 * cosU2 * cosLambda)
            );

            if (sinSigma == 0) {
                // Coincident points
                return 0;
            }

            cosSigma = sinU1 * sinU2 + cosU1 * cosU2 * cosLambda;
            sigma = Math.atan2(sinSigma, cosSigma);
            sinAlpha = cosU1 * cosU2 * sinLambda / sinSigma;
            cos2Alpha = 1 - sinAlpha * sinAlpha;

            cos2SigmaM = cosSigma - 2 * sinU1 * sinU2 / cos2Alpha;

            if (Double.isNaN(cos2SigmaM)) {
                // Equatorial line
                cos2SigmaM = 0;
            }

            double C = f / 16 * cos2Alpha * (4 + f * (4 - 3 * cos2Alpha));
            lambdaP = lambda;
            lambda = L + (1 - C) * f * sinAlpha * (sigma + C * sinSigma *
                    (cos2SigmaM + C * cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM)));

        } while (Math.abs(lambda - lambdaP) > 1e-12 && ++iterations < 100);

        if (iterations >= 100) {
            // Formula failed to converge
            // Fall back to haversine formula
            return haversineDistance(point1, point2);
        }

        // Calculate ellipsoidal distance
        double uSq = cos2Alpha * (a * a - b * b) / (b * b);
        double A = 1 + uSq / 16384 * (4096 + uSq * (-768 + uSq * (320 - 175 * uSq)));
        double B = uSq / 1024 * (256 + uSq * (-128 + uSq * (74 - 47 * uSq)));

        double deltaSigma = B * sinSigma * (cos2SigmaM + B / 4 * (cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM) -
                B / 6 * cos2SigmaM * (-3 + 4 * sinSigma * sinSigma) * (-3 + 4 * cos2SigmaM * cos2SigmaM)));

        double distanceMeters = b * A * (sigma - deltaSigma);

        // Convert to nautical miles
        return distanceMeters / EarthModel.NAUTICAL_MILE_IN_METERS;
    }

    /**
     * Calculate the Haversine distance (great circle distance) between two points
     * Simpler and faster than Vincenty, but less accurate especially for long distances
     *
     * @param point1 First point
     * @param point2 Second point
     * @return Distance in nautical miles
     */
    public static double haversineDistance(GeoCoordinate point1, GeoCoordinate point2) {
        double lat1 = Math.toRadians(point1.getLatitude());
        double lon1 = Math.toRadians(point1.getLongitude());
        double lat2 = Math.toRadians(point2.getLatitude());
        double lon2 = Math.toRadians(point2.getLongitude());

        double dLat = lat2 - lat1;
        double dLon = lon2 - lon1;

        double a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
                Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));

        return EarthModel.MEAN_EARTH_RADIUS_NM * c;
    }

    /**
     * Calculate the initial bearing from point1 to point2 on a great circle path
     *
     * @param point1 Starting point
     * @param point2 Destination point
     * @return Initial bearing in degrees (0-360)
     */
    public static double initialBearing(GeoCoordinate point1, GeoCoordinate point2) {
        double lat1 = Math.toRadians(point1.getLatitude());
        double lat2 = Math.toRadians(point2.getLatitude());
        double dLon = Math.toRadians(point2.getLongitude() - point1.getLongitude());

        double y = Math.sin(dLon) * Math.cos(lat2);
        double x = Math.cos(lat1) * Math.sin(lat2) - Math.sin(lat1) * Math.cos(lat2) * Math.cos(dLon);

        double bearing = Math.atan2(y, x);

        // Convert to degrees
        bearing = Math.toDegrees(bearing);

        // Normalize to 0-360
        return (bearing + 360) % 360;
    }

    /**
     * Calculate the final bearing arriving at point2 from point1 on a great circle path
     *
     * @param point1 Starting point
     * @param point2 Destination point
     * @return Final bearing in degrees (0-360)
     */
    public static double finalBearing(GeoCoordinate point1, GeoCoordinate point2) {
        // The final bearing is the initial bearing in the reverse direction plus 180°
        double bearing = initialBearing(point2, point1);
        return (bearing + 180) % 360;
    }

    /**
     * Calculate the halfway point along a great circle path between two points
     *
     * @param point1 First point
     * @param point2 Second point
     * @return The midpoint
     */
    public static GeoCoordinate midpoint(GeoCoordinate point1, GeoCoordinate point2) {
        double lat1 = Math.toRadians(point1.getLatitude());
        double lon1 = Math.toRadians(point1.getLongitude());
        double lat2 = Math.toRadians(point2.getLatitude());
        double lon2 = Math.toRadians(point2.getLongitude());

        double dLon = lon2 - lon1;

        double Bx = Math.cos(lat2) * Math.cos(dLon);
        double By = Math.cos(lat2) * Math.sin(dLon);

        double lat3 = Math.atan2(
                Math.sin(lat1) + Math.sin(lat2),
                Math.sqrt((Math.cos(lat1) + Bx) * (Math.cos(lat1) + Bx) + By * By)
        );
        double lon3 = lon1 + Math.atan2(By, Math.cos(lat1) + Bx);

        return GeoCoordinate.builder()
                .latitude(Math.toDegrees(lat3))
                .longitude(Math.toDegrees(lon3))
                .altitude((point1.getAltitude() + point2.getAltitude()) / 2)
                .build();
    }

    /**
     * Find a point at a given distance and bearing from a starting point
     *
     * @param start Starting point
     * @param distanceNM Distance in nautical miles
     * @param bearingDeg Bearing in degrees
     * @return The destination point
     */
    public static GeoCoordinate destinationPoint(GeoCoordinate start, double distanceNM, double bearingDeg) {
        double distanceMeters = distanceNM * EarthModel.NAUTICAL_MILE_IN_METERS;
        double bearingRad = Math.toRadians(bearingDeg);

        double lat1 = Math.toRadians(start.getLatitude());
        double lon1 = Math.toRadians(start.getLongitude());

        // Using WGS84
        double a = EarthModel.WGS84_SEMI_MAJOR_AXIS;
        double b = EarthModel.WGS84_SEMI_MINOR_AXIS;
        double f = EarthModel.WGS84_FLATTENING;

        double sinAlpha1 = Math.sin(bearingRad);
        double cosAlpha1 = Math.cos(bearingRad);

        double tanU1 = (1 - f) * Math.tan(lat1);
        double cosU1 = 1 / Math.sqrt(1 + tanU1 * tanU1);
        double sinU1 = tanU1 * cosU1;

        double sigma1 = Math.atan2(tanU1, cosAlpha1);
        double sinAlpha = cosU1 * sinAlpha1;
        double cos2Alpha = 1 - sinAlpha * sinAlpha;
        double uSq = cos2Alpha * (a * a - b * b) / (b * b);

        double A = 1 + uSq / 16384 * (4096 + uSq * (-768 + uSq * (320 - 175 * uSq)));
        double B = uSq / 1024 * (256 + uSq * (-128 + uSq * (74 - 47 * uSq)));

        double sigma = distanceMeters / (b * A);
        double sigmaP, iterations = 0;

        double sinSigma, cosSigma, cos2SigmaM;

        do {
            cos2SigmaM = Math.cos(2 * sigma1 + sigma);
            sinSigma = Math.sin(sigma);
            cosSigma = Math.cos(sigma);

            double deltaSigma = B * sinSigma * (cos2SigmaM + B / 4 * (cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM) -
                    B / 6 * cos2SigmaM * (-3 + 4 * sinSigma * sinSigma) *
                            (-3 + 4 * cos2SigmaM * cos2SigmaM)));

            sigmaP = sigma;
            sigma = distanceMeters / (b * A) + deltaSigma;

        } while (Math.abs(sigma - sigmaP) > 1e-12 && ++iterations < 100);

        double tmp = sinU1 * sinSigma - cosU1 * cosSigma * cosAlpha1;
        double lat2 = Math.atan2(
                sinU1 * cosSigma + cosU1 * sinSigma * cosAlpha1,
                (1 - f) * Math.sqrt(sinAlpha * sinAlpha + tmp * tmp)
        );

        double lambda = Math.atan2(
                sinAlpha,
                tmp * Math.cos(lat2)
        );

        double C = f / 16 * cos2Alpha * (4 + f * (4 - 3 * cos2Alpha));
        double L = lambda - (1 - C) * f * sinAlpha * (sigma + C * sinSigma *
                (cos2SigmaM + C * cosSigma * (-1 + 2 * cos2SigmaM * cos2SigmaM)));

        double lon2 = lon1 + L;

        return GeoCoordinate.builder()
                .latitude(Math.toDegrees(lat2))
                .longitude(Math.toDegrees(lon2))
                .altitude(start.getAltitude())
                .build();
    }

    /**
     * Generate a great circle path between two points
     *
     * @param start Starting point
     * @param end Ending point
     * @param numPoints Number of points to generate (including start and end)
     * @return List of points along the great circle path
     */
    public static List<GeoCoordinate> greatCirclePath(GeoCoordinate start, GeoCoordinate end, int numPoints) {
        List<GeoCoordinate> path = new ArrayList<>();

        if (numPoints < 2) {
            throw new IllegalArgumentException("Number of points must be at least 2");
        }

        // Add the starting point
        path.add(start);

        if (numPoints == 2) {
            // Just start and end
            path.add(end);
            return path;
        }

        double lat1 = Math.toRadians(start.getLatitude());
        double lon1 = Math.toRadians(start.getLongitude());
        double lat2 = Math.toRadians(end.getLatitude());
        double lon2 = Math.toRadians(end.getLongitude());

        // For altitude interpolation
        double altStart = start.getAltitude();
        double altEnd = end.getAltitude();
        double altDiff = altEnd - altStart;

        // Generate intermediate points
        for (int i = 1; i < numPoints - 1; i++) {
            double fraction = (double) i / (numPoints - 1);

            // Great circle interpolation
            double a = Math.sin((1 - fraction) * fraction) / Math.sin(fraction);
            double b = Math.sin(fraction * (1 - fraction)) / Math.sin(fraction);

            double x = a * Math.cos(lat1) * Math.cos(lon1) + b * Math.cos(lat2) * Math.cos(lon2);
            double y = a * Math.cos(lat1) * Math.sin(lon1) + b * Math.cos(lat2) * Math.sin(lon2);
            double z = a * Math.sin(lat1) + b * Math.sin(lat2);

            double lat = Math.atan2(z, Math.sqrt(x * x + y * y));
            double lon = Math.atan2(y, x);

            // Linear altitude interpolation
            double alt = altStart + fraction * altDiff;

            path.add(GeoCoordinate.builder()
                    .latitude(Math.toDegrees(lat))
                    .longitude(Math.toDegrees(lon))
                    .altitude(alt)
                    .build());
        }

        // Add the ending point
        path.add(end);

        return path;
    }

    /**
     * Calculate the cross-track distance of a point from a great circle path
     *
     * @param pathStart Start point of path
     * @param pathEnd End point of path
     * @param point The point to check
     * @return Cross-track distance in nautical miles (negative if to the left of the path)
     */
    public static double crossTrackDistance(GeoCoordinate pathStart, GeoCoordinate pathEnd, GeoCoordinate point) {
        double distAD = haversineDistance(pathStart, point) / EarthModel.MEAN_EARTH_RADIUS_NM;
        double bearingAD = Math.toRadians(initialBearing(pathStart, point));
        double bearingAB = Math.toRadians(initialBearing(pathStart, pathEnd));

        return Math.asin(Math.sin(distAD) * Math.sin(bearingAD - bearingAB)) * EarthModel.MEAN_EARTH_RADIUS_NM;
    }

    /**
     * Calculate the along-track distance of a point from the start of a great circle path
     *
     * @param pathStart Start point of path
     * @param pathEnd End point of path
     * @param point The point to check
     * @return Along-track distance in nautical miles
     */
    public static double alongTrackDistance(GeoCoordinate pathStart, GeoCoordinate pathEnd, GeoCoordinate point) {
        double distAD = haversineDistance(pathStart, point) / EarthModel.MEAN_EARTH_RADIUS_NM;
        double crossTrack = crossTrackDistance(pathStart, pathEnd, point) / EarthModel.MEAN_EARTH_RADIUS_NM;

        return Math.acos(Math.cos(distAD) / Math.cos(crossTrack)) * EarthModel.MEAN_EARTH_RADIUS_NM;
    }

    /**
     * Calculate the closest point on a great circle path to a given point
     *
     * @param pathStart Start point of path
     * @param pathEnd End point of path
     * @param point The point to check
     * @return The closest point on the path
     */
    public static GeoCoordinate closestPointOnPath(GeoCoordinate pathStart, GeoCoordinate pathEnd, GeoCoordinate point) {
        double crossTrack = crossTrackDistance(pathStart, pathEnd, point);
        double alongTrack = alongTrackDistance(pathStart, pathEnd, point);

        // Calculate bearing from start to end
        double bearing = initialBearing(pathStart, pathEnd);

        // Calculate the closest point
        return destinationPoint(pathStart, alongTrack, bearing);
    }

    /**
     * Check if a point is on a path segment (within a tolerance)
     *
     * @param pathStart Start point of path segment
     * @param pathEnd End point of path segment
     * @param point The point to check
     * @param toleranceNM Tolerance in nautical miles
     * @return True if the point is on the path segment
     */
    public static boolean isPointOnPathSegment(GeoCoordinate pathStart, GeoCoordinate pathEnd,
                                               GeoCoordinate point, double toleranceNM) {
        // Check cross-track distance is within tolerance
        double crossTrack = Math.abs(crossTrackDistance(pathStart, pathEnd, point));
        if (crossTrack > toleranceNM) {
            return false;
        }

        // Check along-track distance is within path length
        double alongTrack = alongTrackDistance(pathStart, pathEnd, point);
        double pathLength = haversineDistance(pathStart, pathEnd);

        return alongTrack >= -toleranceNM && alongTrack <= pathLength + toleranceNM;
    }

    /**
     * Calculate the area of a polygon on the Earth's surface
     * Uses the spherical excess formula
     *
     * @param vertices List of points forming a closed polygon
     * @return Area in square nautical miles
     */
    public static double polygonArea(List<GeoCoordinate> vertices) {
        int n = vertices.size();

        if (n < 3) {
            return 0;
        }

        // Make sure first and last points are the same (closed polygon)
        List<GeoCoordinate> points = new ArrayList<>(vertices);
        if (!points.get(0).equals(points.get(n - 1))) {
            points.add(points.get(0));
            n++;
        }

        // Convert to radians
        double[][] radians = new double[n][2];
        for (int i = 0; i < n; i++) {
            radians[i][0] = Math.toRadians(points.get(i).getLatitude());
            radians[i][1] = Math.toRadians(points.get(i).getLongitude());
        }

        // Calculate the sum of the angles
        double sum = 0;
        for (int i = 0; i < n - 1; i++) {
            sum += (radians[i + 1][1] - radians[0][1]) *
                    Math.sin(radians[i][0] + (radians[i + 1][0] - radians[i][0]) / 2);
        }

        // Calculate area using the spherical excess formula
        double earthRadius = EarthModel.MEAN_EARTH_RADIUS_NM;
        double area = Math.abs(sum) * earthRadius * earthRadius / 2;

        return area;
    }

    private GeodeticCalculator() {
    }



    public static double minimumDistance(SpatialLine spatialLine, SpatialLine obstacleLine) {
        SpatialPoint p1 = spatialLine.getStartPoint();
        SpatialPoint q1 = spatialLine.getEndPoint();
        SpatialPoint p2 = obstacleLine.getStartPoint();
        SpatialPoint q2 = obstacleLine.getEndPoint();

        // Calculate the minimum distance between the two line segments
        double minDistance = Math.min(
                Math.min(
                        pointToSegmentDistance(p1, p2, q2),
                        pointToSegmentDistance(q1, p2, q2)
                ),
                Math.min(
                        pointToSegmentDistance(p2, p1, q1),
                        pointToSegmentDistance(q2, p1, q1)
                )
        );

        return minDistance;
    }

    private static double pointToSegmentDistance(SpatialPoint p, SpatialPoint v, SpatialPoint w) {
        double l2 = v.distanceTo(w) * v.distanceTo(w);
        if (l2 == 0.0) return p.distanceTo(v);
        double t = ((p.getCoordinate().getLatitude() - v.getCoordinate().getLatitude()) * (w.getCoordinate().getLatitude() - v.getCoordinate().getLatitude()) + (p.getCoordinate().getLongitude() - v.getCoordinate().getLongitude()) * (w.getCoordinate().getLongitude() - v.getCoordinate().getLongitude())) / l2;
        t = Math.max(0, Math.min(1, t));
        double x = v.getCoordinate().getLatitude() + t * (w.getCoordinate().getLatitude() - v.getCoordinate().getLatitude());
        double y = v.getCoordinate().getLongitude() + t * (w.getCoordinate().getLongitude() - v.getCoordinate().getLongitude());
        GeoCoordinate geo = GeoCoordinate.builder().latitude(x).longitude(y).altitude(p.getCoordinate().getAltitude()).build();
        SpatialPoint projection = SpatialPoint.builder().coordinate(geo).build();
        return p.distanceTo(projection);
    }

    public static double minimumDistance(SpatialLine spatialLine, SpatialVolume obstacleVolume) {
        double minDistance = Double.POSITIVE_INFINITY;

        // Get the base polygon of the volume
        SpatialPolygon basePolygon = obstacleVolume.getBasePolygon();

        // Get the altitude bounds of the volume
        double lowerAltitude = obstacleVolume.getLowerAltitude();
        double upperAltitude = obstacleVolume.getUpperAltitude();

        // Check distance to each edge of the base polygon at both lower and upper altitudes
        for (int i = 0; i < basePolygon.getVertices().size(); i++) {
            SpatialPoint p1 = basePolygon.getVertices().get(i);
            SpatialPoint p2 = basePolygon.getVertices().get((i + 1) % basePolygon.getVertices().size());

            // Create edges at lower and upper altitudes
            SpatialLine lowerEdge = new SpatialLine(
                    new SpatialPoint(p1.getCoordinate().withAltitude(lowerAltitude)),
                    new SpatialPoint(p2.getCoordinate().withAltitude(lowerAltitude))
            );
            SpatialLine upperEdge = new SpatialLine(
                    new SpatialPoint(p1.getCoordinate().withAltitude(upperAltitude)),
                    new SpatialPoint(p2.getCoordinate().withAltitude(upperAltitude))
            );

            // Calculate minimum distance to these edges
            minDistance = Math.min(minDistance, GeodeticCalculator.minimumDistance(spatialLine, lowerEdge));
            minDistance = Math.min(minDistance, GeodeticCalculator.minimumDistance(spatialLine, upperEdge));
        }

        // Check distance to vertical edges connecting lower and upper altitudes
        for (SpatialPoint vertex : basePolygon.getVertices()) {
            SpatialLine verticalEdge = new SpatialLine(
                    new SpatialPoint(vertex.getCoordinate().withAltitude(lowerAltitude)),
                    new SpatialPoint(vertex.getCoordinate().withAltitude(upperAltitude))
            );

            minDistance = Math.min(minDistance, GeodeticCalculator.minimumDistance(spatialLine, verticalEdge));
        }

        return minDistance;
    }
}