package org.tash.extensions.weather.wind;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.weather.Vector3;
import org.tash.extensions.weather.WindComponent;

import java.time.ZonedDateTime;
import java.util.*;


/**
 * A wind field model that uses a grid of wind data points
 */
@Data
public class GriddedWindField extends AbstractWindField {
    /**
     * Grid resolution in degrees of latitude/longitude
     */
    private final double gridResolution;

    /**
     * Altitude levels in feet
     */
    private final List<Double> altitudeLevels;

    /**
     * Grid data stored as [lat_index][lon_index][alt_index]
     */
    private final Map<Integer, Map<Integer, Map<Integer, WindComponent>>> gridData;

    /**
     * Bounds of the grid
     */
    private final double minLat;
    private final double maxLat;
    private final double minLon;
    private final double maxLon;

    @Builder
    public GriddedWindField(
            String id,
            ZonedDateTime validityTime,
            double gridResolution,
            List<Double> altitudeLevels,
            double minLat,
            double maxLat,
            double minLon,
            double maxLon) {
        super(id, validityTime);
        this.gridResolution = gridResolution;
        this.altitudeLevels = new ArrayList<>(altitudeLevels);
        this.minLat = minLat;
        this.maxLat = maxLat;
        this.minLon = minLon;
        this.maxLon = maxLon;
        this.gridData = new HashMap<>();
    }

    /**
     * Set wind data at a specific grid point
     *
     * @param latIndex Latitude index
     * @param lonIndex Longitude index
     * @param altIndex Altitude index
     * @param wind     Wind component data
     */
    public void setWindData(int latIndex, int lonIndex, int altIndex, WindComponent wind) {
        gridData.computeIfAbsent(latIndex, k -> new HashMap<>())
                .computeIfAbsent(lonIndex, k -> new HashMap<>())
                .put(altIndex, wind);
    }

    /**
     * Convert latitude to grid index
     *
     * @param latitude Latitude in degrees
     * @return Grid index
     */
    private int getLatIndex(double latitude) {
        return (int) Math.round((latitude - minLat) / gridResolution);
    }

    /**
     * Convert longitude to grid index
     *
     * @param longitude Longitude in degrees
     * @return Grid index
     */
    private int getLonIndex(double longitude) {
        return (int) Math.round((longitude - minLon) / gridResolution);
    }

    /**
     * Find the closest altitude level index
     *
     * @param altitude Altitude in feet
     * @return Index of the closest altitude level
     */
    private int getAltitudeIndex(double altitude) {
        int closest = 0;
        double minDiff = Double.MAX_VALUE;

        for (int i = 0; i < altitudeLevels.size(); i++) {
            double diff = Math.abs(altitudeLevels.get(i) - altitude);
            if (diff < minDiff) {
                minDiff = diff;
                closest = i;
            }
        }

        return closest;
    }

    /**
     * Get latitude for a grid index
     *
     * @param latIndex Grid index
     * @return Latitude in degrees
     */
    private double getLatFromIndex(int latIndex) {
        return minLat + latIndex * gridResolution;
    }

    /**
     * Get longitude for a grid index
     *
     * @param lonIndex Grid index
     * @return Longitude in degrees
     */
    private double getLonFromIndex(int lonIndex) {
        return minLon + lonIndex * gridResolution;
    }

    @Override
    public WindComponent getWindAt(GeoCoordinate location, ZonedDateTime time) {
        // Check if location is within bounds
        if (location.getLatitude() < minLat || location.getLatitude() > maxLat ||
                location.getLongitude() < minLon || location.getLongitude() > maxLon) {
            return WindComponent.builder().direction(0).speed(0).verticalSpeed(0).build();
        }

        // Find nearest altitude level
        int altIndex = getAltitudeIndex(location.getAltitude());

        // Perform trilinear interpolation for the wind
        return interpolateWind(location, altIndex);
    }

    /**
     * Interpolate wind data at a specific location and altitude index
     *
     * @param location Geographic location
     * @param altIndex Altitude index
     * @return Interpolated wind component
     */
    private WindComponent interpolateWind(GeoCoordinate location, int altIndex) {
        double lat = location.getLatitude();
        double lon = location.getLongitude();

        // Get the grid cell indices
        int latIndex = getLatIndex(lat);
        int lonIndex = getLonIndex(lon);

        // Calculate interpolation weights
        double latFrac = (lat - getLatFromIndex(latIndex)) / gridResolution;
        double lonFrac = (lon - getLatFromIndex(lonIndex)) / gridResolution;

        // Get the wind at the 4 surrounding grid points
        WindComponent w00 = getWindDataAt(latIndex, lonIndex, altIndex)
                .orElse(WindComponent.builder().direction(0).speed(0).verticalSpeed(0).build());

        WindComponent w01 = getWindDataAt(latIndex, lonIndex + 1, altIndex)
                .orElse(WindComponent.builder().direction(0).speed(0).verticalSpeed(0).build());

        WindComponent w10 = getWindDataAt(latIndex + 1, lonIndex, altIndex)
                .orElse(WindComponent.builder().direction(0).speed(0).verticalSpeed(0).build());

        WindComponent w11 = getWindDataAt(latIndex + 1, lonIndex + 1, altIndex)
                .orElse(WindComponent.builder().direction(0).speed(0).verticalSpeed(0).build());

        // Convert to vectors for interpolation
        Vector3 v00 = w00.toVector();
        Vector3 v01 = w01.toVector();
        Vector3 v10 = w10.toVector();
        Vector3 v11 = w11.toVector();

        // Perform bilinear interpolation
        Vector3 v0 = v00.multiply(1 - lonFrac).add(v01.multiply(lonFrac));
        Vector3 v1 = v10.multiply(1 - lonFrac).add(v11.multiply(lonFrac));
        Vector3 v = v0.multiply(1 - latFrac).add(v1.multiply(latFrac));

        return v.toWindComponent();
    }

    /**
     * Get wind data at specific grid indices
     *
     * @param latIndex Latitude index
     * @param lonIndex Longitude index
     * @param altIndex Altitude index
     * @return Optional containing wind data if it exists
     */
    private Optional<WindComponent> getWindDataAt(int latIndex, int lonIndex, int altIndex) {
        return Optional.ofNullable(gridData.getOrDefault(latIndex, new HashMap<>())
                .getOrDefault(lonIndex, new HashMap<>())
                .get(altIndex));
    }
}
