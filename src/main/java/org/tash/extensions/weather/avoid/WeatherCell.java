package org.tash.extensions.weather.avoid;

import org.tash.data.GeoCoordinate;
import lombok.Data;
import org.tash.extensions.weather.HazardSeverity;
import org.tash.extensions.weather.HazardousWeather;
import org.tash.extensions.weather.WeatherElementType;

import java.time.ZonedDateTime;
import java.util.List;

/**
 * Base class for weather cells - regions of hazardous weather
 */
@Data
public abstract class WeatherCell implements HazardousWeather {
    protected final String id;
    protected final WeatherElementType type;
    protected final HazardSeverity severity;
    protected final ZonedDateTime startTime;
    protected final ZonedDateTime endTime;
    protected final double minAltitude;  // feet
    protected final double maxAltitude;  // feet
    
    public WeatherCell(
            String id,
            WeatherElementType type,
            HazardSeverity severity,
            ZonedDateTime startTime,
            ZonedDateTime endTime,
            double minAltitude,
            double maxAltitude) {
        this.id = id;
        this.type = type;
        this.severity = severity;
        this.startTime = startTime;
        this.endTime = endTime;
        this.minAltitude = minAltitude;
        this.maxAltitude = maxAltitude;
    }
    
    @Override
    public String getId() {
        return id;
    }
    
    @Override
    public WeatherElementType getType() {
        return type;
    }
    
    @Override
    public HazardSeverity getSeverity() {
        return severity;
    }
    
    @Override
    public ZonedDateTime getValidityTime() {
        // Use the middle of the time range as the validity time
        return startTime.plus(java.time.Duration.between(startTime, endTime).dividedBy(2));
    }
    
    @Override
    public boolean isValidAt(ZonedDateTime time) {
        return !time.isBefore(startTime) && !time.isAfter(endTime);
    }
    
    @Override
    public boolean affectsPoint(GeoCoordinate point, ZonedDateTime time) {
        // Check time range
        if (!isValidAt(time)) {
            return false;
        }
        
        // Check altitude range
        if (point.getAltitude() < minAltitude || point.getAltitude() > maxAltitude) {
            return false;
        }
        
        // Check horizontal containment (implemented by subclasses)
        return containsHorizontally(point);
    }
    
    @Override
    public double getCostFactor() {
        return severity.getCostFactor();
    }
    
    /**
     * Check if a point is horizontally contained within the weather cell
     * @param point The point to check
     * @return True if the point is contained horizontally
     */
    protected abstract boolean containsHorizontally(GeoCoordinate point);
    
    /**
     * Get a list of points that represent the outer boundary of the cell
     * @return List of boundary points
     */
    public abstract List<GeoCoordinate> getBoundaryPoints();
    
    /**
     * Check if a direct path between two points intersects this weather cell
     * @param start Start point
     * @param end End point
     * @param time Time of path
     * @return True if the direct path intersects the cell
     */
    public boolean intersectsPath(GeoCoordinate start, GeoCoordinate end, ZonedDateTime time) {
        // Check time range
        if (!isValidAt(time)) {
            return false;
        }
        
        // Check if either endpoint is inside the cell
        if (affectsPoint(start, time) || affectsPoint(end, time)) {
            return true;
        }
        
        // Check if the path intersects the cell boundary
        return doesPathIntersectBoundary(start, end);
    }
    
    /**
     * Check if a path intersects the cell boundary
     * @param start Start point
     * @param end End point
     * @return True if the path intersects the boundary
     */
    protected abstract boolean doesPathIntersectBoundary(GeoCoordinate start, GeoCoordinate end);
}

