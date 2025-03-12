package org.tash.extensions.geodetic;

import lombok.*;
import org.tash.data.GeoCoordinate;

/**
 * North-East-Down (NED) coordinate system
 * Local tangent plane coordinates with origin at a reference point
 * Similar to ENU but with different axis orientation
 */
@Data
@Builder
@NoArgsConstructor
@AllArgsConstructor
public class NEDCoordinate {
    /** North coordinate in meters */
    private double north;
    
    /** East coordinate in meters */
    private double east;
    
    /** Down coordinate in meters */
    private double down;
    
    /**
     * Convert from ENU to NED coordinates
     * 
     * @param enu ENU coordinate
     * @return NED coordinate
     */
    public static NEDCoordinate fromENU(ENUCoordinate enu) {
        return new NEDCoordinate(
            enu.getNorth(),
            enu.getEast(),
            -enu.getUp()
        );
    }
    
    /**
     * Convert to ENU coordinates
     * 
     * @return ENU coordinate
     */
    public ENUCoordinate toENU() {
        return new ENUCoordinate(
            this.east,
            this.north,
            -this.down
        );
    }
    
    /**
     * Convert from geodetic to NED coordinates
     * 
     * @param coordinate The point to convert
     * @param reference The reference point (origin of NED)
     * @return NED coordinate
     */
    public static NEDCoordinate fromGeodetic(GeoCoordinate coordinate, GeoCoordinate reference) {
        ENUCoordinate enu = ENUCoordinate.fromGeodetic(coordinate, reference);
        return fromENU(enu);
    }
    
    /**
     * Convert NED to geodetic coordinates
     * 
     * @param reference The reference point (origin of NED)
     * @return Geodetic coordinate
     */
    public GeoCoordinate toGeodetic(GeoCoordinate reference) {
        return toENU().toGeodetic(reference);
    }
}