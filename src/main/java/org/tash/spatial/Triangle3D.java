package org.tash.spatial;

import lombok.AllArgsConstructor;
import lombok.Data;
import org.tash.data.GeoCoordinate;

/**
 * Helper class for 3D triangles
 */
@Data
@AllArgsConstructor
public class Triangle3D {
    private GeoCoordinate v1;
    private GeoCoordinate v2;
    private GeoCoordinate v3;
}
