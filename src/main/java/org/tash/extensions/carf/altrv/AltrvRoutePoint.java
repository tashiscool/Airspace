package org.tash.extensions.carf.altrv;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;

@Data
@Builder
public class AltrvRoutePoint {
    private String id;
    private String rawText;
    private String timeOffset;
    private GeoCoordinate coordinate;
    private Double radialDegrees;
    private Double dmeNauticalMiles;
    private boolean coordinateLiteral;
    private AltrvSourceSpan sourceSpan;
}
