package org.tash.extensions.carf.altrv;

import lombok.Builder;
import lombok.Data;

import java.util.List;

@Data
@Builder
public class AltrvArea {
    private AltrvAreaType type;
    private String rawText;
    private double radiusNauticalMiles;
    private double widthNauticalMiles;
    private List<AltrvRoutePoint> boundaryPoints;
    private AltrvSourceSpan sourceSpan;
}
