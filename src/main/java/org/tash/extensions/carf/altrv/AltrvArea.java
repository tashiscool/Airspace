package org.tash.extensions.carf.altrv;

import lombok.Builder;
import lombok.Data;

import java.util.List;
import java.util.Map;

@Data
@Builder
public class AltrvArea {
    private AltrvAreaType type;
    private String rawText;
    private String geometryIntent;
    private String enterExitAssociation;
    private String lowerFlightLevel;
    private String upperFlightLevel;
    private String timingText;
    private double radiusNauticalMiles;
    private double widthNauticalMiles;
    private List<AltrvRoutePoint> boundaryPoints;
    private AltrvSourceSpan sourceSpan;
    private Map<String, String> metadata;
}
