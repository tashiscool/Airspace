package org.tash.extensions.engine;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.weather.decision.WeatherDecisionSeverity;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class OperationalConstraint {
    private final String id;
    private final OperationalConstraintType type;
    private final ZonedDateTime startTime;
    private final ZonedDateTime endTime;
    private final double lowerAltitudeFeet;
    private final double upperAltitudeFeet;
    private final WeatherDecisionSeverity severity;
    private final double confidence;
    private final String rationale;
    @Builder.Default
    private final List<GeoCoordinate> geometry = new ArrayList<>();
    @Builder.Default
    private final List<DecisionSourceRef> sources = new ArrayList<>();
    @Builder.Default
    private final List<String> componentConstraintIds = new ArrayList<>();
    @Builder.Default
    private final List<String> overlapDimensions = new ArrayList<>();

    public List<GeoCoordinate> getGeometry() {
        return Collections.unmodifiableList(geometry);
    }

    public List<DecisionSourceRef> getSources() {
        return Collections.unmodifiableList(sources);
    }

    public List<String> getComponentConstraintIds() {
        return Collections.unmodifiableList(componentConstraintIds);
    }

    public List<String> getOverlapDimensions() {
        return Collections.unmodifiableList(overlapDimensions);
    }
}
