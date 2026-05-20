package org.tash.extensions.routing;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.engine.OperationalConstraint;
import org.tash.extensions.engine.OperationalConstraintType;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class RoutePlanningConstraint {
    private final String id;
    private final OperationalConstraintType type;
    private final ZonedDateTime startTime;
    private final ZonedDateTime endTime;
    private final double lowerAltitudeFeet;
    private final double upperAltitudeFeet;
    private final String rationale;
    @Builder.Default
    private final List<GeoCoordinate> geometry = new ArrayList<>();

    public static RoutePlanningConstraint from(OperationalConstraint constraint) {
        return RoutePlanningConstraint.builder()
                .id(constraint.getId())
                .type(constraint.getType())
                .startTime(constraint.getStartTime())
                .endTime(constraint.getEndTime())
                .lowerAltitudeFeet(constraint.getLowerAltitudeFeet())
                .upperAltitudeFeet(constraint.getUpperAltitudeFeet())
                .rationale(constraint.getRationale())
                .geometry(new ArrayList<>(constraint.getGeometry()))
                .build();
    }

    public List<GeoCoordinate> getGeometry() {
        return Collections.unmodifiableList(geometry == null ? Collections.emptyList() : geometry);
    }
}
