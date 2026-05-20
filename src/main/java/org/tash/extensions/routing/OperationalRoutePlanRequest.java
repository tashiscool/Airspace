package org.tash.extensions.routing;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class OperationalRoutePlanRequest {
    @Builder.Default
    private final List<GeoCoordinate> originalRoute = new ArrayList<>();
    @Builder.Default
    private final List<RoutePlanningConstraint> constraints = new ArrayList<>();
    private final ZonedDateTime decisionTime;
    @Builder.Default
    private final double corridorWidthNauticalMiles = 10.0;
    @Builder.Default
    private final int maxCandidates = 3;

    public List<GeoCoordinate> getOriginalRoute() {
        return Collections.unmodifiableList(originalRoute == null ? Collections.emptyList() : originalRoute);
    }

    public List<RoutePlanningConstraint> getConstraints() {
        return Collections.unmodifiableList(constraints == null ? Collections.emptyList() : constraints);
    }
}
