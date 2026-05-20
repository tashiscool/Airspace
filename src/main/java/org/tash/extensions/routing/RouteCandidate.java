package org.tash.extensions.routing;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class RouteCandidate {
    private final String id;
    private final double cost;
    private final String rationale;
    @Builder.Default
    private final List<GeoCoordinate> points = new ArrayList<>();
    @Builder.Default
    private final List<String> avoidedConstraintIds = new ArrayList<>();
    @Builder.Default
    private final List<String> remainingConstraintIds = new ArrayList<>();

    public List<GeoCoordinate> getPoints() {
        return Collections.unmodifiableList(points == null ? Collections.emptyList() : points);
    }

    public List<String> getAvoidedConstraintIds() {
        return Collections.unmodifiableList(avoidedConstraintIds == null ? Collections.emptyList() : avoidedConstraintIds);
    }

    public List<String> getRemainingConstraintIds() {
        return Collections.unmodifiableList(remainingConstraintIds == null ? Collections.emptyList() : remainingConstraintIds);
    }
}
