package org.tash.extensions.routing;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.engine.DecisionSourceRef;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class OperationalRoutePlanResult {
    @Builder.Default
    private final List<GeoCoordinate> originalRoute = new ArrayList<>();
    @Builder.Default
    private final List<RouteCandidate> candidates = new ArrayList<>();
    private final boolean blocked;
    private final String rationale;
    @Builder.Default
    private final List<DecisionSourceRef> decisionTraceRefs = new ArrayList<>();

    public List<RouteCandidate> getCandidates() {
        return Collections.unmodifiableList(candidates == null ? Collections.emptyList() : candidates);
    }
}
