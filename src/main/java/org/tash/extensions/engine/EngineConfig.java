package org.tash.extensions.engine;

import lombok.Builder;
import lombok.Data;
import org.tash.extensions.engine.spatial.SpatialTopologyBackend;

@Data
@Builder
public class EngineConfig {
    @Builder.Default
    private final String engineVersion = "airspace-weather-engine-2026-05-20";
    @Builder.Default
    private final String ruleCatalogVersion = DecisionRuleCatalog.version();
    @Builder.Default
    private final boolean strictReplayRuleVersion = true;
    @Builder.Default
    private final double indexCellResolutionDegrees = 0.25;
    @Builder.Default
    private final int timeBucketMinutes = 15;
    @Builder.Default
    private final int altitudeBucketFeet = 2000;
    @Builder.Default
    private final SpatialTopologyBackend preciseTopologyBackend = SpatialTopologyBackend.JTS;
    @Builder.Default
    private final SpatialTopologyBackend discreteIndexBackend = SpatialTopologyBackend.H3;
    @Builder.Default
    private final int h3Resolution = 5;
    @Builder.Default
    private final int s2Level = 10;
}
