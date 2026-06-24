package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;
import org.tash.extensions.product.dto.ProductDtos;
import org.tash.extensions.visualization.AirspaceFeatureCollection;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class SimulationStepResult {
    private String id;
    private int offsetMinutes;
    private ZonedDateTime simulatedTime;
    private SimulationEvent injectedEvent;
    private String engineAction;
    private String recommendedAction;
    private double confidence;
    private ProductDtos.MissionWeatherVerdictSummary missionVerdict;
    private ProductDtos.RouteImpactSummary routeImpact;
    private ProductDtos.CoordinationDraftSummary coordinationDraft;
    private ProductDtos.PilotBriefSummary pilotBrief;
    private AirspaceFeatureCollection features;
    private SimulationDynamicsSnapshot dynamics;
    @Builder.Default
    private List<String> affectedMissionDeltas = new ArrayList<>();
    @Builder.Default
    private List<String> routeImpactDeltas = new ArrayList<>();
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
    @Builder.Default
    private Map<String, String> replayAuditIds = new java.util.LinkedHashMap<>();
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
}
