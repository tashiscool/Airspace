package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class FlowEvaluationAreaModel {
    private String areaId;
    private TrafficManagementInitiativeType areaType;
    private String name;
    private String geometryType;
    @Builder.Default
    private List<List<Double>> geometry = new ArrayList<>();
    @Builder.Default
    private List<String> sectorIds = new ArrayList<>();
    @Builder.Default
    private List<String> fixIds = new ArrayList<>();
    @Builder.Default
    private List<String> routeFilters = new ArrayList<>();
    private int lowerAltitudeFeet;
    private int upperAltitudeFeet;
    private int startOffsetMinutes;
    private int endOffsetMinutes;
    private int acceptanceRatePerHour;
    private boolean controlArea;
    private String rationale;
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
}
