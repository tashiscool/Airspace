package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class HistoricalReplayDay {
    private String id;
    private String name;
    private String operatingDate;
    private String scenarioId;
    private String sourceMode;
    private String sourceFamily;
    private String authorizationMode;
    private String providerFamily;
    private String sourceVersion;
    private String expectedFinalAction;
    private TrafficReplayBundle trafficReplay;
    @Builder.Default
    private List<String> airportIds = new ArrayList<>();
    @Builder.Default
    private List<String> sectorIds = new ArrayList<>();
    @Builder.Default
    private List<String> tags = new ArrayList<>();
    @Builder.Default
    private List<String> publicSourceRefs = new ArrayList<>();
    @Builder.Default
    private List<String> providerReceiptIds = new ArrayList<>();
    @Builder.Default
    private List<String> dataQualityWarnings = new ArrayList<>();
    @Builder.Default
    private List<String> assumptions = new ArrayList<>();
    @Builder.Default
    private List<HistoricalReplayOutcome> expectedOutcomes = new ArrayList<>();
    @Builder.Default
    private Map<String, String> expectedSummary = new LinkedHashMap<>();
}
