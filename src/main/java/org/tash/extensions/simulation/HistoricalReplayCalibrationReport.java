package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class HistoricalReplayCalibrationReport {
    private String id;
    private ZonedDateTime generatedAt;
    private String calibrationVersion;
    private int corpusDayCount;
    private int expectedOutcomeCount;
    private int routeOutcomeCount;
    private int weatherOutcomeCount;
    private int pirepOutcomeCount;
    private int notamOutcomeCount;
    private int sectorCapacityOutcomeCount;
    private int falseClearLabelCount;
    private int falseBlockLabelCount;
    private int uncalibratedCoefficientCount;
    private double deterministicAgreementRate;
    private double sourceRefPreservationRate;
    @Builder.Default
    private List<String> sourceModes = new ArrayList<>();
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
    @Builder.Default
    private List<String> uncalibratedCoefficients = new ArrayList<>();
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
    @Builder.Default
    private Map<String, Integer> outcomeCountsByType = new java.util.LinkedHashMap<>();
}
