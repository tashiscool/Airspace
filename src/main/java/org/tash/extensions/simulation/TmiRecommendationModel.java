package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class TmiRecommendationModel {
    private String id;
    private TrafficManagementInitiativeType recommendedType;
    private String action;
    private String targetResourceType;
    private String targetResourceId;
    private String trigger;
    private String severity;
    private int expectedDelayMinutes;
    private double confidence;
    private String rationale;
    @Builder.Default
    private List<String> sourceTmiIds = new ArrayList<>();
    @Builder.Default
    private List<String> affectedFlightIds = new ArrayList<>();
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
}
