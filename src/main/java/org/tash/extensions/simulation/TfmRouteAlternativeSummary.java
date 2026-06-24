package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class TfmRouteAlternativeSummary {
    private String id;
    private String routeName;
    private String routeText;
    private String advisoryType;
    private boolean required;
    private String targetResourceId;
    private String reason;
    private int startOffsetMinutes;
    private int endOffsetMinutes;
    private int expectedDelayMinutes;
    private double confidence;
    private int affectedFlightCount;
    private String residualRisk;
    @Builder.Default
    private List<List<Double>> routePoints = new ArrayList<>();
    @Builder.Default
    private List<String> affectedFlightIds = new ArrayList<>();
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
}
