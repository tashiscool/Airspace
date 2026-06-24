package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class TfmActiveConstraintSummary {
    private String id;
    private String type;
    private String status;
    private String scope;
    private String targetResourceId;
    private String reason;
    private int startOffsetMinutes;
    private int endOffsetMinutes;
    private int expectedDelayMinutes;
    private double confidence;
    private int affectedFlightCount;
    @Builder.Default
    private List<String> affectedFlightIds = new ArrayList<>();
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
}
