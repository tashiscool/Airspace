package org.tash.extensions.simulation;

import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class HistoricalReplayOutcome {
    private String id;
    private String outcomeType;
    private String label;
    private String expectedAction;
    private String observedAction;
    private String targetResourceType;
    private String targetResourceId;
    private int offsetMinutes;
    private int expectedDelayMinutes;
    private double expectedConfidence;
    private boolean routeBlocked;
    private boolean rerouteExpected;
    private boolean staleDataExpected;
    private boolean capacityCompressionExpected;
    private boolean falseClearLabel;
    private boolean falseBlockLabel;
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
    @Builder.Default
    private List<String> diagnostics = new ArrayList<>();
}
