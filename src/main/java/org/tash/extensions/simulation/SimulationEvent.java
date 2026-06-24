package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.util.ArrayList;
import java.util.List;

@Data
@Builder(toBuilder = true)
@Jacksonized
public class SimulationEvent {
    private String id;
    private int offsetMinutes;
    private String family;
    private String eventType;
    private String label;
    private String payload;
    private String expectedAction;
    @Builder.Default
    private List<String> sourceRefs = new ArrayList<>();
}
