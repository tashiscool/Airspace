package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

@Data
@Builder
@Jacksonized
public class SimulationTimeline {
    private ZonedDateTime startTime;
    private ZonedDateTime endTime;
    @Builder.Default
    private List<SimulationEvent> events = new ArrayList<>();

    public List<SimulationEvent> orderedEvents() {
        return events.stream()
                .sorted(Comparator.comparingInt(SimulationEvent::getOffsetMinutes).thenComparing(SimulationEvent::getId))
                .toList();
    }
}
