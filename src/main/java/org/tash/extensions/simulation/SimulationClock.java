package org.tash.extensions.simulation;


import lombok.Builder;
import lombok.Data;
import lombok.extern.jackson.Jacksonized;

import java.time.ZoneOffset;
import java.time.ZonedDateTime;

@Data
@Builder
@Jacksonized
public class SimulationClock {
    @Builder.Default
    private ZonedDateTime startTime = ZonedDateTime.of(2026, 6, 20, 12, 0, 0, 0, ZoneOffset.UTC);

    public ZonedDateTime atOffsetMinutes(int offsetMinutes) {
        return startTime.plusMinutes(offsetMinutes);
    }
}
