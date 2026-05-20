package org.tash.extensions.weather.decision;

import lombok.Builder;
import lombok.Data;

import java.time.ZonedDateTime;

@Data
@Builder
public class SectorDemandSnapshot {
    private final AirspaceSector sector;
    private final ZonedDateTime validTime;
    private final double activeDemandPerHour;
}
