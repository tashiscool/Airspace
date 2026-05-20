package org.tash.extensions.weather.decision;

import lombok.Builder;
import lombok.Data;

import java.time.ZonedDateTime;

@Data
@Builder
public class RouteDemandSnapshot {
    private final String routeId;
    private final ZonedDateTime validTime;
    private final double activeDemandPerHour;
}
