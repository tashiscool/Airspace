package org.tash.extensions.engine;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.weather.pirep.PirepReport;

import java.time.Duration;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.List;

@Data
@Builder
public class PirepRelevanceRequest {
    @Builder.Default private final List<GeoCoordinate> route = new ArrayList<>();
    @Builder.Default private final List<PirepReport> pireps = new ArrayList<>();
    private final ZonedDateTime decisionTime;
    @Builder.Default private final double routeBufferNauticalMiles = 40.0;
    @Builder.Default private final double altitudeToleranceFeet = 2000.0;
    @Builder.Default private final Duration recencyWindow = Duration.ofMinutes(60);
    private final Double routeAltitudeFeet;
}
