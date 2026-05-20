package org.tash.extensions.weather.pirep;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;

import java.time.Duration;

@Data
@Builder
public class PirepDuplicatePolicy {
    @Builder.Default
    private final Duration timeTolerance = Duration.ofMinutes(10);
    @Builder.Default
    private final double distanceToleranceNauticalMiles = 10.0;

    public boolean isDuplicate(PirepReport candidate, PirepReport existing) {
        if (candidate == null || existing == null) {
            return false;
        }
        if (candidate.getObservationTime() == null || existing.getObservationTime() == null) {
            return false;
        }
        if (candidate.getPhenomenon() != existing.getPhenomenon()) {
            return false;
        }
        if (candidate.getAircraftType() != null && existing.getAircraftType() != null
                && !candidate.getAircraftType().equalsIgnoreCase(existing.getAircraftType())) {
            return false;
        }
        Duration delta = Duration.between(candidate.getObservationTime(), existing.getObservationTime()).abs();
        if (delta.compareTo(timeTolerance) > 0) {
            return false;
        }
        GeoCoordinate first = candidate.getLocation();
        GeoCoordinate second = existing.getLocation();
        return first != null && second != null && first.distanceTo(second) <= distanceToleranceNauticalMiles;
    }
}
