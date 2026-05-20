package org.tash.extensions.weather.decision;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class AirspaceSector {
    private final String id;
    private final String name;
    private final double baselineCapacityPerHour;
    @Builder.Default
    private final List<GeoCoordinate> boundary = new ArrayList<>();

    public List<GeoCoordinate> getBoundary() {
        return Collections.unmodifiableList(boundary == null ? Collections.emptyList() : boundary);
    }
}
