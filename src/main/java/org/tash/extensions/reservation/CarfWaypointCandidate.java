package org.tash.extensions.reservation;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;

@Data
@Builder
public class CarfWaypointCandidate {
    private String identifier;
    private String countryCode;
    private GeoCoordinate coordinate;
    private boolean preferred;
}
