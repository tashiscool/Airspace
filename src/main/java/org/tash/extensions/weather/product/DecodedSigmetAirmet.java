package org.tash.extensions.weather.product;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder
public class DecodedSigmetAirmet {
    private final WeatherProductType type;
    private final String hazard;
    private final String coverage;
    private final Double lineWidthNauticalMiles;
    private final Double lowerAltitudeFeet;
    private final Double upperAltitudeFeet;
    private final WeatherMovementVector movement;
    private final Double echoTopFeet;
    @Builder.Default
    private final List<GeoCoordinate> geometry = new ArrayList<>();

    public List<GeoCoordinate> getGeometry() {
        return Collections.unmodifiableList(geometry == null ? Collections.emptyList() : geometry);
    }
}
