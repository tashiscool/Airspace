package org.tash.extensions.visualization;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;

public class GeoJsonAirspaceExporter {
    private final ObjectMapper mapper;

    public GeoJsonAirspaceExporter() {
        this(new ObjectMapper());
    }

    public GeoJsonAirspaceExporter(ObjectMapper mapper) {
        this.mapper = mapper;
    }

    public String toGeoJson(AirspaceFeatureCollection collection) {
        try {
            return mapper.writeValueAsString(collection);
        } catch (JsonProcessingException ex) {
            throw new IllegalStateException("Unable to serialize airspace GeoJSON", ex);
        }
    }
}
