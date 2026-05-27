package org.tash.extensions.weather.pattern;

import org.tash.data.GeoCoordinate;
import org.tash.extensions.visualization.AirspaceFeature;
import org.tash.extensions.visualization.AirspaceFeatureCollection;
import org.tash.extensions.visualization.AirspaceGeometry;

import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public class WeatherPatternMapFeatureFactory {
    public AirspaceFeatureCollection features(Collection<WeatherPattern> patterns) {
        List<AirspaceFeature> features = new ArrayList<>();
        if (patterns != null) {
            for (WeatherPattern pattern : patterns) {
                features.add(feature(pattern));
            }
        }
        return AirspaceFeatureCollection.builder().features(features).build();
    }

    public AirspaceFeature feature(WeatherPattern pattern) {
        Map<String, Object> properties = new LinkedHashMap<>();
        properties.put("featureKind", "weather-pattern");
        properties.put("id", pattern.getId());
        properties.put("patternType", pattern.getType() == null ? null : pattern.getType().name());
        properties.put("sourceFamily", pattern.getSourceFamily());
        properties.put("sourceProduct", pattern.getSourceProductId());
        properties.put("sourceUrl", pattern.getSourceUrl());
        properties.put("displayLayer", displayLayer(pattern.getType()));
        properties.put("constraintType", "WEATHER_PATTERN");
        properties.put("operationalRole", "time-enabled-weather-pattern");
        properties.put("geometryIntent", pattern.getGeometryIntent());
        properties.put("validStart", string(pattern.getValidStart()));
        properties.put("validEnd", string(pattern.getValidEnd()));
        properties.put("forecastHour", pattern.getForecastHour());
        properties.put("movementBearingDegrees", pattern.getMovementBearingDegrees());
        properties.put("movementSpeedKnots", pattern.getMovementSpeedKnots());
        properties.put("lowerAltitudeFeet", pattern.getLowerAltitudeFeet());
        properties.put("upperAltitudeFeet", pattern.getUpperAltitudeFeet());
        properties.put("severity", pattern.getSeverity());
        properties.put("confidence", pattern.getConfidence());
        properties.put("freshnessCategory", pattern.getFreshnessCategory());
        properties.put("sourceRefs", pattern.getSourceRefs());
        properties.put("rawText", pattern.getRawText());
        properties.put("rationale", pattern.getRationale());
        properties.put("diagnostics", pattern.getDiagnostics());
        return AirspaceFeature.builder()
                .id(pattern.getId())
                .geometry(geometry(pattern))
                .properties(properties)
                .build();
    }

    private AirspaceGeometry geometry(WeatherPattern pattern) {
        List<GeoCoordinate> points = pattern.getGeometry();
        if (points == null || points.isEmpty()) {
            return AirspaceGeometry.builder().type("GeometryCollection").coordinates(new ArrayList<>()).build();
        }
        if (points.size() == 1) {
            return AirspaceGeometry.builder().type("Point").coordinates(position(points.get(0))).build();
        }
        if (points.size() == 2 || "LINE_CORRIDOR".equals(pattern.getGeometryIntent())) {
            List<List<Double>> line = new ArrayList<>();
            for (GeoCoordinate point : points) line.add(position(point));
            return AirspaceGeometry.builder().type("LineString").coordinates(line).build();
        }
        List<List<Double>> ring = new ArrayList<>();
        for (GeoCoordinate point : points) ring.add(position(point));
        if (!ring.isEmpty() && !ring.get(0).equals(ring.get(ring.size() - 1))) {
            ring.add(new ArrayList<>(ring.get(0)));
        }
        List<List<List<Double>>> polygon = new ArrayList<>();
        polygon.add(ring);
        return AirspaceGeometry.builder().type("Polygon").coordinates(polygon).build();
    }

    private String displayLayer(WeatherPatternType type) {
        if (type == WeatherPatternType.CONVECTION) return "wx-convective";
        if (type == WeatherPatternType.TURBULENCE) return "wx-turbulence";
        if (type == WeatherPatternType.ICING) return "wx-icing";
        if (type == WeatherPatternType.WIND_SHEAR) return "wx-wshear";
        if (type == WeatherPatternType.VOLCANIC_ASH) return "wx-ash";
        if (type == WeatherPatternType.PIREP_CLUSTER) return "pireps";
        return "weather";
    }

    private List<Double> position(GeoCoordinate point) {
        List<Double> out = new ArrayList<>();
        out.add(point.getLongitude());
        out.add(point.getLatitude());
        out.add(point.getAltitude());
        return out;
    }

    private String string(Object value) {
        return value == null ? null : String.valueOf(value);
    }
}
