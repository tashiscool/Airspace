package org.tash.extensions.weather.pattern;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.feed.OperationalFeedIngestResult;
import org.tash.extensions.feed.OperationalFeedEnvelope;
import org.tash.extensions.weather.HazardSeverity;
import org.tash.extensions.weather.HazardousWeather;
import org.tash.extensions.weather.WeatherElementType;
import org.tash.extensions.weather.avoid.WeatherCell;
import org.tash.extensions.weather.pirep.PirepReport;
import org.tash.extensions.weather.product.WeatherProduct;
import org.tash.extensions.weather.product.WeatherProductParseResult;
import org.tash.extensions.weather.product.WeatherProductType;
import org.tash.extensions.weather.product.WeatherValidityWindow;

import java.time.Duration;
import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.UUID;
import java.util.stream.Collectors;

public class WeatherPatternService {
    private final ObjectMapper mapper = new ObjectMapper();

    public List<WeatherPattern> patternsFromFeedResults(Collection<OperationalFeedIngestResult> results) {
        List<WeatherPattern> patterns = new ArrayList<>();
        if (results == null) {
            return patterns;
        }
        for (OperationalFeedIngestResult result : results) {
            patterns.addAll(patternsFromResult(result));
        }
        patterns.sort(Comparator.comparing(WeatherPattern::getValidStart,
                Comparator.nullsLast(Comparator.naturalOrder())));
        return patterns;
    }

    public List<WeatherPatternEvent> events(List<WeatherPattern> patterns, Collection<String> affectedSourceRefs) {
        Map<WeatherPatternType, List<WeatherPattern>> byType = new LinkedHashMap<>();
        for (WeatherPattern pattern : safe(patterns)) {
            byType.computeIfAbsent(pattern.getType(), ignored -> new ArrayList<>()).add(pattern);
        }
        Set<String> affected = new LinkedHashSet<>(affectedSourceRefs == null ? Collections.emptyList() : affectedSourceRefs);
        List<WeatherPatternEvent> events = new ArrayList<>();
        for (Map.Entry<WeatherPatternType, List<WeatherPattern>> entry : byType.entrySet()) {
            List<WeatherPattern> values = entry.getValue();
            if (values.isEmpty()) {
                continue;
            }
            Set<String> refs = values.stream().flatMap(p -> p.getSourceRefs().stream()).collect(Collectors.toCollection(LinkedHashSet::new));
            int affectedCount = (int) refs.stream().filter(ref -> affected.contains(ref) || affected.contains(stripFamily(ref))).count();
            events.add(WeatherPatternEvent.builder()
                    .id("weather-event-" + entry.getKey().name().toLowerCase(Locale.US))
                    .type(entry.getKey())
                    .label(label(entry.getKey()))
                    .severity(strongestSeverity(values))
                    .confidence(values.stream().mapToDouble(WeatherPattern::getConfidence).max().orElse(0.0))
                    .validStart(values.stream().map(WeatherPattern::getValidStart).filter(v -> v != null).min(ZonedDateTime::compareTo).orElse(null))
                    .validEnd(values.stream().map(WeatherPattern::getValidEnd).filter(v -> v != null).max(ZonedDateTime::compareTo).orElse(null))
                    .affectedMissionCount(affectedCount > 0 ? affectedCount : 0)
                    .productCount(values.size())
                    .pirepCount((int) values.stream().filter(p -> p.getType() == WeatherPatternType.PIREP_CLUSTER).count())
                    .sourceRefs(new ArrayList<>(refs))
                    .patternIds(values.stream().map(WeatherPattern::getId).collect(Collectors.toList()))
                    .representativeGeometry(values.stream()
                            .filter(p -> p.getGeometry() != null && !p.getGeometry().isEmpty())
                            .findFirst()
                            .map(WeatherPattern::getGeometry)
                            .orElse(Collections.emptyList()))
                    .rationale(values.size() + " source product(s) grouped by " + label(entry.getKey()))
                    .build());
        }
        events.sort((left, right) -> severityRank(right.getSeverity()) - severityRank(left.getSeverity()));
        return events;
    }

    public List<RouteWeatherPatternIntersection> routeSample(WeatherPatternQuery query, List<WeatherPattern> patterns) {
        List<GeoCoordinate> route = route(query == null ? null : query.getRoute());
        if (route.size() < 2 || patterns == null || patterns.isEmpty()) {
            return Collections.emptyList();
        }
        double corridor = query.getCorridorNauticalMiles() == null ? 25.0 : Math.max(0, query.getCorridorNauticalMiles());
        List<RouteWeatherPatternIntersection> out = new ArrayList<>();
        for (WeatherPattern pattern : patterns) {
            boolean time = timeOverlap(query, pattern);
            boolean altitude = altitudeOverlap(query, pattern);
            Nearest nearest = nearest(route, pattern.getGeometry());
            boolean geometry = pattern.getGeometry() != null && !pattern.getGeometry().isEmpty()
                    && nearest.distanceNauticalMiles <= corridor;
            if (time && altitude && geometry) {
                out.add(RouteWeatherPatternIntersection.builder()
                        .patternId(pattern.getId())
                        .patternType(pattern.getType())
                        .severity(pattern.getSeverity())
                        .confidence(pattern.getConfidence())
                        .timeOverlap(true)
                        .altitudeOverlap(true)
                        .geometryOverlap(true)
                        .segmentIndex(nearest.segmentIndex)
                        .nearestDistanceNauticalMiles(nearest.distanceNauticalMiles)
                        .sourceRefs(new ArrayList<>(pattern.getSourceRefs()))
                        .rationale(label(pattern.getType()) + " intersects route segment " + nearest.segmentIndex
                                + " within " + Math.round(nearest.distanceNauticalMiles) + " NM")
                        .build());
            }
        }
        out.sort((left, right) -> severityRank(right.getSeverity()) - severityRank(left.getSeverity()));
        return out;
    }

    private List<WeatherPattern> patternsFromResult(OperationalFeedIngestResult result) {
        if (result == null || result.getEnvelope() == null) {
            return Collections.emptyList();
        }
        List<WeatherPattern> out = new ArrayList<>();
        OperationalFeedEnvelope envelope = result.getEnvelope();
        WeatherProductParseResult parsed = result.getWeatherProductResult();
        if (parsed != null && parsed.getProduct() != null) {
            out.add(fromProduct(parsed.getProduct(), envelope, result));
        }
        if (result.getPirepResult() != null && result.getPirepResult().getReport() != null) {
            out.add(fromPirep(result.getPirepResult().getReport(), envelope, result));
        }
        if (out.isEmpty() && envelope.getMetadata() != null && envelope.getMetadata().containsKey("geoJsonGeometry")) {
            out.add(fromEnvelope(envelope, result));
        }
        return out;
    }

    private WeatherPattern fromProduct(WeatherProduct product, OperationalFeedEnvelope envelope, OperationalFeedIngestResult result) {
        WeatherValidityWindow validity = product.getValidity();
        List<GeoCoordinate> geometry = new ArrayList<>(product.getGeometry());
        if (geometry.isEmpty()) {
            geometry.addAll(metadataGeometry(envelope));
        }
        List<String> refs = refs(envelope, product.getId());
        return WeatherPattern.builder()
                .id(value(product.getId(), envelope.getId()))
                .type(type(product, envelope))
                .productFamily(product.getType() == null ? metadata(envelope, "productFamily") : product.getType().name())
                .sourceFamily("WEATHER")
                .sourceProductId(product.getSourceProduct())
                .sourceUrl(metadata(envelope, "sourceUrl"))
                .rawText(value(product.getRawText(), envelope.getRawPayload()))
                .geometryIntent(geometryIntent(product, geometry))
                .issuedAt(product.getIssuedAt())
                .receivedAt(value(product.getReceivedAt(), envelope.getReceivedAt()))
                .validStart(validity == null ? null : validity.getValidStart())
                .validEnd(validity == null ? null : validity.getValidEnd())
                .forecastHour(product.getForecastHour())
                .lowerAltitudeFeet(product.getLowerAltitudeFeet())
                .upperAltitudeFeet(product.getUpperAltitudeFeet())
                .movementBearingDegrees(product.getMovement() == null ? null : product.getMovement().getBearingDegrees())
                .movementSpeedKnots(product.getMovement() == null ? null : product.getMovement().getSpeedNauticalMilesPerHour())
                .severity(severity(product.getHazard()))
                .confidence(product.confidenceValue() == 0.0 ? confidenceFromDiagnostics(result) : product.confidenceValue())
                .freshnessCategory(freshness(product.getIssuedAt(), validity))
                .geometry(geometry)
                .sourceRefs(refs)
                .diagnostics(diagnostics(result, product.diagnosticsAt(ZonedDateTime.now(ZoneOffset.UTC), Duration.ofMinutes(90)).stream()
                        .map(d -> d.getType() + ": " + d.getMessage())
                        .collect(Collectors.toList())))
                .rationale(rationale(type(product, envelope), product.getRawText()))
                .build();
    }

    private WeatherPattern fromPirep(PirepReport report, OperationalFeedEnvelope envelope, OperationalFeedIngestResult result) {
        List<GeoCoordinate> geometry = new ArrayList<>();
        if (report.getLocation() != null) {
            geometry.add(report.getLocation());
        }
        return WeatherPattern.builder()
                .id(value(report.getId(), envelope.getId()))
                .type(WeatherPatternType.PIREP_CLUSTER)
                .productFamily("PIREP")
                .sourceFamily("PIREP")
                .sourceProductId(report.getId())
                .sourceUrl(metadata(envelope, "sourceUrl"))
                .rawText(value(report.getRawText(), envelope.getRawPayload()))
                .geometryIntent(geometry.isEmpty() ? "NONE" : "POINT")
                .issuedAt(report.getObservationTime())
                .receivedAt(value(report.getReceivedTime(), envelope.getReceivedAt()))
                .validStart(report.getObservationTime())
                .validEnd(report.getObservationTime() == null ? null : report.getObservationTime().plusHours(1))
                .lowerAltitudeFeet(report.getAltitudeFeet())
                .upperAltitudeFeet(report.getAltitudeFeet())
                .severity(report.getIntensity() == null ? "UNKNOWN" : report.getIntensity().name())
                .confidence(report.getLocationQuality() == null ? 0.65 : Math.max(0.35, Math.min(0.95, report.getLocationQuality())))
                .freshnessCategory(freshness(report.getObservationTime(), null))
                .geometry(geometry)
                .sourceRefs(refs(envelope, report.getId()))
                .diagnostics(diagnostics(result, Collections.emptyList()))
                .rationale("PIREP " + value(String.valueOf(report.getPhenomenon()), "weather observation"))
                .build();
    }

    private WeatherPattern fromEnvelope(OperationalFeedEnvelope envelope, OperationalFeedIngestResult result) {
        List<GeoCoordinate> geometry = metadataGeometry(envelope);
        return WeatherPattern.builder()
                .id(envelope.getId())
                .type(type(envelope.getRawPayload(), metadata(envelope, "productFamily")))
                .productFamily(metadata(envelope, "productFamily"))
                .sourceFamily("WEATHER")
                .sourceProductId(metadata(envelope, "sourceRef"))
                .sourceUrl(metadata(envelope, "sourceUrl"))
                .rawText(envelope.getRawPayload())
                .geometryIntent(geometryIntent(null, geometry))
                .receivedAt(envelope.getReceivedAt())
                .confidence(confidenceFromDiagnostics(result))
                .freshnessCategory(freshness(envelope.getReceivedAt(), null))
                .geometry(geometry)
                .sourceRefs(refs(envelope, envelope.getId()))
                .diagnostics(diagnostics(result, Collections.emptyList()))
                .rationale("AWC retained weather pattern from GeoJSON metadata")
                .build();
    }

    private WeatherPatternType type(WeatherProduct product, OperationalFeedEnvelope envelope) {
        WeatherProductType productType = product == null ? null : product.getType();
        HazardousWeather hazard = product == null ? null : product.getHazard();
        String text = ((productType == null ? "" : productType.name()) + " "
                + (hazard == null || hazard.getType() == null ? "" : hazard.getType().name()) + " "
                + (product == null ? "" : product.getRawText()) + " "
                + metadata(envelope, "productFamily")).toUpperCase(Locale.US);
        return type(text, metadata(envelope, "productFamily"));
    }

    private WeatherPatternType type(String raw, String family) {
        String text = (value(raw, "") + " " + value(family, "")).toUpperCase(Locale.US);
        if (text.contains("PIREP") || text.contains("/OV")) return WeatherPatternType.PIREP_CLUSTER;
        if (text.contains("METAR") || text.contains("SPECI") || text.contains("TAF")) return WeatherPatternType.CEILING_VISIBILITY;
        if (text.contains("ASH") || text.contains("VOLCANIC")) return WeatherPatternType.VOLCANIC_ASH;
        if (text.contains("TURB")) return WeatherPatternType.TURBULENCE;
        if (text.contains("ICING") || text.contains(" ICE") || text.contains("/IC")) return WeatherPatternType.ICING;
        if (text.contains("SHEAR") || text.contains("LLWS")) return WeatherPatternType.WIND_SHEAR;
        if (text.contains("CONV") || text.contains("TS") || text.contains("TCF") || text.contains("CWA")) return WeatherPatternType.CONVECTION;
        if (text.contains("BKN") || text.contains("OVC") || text.contains("VIS")) {
            return WeatherPatternType.CEILING_VISIBILITY;
        }
        if (text.contains("RA") || text.contains("SN") || text.contains("PRECIP")) return WeatherPatternType.PRECIPITATION;
        return WeatherPatternType.GENERIC_ADVISORY;
    }

    private String geometryIntent(WeatherProduct product, List<GeoCoordinate> geometry) {
        if (product != null && (product.getType() == WeatherProductType.METAR || product.getType() == WeatherProductType.TAF)
                && (geometry == null || geometry.isEmpty())) {
            return "STATION_GUIDANCE";
        }
        if (geometry == null || geometry.isEmpty()) return "NONE";
        if (geometry.size() == 1) return "POINT";
        if (geometry.size() == 2) return "LINE_CORRIDOR";
        return "POLYGON";
    }

    private List<GeoCoordinate> metadataGeometry(OperationalFeedEnvelope envelope) {
        String raw = metadata(envelope, "geoJsonGeometry");
        if (raw == null) {
            return Collections.emptyList();
        }
        try {
            JsonNode geometry = mapper.readTree(raw);
            String type = geometry.path("type").asText();
            JsonNode coordinates = geometry.path("coordinates");
            List<GeoCoordinate> out = new ArrayList<>();
            if ("Point".equalsIgnoreCase(type)) {
                addPoint(out, coordinates);
            } else if ("LineString".equalsIgnoreCase(type)) {
                for (JsonNode point : coordinates) addPoint(out, point);
            } else if ("Polygon".equalsIgnoreCase(type) && coordinates.isArray() && coordinates.size() > 0) {
                for (JsonNode point : coordinates.get(0)) addPoint(out, point);
            }
            return out;
        } catch (Exception ignored) {
            return Collections.emptyList();
        }
    }

    private void addPoint(List<GeoCoordinate> out, JsonNode point) {
        if (point != null && point.isArray() && point.size() >= 2) {
            out.add(GeoCoordinate.builder()
                    .longitude(point.get(0).asDouble())
                    .latitude(point.get(1).asDouble())
                    .altitude(point.size() > 2 ? point.get(2).asDouble() : 0)
                    .build());
        }
    }

    private Nearest nearest(List<GeoCoordinate> route, List<GeoCoordinate> geometry) {
        if (geometry == null || geometry.isEmpty()) {
            return new Nearest(Double.POSITIVE_INFINITY, -1);
        }
        double best = Double.POSITIVE_INFINITY;
        int bestSegment = 0;
        for (int i = 0; i < route.size() - 1; i++) {
            GeoCoordinate a = route.get(i);
            GeoCoordinate b = route.get(i + 1);
            for (GeoCoordinate point : geometry) {
                double distance = Math.min(point.distanceTo(a), point.distanceTo(b));
                if (distance < best) {
                    best = distance;
                    bestSegment = i;
                }
            }
        }
        return new Nearest(best, bestSegment);
    }

    private boolean timeOverlap(WeatherPatternQuery query, WeatherPattern pattern) {
        if (query == null || query.getStartTime() == null || query.getEndTime() == null
                || pattern.getValidStart() == null || pattern.getValidEnd() == null) {
            return true;
        }
        return !pattern.getValidEnd().isBefore(query.getStartTime()) && !pattern.getValidStart().isAfter(query.getEndTime());
    }

    private boolean altitudeOverlap(WeatherPatternQuery query, WeatherPattern pattern) {
        if (query == null || query.getLowerAltitudeFeet() == null || query.getUpperAltitudeFeet() == null
                || pattern.getLowerAltitudeFeet() == null || pattern.getUpperAltitudeFeet() == null) {
            return true;
        }
        return pattern.getUpperAltitudeFeet() >= query.getLowerAltitudeFeet()
                && pattern.getLowerAltitudeFeet() <= query.getUpperAltitudeFeet();
    }

    private List<GeoCoordinate> route(List<List<Double>> route) {
        List<GeoCoordinate> out = new ArrayList<>();
        if (route != null) {
            for (List<Double> point : route) {
                if (point != null && point.size() >= 2) {
                    out.add(GeoCoordinate.builder()
                            .latitude(point.get(0))
                            .longitude(point.get(1))
                            .altitude(point.size() > 2 ? point.get(2) : 0)
                            .build());
                }
            }
        }
        return out;
    }

    private String severity(HazardousWeather hazard) {
        HazardSeverity severity = hazard == null ? null : hazard.getSeverity();
        return severity == null ? "UNKNOWN" : severity.name();
    }

    private double confidenceFromDiagnostics(OperationalFeedIngestResult result) {
        if (result == null) return 0.55;
        if (!result.getErrors().isEmpty()) return 0.25;
        if (!result.getWarnings().isEmpty()) return 0.55;
        return 0.72;
    }

    private String freshness(ZonedDateTime issuedAt, WeatherValidityWindow validity) {
        ZonedDateTime now = ZonedDateTime.now(ZoneOffset.UTC);
        if (validity != null && validity.getValidEnd() != null && validity.getValidEnd().isBefore(now)) {
            return "expired";
        }
        if (issuedAt == null) {
            return "unknown";
        }
        long minutes = Math.max(0, Duration.between(issuedAt, now).toMinutes());
        return minutes > 90 ? "stale" : minutes > 60 ? "aging" : "current";
    }

    private List<String> refs(OperationalFeedEnvelope envelope, String fallbackId) {
        Set<String> refs = new LinkedHashSet<>();
        String metadataRef = metadata(envelope, "sourceRef");
        if (metadataRef != null) refs.add(metadataRef);
        String family = metadata(envelope, "productFamily");
        refs.add(value(family, envelope == null || envelope.getType() == null ? "WEATHER" : envelope.getType().name()) + ":" + value(fallbackId, envelope == null ? UUID.randomUUID().toString() : envelope.getId()));
        return new ArrayList<>(refs);
    }

    private List<String> diagnostics(OperationalFeedIngestResult result, List<String> extra) {
        List<String> out = new ArrayList<>();
        if (result != null) {
            out.addAll(result.getWarnings());
            out.addAll(result.getErrors());
        }
        out.addAll(extra == null ? Collections.emptyList() : extra);
        return out;
    }

    private String metadata(OperationalFeedEnvelope envelope, String key) {
        return envelope == null || envelope.getMetadata() == null ? null : envelope.getMetadata().get(key);
    }

    private <T> T value(T value, T fallback) {
        return value == null ? fallback : value;
    }

    private String rationale(WeatherPatternType type, String raw) {
        return label(type) + " pattern normalized from retained aviation weather product"
                + (raw == null || raw.trim().isEmpty() ? "" : ": " + raw.substring(0, Math.min(raw.length(), 90)));
    }

    private String strongestSeverity(List<WeatherPattern> patterns) {
        return patterns.stream()
                .map(WeatherPattern::getSeverity)
                .max((a, b) -> severityRank(a) - severityRank(b))
                .orElse("UNKNOWN");
    }

    private int severityRank(String value) {
        String text = value == null ? "" : value.toUpperCase(Locale.US);
        if (text.contains("EXTREME")) return 5;
        if (text.contains("SEV") || text.contains("URGENT")) return 4;
        if (text.contains("MOD")) return 3;
        if (text.contains("LIGHT") || text.contains("LOW")) return 1;
        return 2;
    }

    private String label(WeatherPatternType type) {
        if (type == null) return "Generic Weather";
        String value = type.name().replace('_', ' ').toLowerCase(Locale.US);
        return Character.toUpperCase(value.charAt(0)) + value.substring(1);
    }

    private String stripFamily(String ref) {
        if (ref == null || !ref.contains(":")) return ref;
        return ref.substring(ref.indexOf(':') + 1);
    }

    private <T> List<T> safe(List<T> values) {
        return values == null ? Collections.emptyList() : values;
    }

    private static class Nearest {
        final double distanceNauticalMiles;
        final int segmentIndex;
        Nearest(double distanceNauticalMiles, int segmentIndex) {
            this.distanceNauticalMiles = distanceNauticalMiles;
            this.segmentIndex = segmentIndex;
        }
    }
}
