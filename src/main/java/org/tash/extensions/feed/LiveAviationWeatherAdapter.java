package org.tash.extensions.feed;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.net.URI;
import java.net.URLEncoder;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.charset.StandardCharsets;
import java.time.Duration;
import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.UUID;

/**
 * Dependency-light AWC Data API adapter. It retains source URL, GeoJSON geometry,
 * raw feature properties, and aviation text in feed envelope metadata.
 */
public class LiveAviationWeatherAdapter implements WeatherFeedAdapter {
    private static final List<String> DEFAULT_PRODUCTS =
            Arrays.asList("metar", "taf", "pirep", "airsigmet", "gairmet", "cwa", "tcf");

    private final boolean enabled;
    private final String baseUrl;
    private final String userAgent;
    private final int maxResults;
    private final HttpClient client;
    private final ObjectMapper mapper = new ObjectMapper();

    public LiveAviationWeatherAdapter(boolean enabled, String baseUrl, String userAgent, int maxResults) {
        this(enabled, baseUrl, userAgent, maxResults, HttpClient.newBuilder()
                .connectTimeout(Duration.ofSeconds(5))
                .build());
    }

    LiveAviationWeatherAdapter(boolean enabled, String baseUrl, String userAgent, int maxResults, HttpClient client) {
        this.enabled = enabled;
        this.baseUrl = blank(baseUrl) ? "https://aviationweather.gov/api/data" : trimTrailingSlash(baseUrl);
        this.userAgent = blank(userAgent) ? "Airspace-WeatherPatternPrototype/1.0" : userAgent;
        this.maxResults = maxResults <= 0 ? 250 : maxResults;
        this.client = client;
    }

    @Override
    public OperationalFeedPollResult poll() {
        WeatherFeedPollRequest request = new WeatherFeedPollRequest();
        request.setProducts(DEFAULT_PRODUCTS);
        request.setMaxResults(maxResults);
        return poll(request).getPollResult();
    }

    @Override
    public WeatherFeedBatch poll(WeatherFeedPollRequest request) {
        List<OperationalFeedEnvelope> envelopes = new ArrayList<>();
        List<String> diagnostics = new ArrayList<>();
        if (!enabled) {
            diagnostics.add("Live aviation weather polling is disabled by configuration");
            OperationalFeedPollResult poll = OperationalFeedPollResult.builder()
                    .accepted(false)
                    .diagnostics(diagnostics)
                    .build();
            return WeatherFeedBatch.builder()
                    .accepted(false)
                    .sourceId("awc-live")
                    .receivedAt(ZonedDateTime.now(ZoneOffset.UTC))
                    .pollResult(poll)
                    .diagnostics(diagnostics)
                    .build();
        }
        List<String> products = request == null || request.getProducts() == null || request.getProducts().isEmpty()
                ? DEFAULT_PRODUCTS
                : request.getProducts();
        int limit = request == null || request.getMaxResults() == null || request.getMaxResults() <= 0
                ? maxResults
                : Math.min(request.getMaxResults(), maxResults);
        for (String product : products) {
            String normalized = product == null ? "" : product.trim().toLowerCase(Locale.US);
            if (normalized.isEmpty()) {
                continue;
            }
            String url = url(normalized, request);
            try {
                HttpRequest http = HttpRequest.newBuilder(URI.create(url))
                        .timeout(Duration.ofSeconds(10))
                        .header("Accept", "application/geo+json, application/json, text/plain")
                        .header("User-Agent", userAgent)
                        .GET()
                        .build();
                HttpResponse<String> response = client.send(http, HttpResponse.BodyHandlers.ofString());
                if (response.statusCode() < 200 || response.statusCode() >= 300) {
                    diagnostics.add("AWC " + normalized + " returned HTTP " + response.statusCode());
                    continue;
                }
                envelopes.addAll(envelopes(normalized, url, response.body(), limit - envelopes.size()));
                if (envelopes.size() >= limit) {
                    break;
                }
            } catch (Exception ex) {
                diagnostics.add("AWC " + normalized + " poll failed: " + ex.getMessage());
            }
        }
        OperationalFeedPollResult poll = OperationalFeedPollResult.builder()
                .accepted(!envelopes.isEmpty() && diagnostics.stream().noneMatch(d -> d.contains("failed")))
                .envelopes(envelopes)
                .diagnostics(diagnostics)
                .build();
        return WeatherFeedBatch.builder()
                .accepted(poll.isAccepted())
                .sourceId("awc-live")
                .receivedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .pollResult(poll)
                .diagnostics(diagnostics)
                .build();
    }

    private String url(String product, WeatherFeedPollRequest request) {
        StringBuilder out = new StringBuilder(baseUrl).append('/').append(URLEncoder.encode(product, StandardCharsets.UTF_8));
        out.append("?format=geojson");
        if (request != null && request.getHoursBeforeNow() != null && request.getHoursBeforeNow() > 0) {
            out.append("&hours=").append(Math.min(request.getHoursBeforeNow(), 24));
        }
        return out.toString();
    }

    private List<OperationalFeedEnvelope> envelopes(String product, String sourceUrl, String body, int remaining) throws Exception {
        List<OperationalFeedEnvelope> values = new ArrayList<>();
        if (remaining <= 0) {
            return values;
        }
        JsonNode root = mapper.readTree(body);
        if ("FeatureCollection".equalsIgnoreCase(root.path("type").asText()) && root.path("features").isArray()) {
            int count = 0;
            for (JsonNode feature : root.path("features")) {
                if (count++ >= remaining) {
                    break;
                }
                values.add(envelope(product, sourceUrl, feature, body));
            }
        } else {
            values.add(envelope(product, sourceUrl, root, body));
        }
        return values;
    }

    private OperationalFeedEnvelope envelope(String product, String sourceUrl, JsonNode node, String fallbackRaw) throws Exception {
        JsonNode properties = node.path("properties").isMissingNode() ? node : node.path("properties");
        JsonNode geometry = node.path("geometry");
        String raw = firstText(properties,
                "rawOb", "rawTaf", "rawAirSigmet", "rawCWA", "rawText", "text", "description", "message");
        if (blank(raw)) {
            raw = fallbackRaw.length() > 4000 ? fallbackRaw.substring(0, 4000) : fallbackRaw;
        }
        Map<String, String> metadata = new LinkedHashMap<>();
        metadata.put("sourceUrl", sourceUrl);
        metadata.put("awcProduct", product.toUpperCase(Locale.US));
        metadata.put("productFamily", product.toUpperCase(Locale.US));
        metadata.put("sourceRef", "AWC:" + value(firstText(properties, "id", "airSigmetId", "sigmetId", "cwaId", "icaoId"), UUID.randomUUID().toString()));
        if (!geometry.isMissingNode() && !geometry.isNull()) {
            metadata.put("geoJsonGeometry", mapper.writeValueAsString(geometry));
        }
        metadata.put("propertiesJson", mapper.writeValueAsString(properties));
        metadata.put("rawProductJson", mapper.writeValueAsString(node));
        return OperationalFeedEnvelope.builder()
                .id(UUID.randomUUID().toString())
                .sourceId("awc-live")
                .type(type(product, raw))
                .receivedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .rawPayload(raw)
                .metadata(metadata)
                .build();
    }

    private OperationalFeedType type(String product, String raw) {
        String text = (product + " " + raw).toUpperCase(Locale.US);
        if (text.contains("PIREP") || text.contains("/OV") || "PIREP".equalsIgnoreCase(product)) {
            return OperationalFeedType.PIREP;
        }
        return OperationalFeedType.WEATHER;
    }

    private String firstText(JsonNode node, String... keys) {
        for (String key : keys) {
            JsonNode value = node.path(key);
            if (!value.isMissingNode() && !value.isNull() && !blank(value.asText())) {
                return value.asText();
            }
        }
        return null;
    }

    private String value(String value, String fallback) {
        return blank(value) ? fallback : value;
    }

    private boolean blank(String value) {
        return value == null || value.trim().isEmpty();
    }

    private String trimTrailingSlash(String value) {
        String out = value.trim();
        while (out.endsWith("/")) {
            out = out.substring(0, out.length() - 1);
        }
        return out;
    }
}
