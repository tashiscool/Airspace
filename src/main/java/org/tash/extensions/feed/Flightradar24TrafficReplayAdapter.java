package org.tash.extensions.feed;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.tash.extensions.simulation.AircraftClass;
import org.tash.extensions.simulation.TrafficReplayAdapter;
import org.tash.extensions.simulation.TrafficReplayAirportDemand;
import org.tash.extensions.simulation.TrafficReplayBundle;
import org.tash.extensions.simulation.TrafficReplayFlightPlan;
import org.tash.extensions.simulation.TrafficReplayPosition;
import org.tash.extensions.simulation.TrafficReplaySectorDemand;

import java.net.URI;
import java.net.URLEncoder;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.nio.charset.StandardCharsets;
import java.time.Duration;
import java.time.Instant;
import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.time.format.DateTimeParseException;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

/**
 * Maps Flightradar24 flight-awareness API responses into Airspace's local
 * traffic replay contract. Live HTTP is disabled unless explicitly configured.
 */
public class Flightradar24TrafficReplayAdapter {
    private final boolean enabled;
    private final String apiKey;
    private final String baseUrl;
    private final String userAgent;
    private final int maxResults;
    private final HttpClient client;
    private final ObjectMapper mapper = new ObjectMapper().findAndRegisterModules();
    private final TrafficReplayAdapter replayAdapter = new TrafficReplayAdapter();

    public Flightradar24TrafficReplayAdapter(boolean enabled,
                                             String apiKey,
                                             String baseUrl,
                                             String userAgent,
                                             int maxResults) {
        this(enabled, apiKey, baseUrl, userAgent, maxResults, HttpClient.newBuilder()
                .connectTimeout(Duration.ofSeconds(5))
                .build());
    }

    Flightradar24TrafficReplayAdapter(boolean enabled,
                                      String apiKey,
                                      String baseUrl,
                                      String userAgent,
                                      int maxResults,
                                      HttpClient client) {
        this.enabled = enabled;
        this.apiKey = apiKey;
        this.baseUrl = blank(baseUrl) ? "https://fr24api.flightradar24.com/api" : trimTrailingSlash(baseUrl);
        this.userAgent = blank(userAgent) ? "Airspace-Flightradar24ReplayPrototype/1.0" : userAgent;
        this.maxResults = maxResults <= 0 ? 250 : maxResults;
        this.client = client;
    }

    public FlightAwarenessFeedPollResult poll(FlightAwarenessFeedPollRequest request) {
        FlightAwarenessFeedPollRequest safe = request == null ? new FlightAwarenessFeedPollRequest() : request;
        ZonedDateTime receivedAt = ZonedDateTime.now(ZoneOffset.UTC);
        List<String> diagnostics = new ArrayList<>();
        if (!blank(safe.getRawResponseJson())) {
            return fromRawJson(safe, safe.getRawResponseJson(), receivedAt, false, diagnostics);
        }
        if (!enabled) {
            diagnostics.add("Flightradar24 live polling is disabled by configuration.");
            diagnostics.add("Provide rawResponseJson for local replay import, or enable airspace.fr24.live.enabled with credentials and egress approval.");
            return result(false, true, false, sourceId(safe), "AUTHENTICATED_EXTERNAL_SETUP_REQUIRED", null, receivedAt, null, diagnostics);
        }
        if (blank(apiKey)) {
            diagnostics.add("Flightradar24 API key is not configured.");
            return result(false, true, false, sourceId(safe), "AUTHENTICATED_EXTERNAL_SETUP_REQUIRED", null, receivedAt, null, diagnostics);
        }
        if (!hasQueryFilter(safe)) {
            diagnostics.add("At least one bounded FR24 query filter is required: bounds, flights, callsigns, registrations, airports, routes, aircraft, altitudeRanges, or categories.");
            return result(false, false, false, sourceId(safe), "AUTHENTICATED_EXTERNAL", null, receivedAt, null, diagnostics);
        }
        String url = url(safe);
        try {
            HttpRequest http = HttpRequest.newBuilder(URI.create(url))
                    .timeout(Duration.ofSeconds(10))
                    .header("Accept", "application/json")
                    .header("Accept-Version", "v1")
                    .header("Authorization", "Bearer " + apiKey)
                    .header("User-Agent", userAgent)
                    .GET()
                    .build();
            HttpResponse<String> response = client.send(http, HttpResponse.BodyHandlers.ofString());
            if (response.statusCode() < 200 || response.statusCode() >= 300) {
                diagnostics.add("Flightradar24 API returned HTTP " + response.statusCode() + ".");
                return result(false, false, true, sourceId(safe), "AUTHENTICATED_EXTERNAL", receiptId(safe, receivedAt), receivedAt, null, diagnostics);
            }
            diagnostics.add("Flightradar24 live response mapped to TrafficReplayBundle; treat as non-authoritative flight-awareness data.");
            return fromRawJson(safe, response.body(), receivedAt, true, diagnostics);
        } catch (Exception ex) {
            diagnostics.add("Flightradar24 poll failed: " + ex.getMessage());
            return result(false, false, true, sourceId(safe), "AUTHENTICATED_EXTERNAL", receiptId(safe, receivedAt), receivedAt, null, diagnostics);
        }
    }

    public FlightAwarenessFeedPollResult fromRawJson(FlightAwarenessFeedPollRequest request,
                                                     String rawJson,
                                                     ZonedDateTime receivedAt,
                                                     boolean liveCallAttempted,
                                                     List<String> inheritedDiagnostics) {
        FlightAwarenessFeedPollRequest safe = request == null ? new FlightAwarenessFeedPollRequest() : request;
        List<String> diagnostics = new ArrayList<>(inheritedDiagnostics == null ? List.of() : inheritedDiagnostics);
        try {
            JsonNode root = mapper.readTree(rawJson);
            List<JsonNode> records = filterByRequestedTrackingSource(records(root), safe.getTrackingSourceTypes());
            if (records.isEmpty()) {
                diagnostics.add("Flightradar24 response contained no flight position or track records.");
            }
            TrafficReplayBundle bundle = bundleFromRecords(safe, records, receivedAt, liveCallAttempted, diagnostics);
            List<String> validation = replayAdapter.validate(bundle);
            diagnostics.addAll(validation);
            return result(validation.isEmpty(), false, liveCallAttempted, bundle.getSourceId(), bundle.getSourceMode(),
                    bundle.getProviderReceiptId(), receivedAt, bundle, diagnostics);
        } catch (Exception ex) {
            diagnostics.add("Malformed Flightradar24 response JSON: " + ex.getMessage());
            return result(false, false, liveCallAttempted, sourceId(safe), sourceMode(liveCallAttempted), receiptId(safe, receivedAt),
                    receivedAt, null, diagnostics);
        }
    }

    private TrafficReplayBundle bundleFromRecords(FlightAwarenessFeedPollRequest request,
                                                  List<JsonNode> records,
                                                  ZonedDateTime receivedAt,
                                                  boolean liveCallAttempted,
                                                  List<String> diagnostics) {
        String sourceId = sourceId(request);
        String receiptId = receiptId(request, receivedAt);
        List<TrafficReplayPosition> positions = positions(records);
        List<TrafficReplayFlightPlan> flightPlans = flightPlans(records, positions);
        List<TrafficReplayAirportDemand> airportDemand = airportDemand(records, positions, receivedAt);
        List<TrafficReplaySectorDemand> sectorDemand = sectorDemand(records, positions, receivedAt);
        List<String> trackingSources = trackingSourceTypes(records, positions);
        List<String> transports = providerTransports(liveCallAttempted);
        List<String> coverageWarnings = coverageWarnings(records, trackingSources, liveCallAttempted);
        if (!positions.isEmpty() && airportDemand.isEmpty()) {
            diagnostics.add("FR24 records lacked airport fields; generated FR24-AREA demand snapshot for simulator completeness.");
        }
        return TrafficReplayBundle.builder()
                .id("fr24-traffic-replay-" + receiptId)
                .sourceId(sourceId)
                .sourceMode(sourceMode(liveCallAttempted))
                .providerFamily("FLIGHTRADAR24")
                .generatedAt(receivedAt.toString())
                .timeBasis("FR24_TIMESTAMPS_TO_SIMULATION_OFFSET_MINUTES")
                .providerReceiptId(receiptId)
                .authorizationMode(liveCallAttempted ? "FR24_API_KEY_CONFIGURED" : "LOCAL_RECORDED_FR24_JSON")
                .sourceRefs(List.of(
                        "FR24_API:" + endpointMode(request),
                        "FR24_PROVIDER:" + sourceId,
                        liveCallAttempted ? "FR24_EGRESS:LIVE_HTTP" : "FR24_EGRESS:NONE_LOCAL_REPLAY"))
                .providerTransports(transports)
                .trackingSourceTypes(trackingSources)
                .coverageWarnings(coverageWarnings)
                .flightPlans(flightPlans)
                .positions(positions)
                .airportDemand(airportDemand)
                .sectorDemand(sectorDemand)
                .trafficManagementInitiatives(List.of())
                .assumptions(List.of(
                        "Flightradar24 data enriches Airspace simulation/replay and is not authoritative FAA SWIM/TFMS/NAS truth.",
                        "Derived airport and sector demand snapshots are approximate unless supplied by an authorized traffic-flow provider.",
                        "Provider receipts, query filters, freshness, source mode, and non-authoritative warnings must remain attached."))
                .diagnostics(diagnostics)
                .build();
    }

    private List<JsonNode> records(JsonNode root) {
        List<JsonNode> out = new ArrayList<>();
        collectRecords(root, out);
        return out;
    }

    private void collectRecords(JsonNode node, List<JsonNode> out) {
        if (node == null || node.isMissingNode() || node.isNull()) {
            return;
        }
        if (node.isArray()) {
            node.forEach(item -> collectRecords(item, out));
            return;
        }
        if (node.has("data") && node.path("data").isArray()) {
            collectRecords(node.path("data"), out);
            return;
        }
        if (node.has("tracks") && node.path("tracks").isArray()) {
            String fr24Id = text(node, "fr24_id", "flight_id", "id");
            for (JsonNode track : node.path("tracks")) {
                Map<String, JsonNode> fields = new LinkedHashMap<>();
                node.fields().forEachRemaining(entry -> {
                    if (!"tracks".equals(entry.getKey())) {
                        fields.put(entry.getKey(), entry.getValue());
                    }
                });
                track.fields().forEachRemaining(entry -> fields.put(entry.getKey(), entry.getValue()));
                if (!blank(fr24Id)) {
                    fields.put("fr24_id", mapper.getNodeFactory().textNode(fr24Id));
                }
                out.add(mapper.valueToTree(fields));
            }
            return;
        }
        if (node.has("lat") || node.has("latitude") || node.has("lon") || node.has("longitude")) {
            out.add(node);
        }
    }

    private List<TrafficReplayPosition> positions(List<JsonNode> records) {
        List<Instant> instants = records.stream()
                .map(this::instant)
                .filter(instant -> instant != null)
                .sorted()
                .toList();
        Instant base = instants.isEmpty() ? null : instants.get(0);
        List<TrafficReplayPosition> out = new ArrayList<>();
        int index = 0;
        for (JsonNode record : records) {
            String flightId = flightId(record, index);
            Instant instant = instant(record);
            int offset = base == null || instant == null ? index : (int) Math.max(0, Duration.between(base, instant).toMinutes());
            out.add(TrafficReplayPosition.builder()
                    .flightId(flightId)
                    .timestamp(instant == null ? null : instant.toString())
                    .offsetMinutes(offset)
                    .latitude(number(record, "lat", "latitude"))
                    .longitude(number(record, "lon", "lng", "longitude"))
                    .altitudeFeet(number(record, "alt", "altitude", "altitudeFeet"))
                    .groundSpeedKnots(number(record, "gspeed", "speed", "groundSpeedKnots"))
                    .headingDegrees(number(record, "track", "heading", "headingDegrees"))
                    .routeProgress(routeProgress(records, record, index))
                    .delaySeconds(0)
                    .phase(phase(record))
                    .trackingSourceType(trackingSourceType(record).name())
                    .trackingSourceLabel(trackingSourceLabel(record))
                    .providerTrackId(text(record, "fr24_id", "flight_id", "id", "hex"))
                    .surveillanceQuality(surveillanceQuality(record))
                    .sourceRefs(List.of(
                            "FR24:position:" + flightId + ":T+" + offset,
                            "FR24:tracking-source:" + trackingSourceType(record).name()))
                    .build());
            index++;
        }
        return out;
    }

    private List<JsonNode> filterByRequestedTrackingSource(List<JsonNode> records, List<String> requestedTypes) {
        if (requestedTypes == null || requestedTypes.isEmpty()) {
            return records;
        }
        Set<String> requested = new LinkedHashSet<>();
        for (String requestedType : requestedTypes) {
            if (!blank(requestedType)) {
                requested.add(FlightTrackingSourceNormalizer.normalize(requestedType).name());
            }
        }
        if (requested.isEmpty()) {
            return records;
        }
        return records.stream()
                .filter(record -> requested.contains(trackingSourceType(record).name()))
                .toList();
    }

    private List<TrafficReplayFlightPlan> flightPlans(List<JsonNode> records, List<TrafficReplayPosition> positions) {
        Map<String, List<JsonNode>> grouped = new LinkedHashMap<>();
        int index = 0;
        for (JsonNode record : records) {
            grouped.computeIfAbsent(flightId(record, index++), ignored -> new ArrayList<>()).add(record);
        }
        List<TrafficReplayFlightPlan> out = new ArrayList<>();
        for (Map.Entry<String, List<JsonNode>> entry : grouped.entrySet()) {
            JsonNode first = entry.getValue().get(0);
            List<List<Double>> route = positions.stream()
                    .filter(position -> entry.getKey().equals(position.getFlightId()))
                    .sorted(Comparator.comparingInt(TrafficReplayPosition::getOffsetMinutes))
                    .map(position -> List.of(position.getLatitude(), position.getLongitude(), position.getAltitudeFeet()))
                    .toList();
            out.add(TrafficReplayFlightPlan.builder()
                    .flightId(entry.getKey())
                    .callsign(value(text(first, "callsign", "flight"), entry.getKey()))
                    .aircraftClass(aircraftClass(text(first, "type", "aircraft")))
                    .operator(value(text(first, "operating_as", "painted_as", "operator"), "FR24_TRACKED"))
                    .origin(value(text(first, "orig_icao", "orig_iata", "origin"), "UNKNOWN"))
                    .destination(value(text(first, "dest_icao", "dest_iata", "destination"), "UNKNOWN"))
                    .filedRouteText(routeText(first))
                    .filedRoutePoints(route)
                    .scheduledDepartureTime(text(first, "datetime_takeoff", "scheduledDepartureTime", "first_seen"))
                    .scheduledArrivalTime(text(first, "eta", "datetime_landed", "scheduledArrivalTime"))
                    .estimatedArrivalTime(text(first, "eta"))
                    .requestedAltitudeFeet(maxAltitude(entry.getValue()))
                    .filedSpeedKnots((int) Math.round(maxSpeed(entry.getValue())))
                    .sourceRefs(List.of("FR24:flight:" + entry.getKey()))
                    .build());
        }
        return out;
    }

    private List<TrafficReplayAirportDemand> airportDemand(List<JsonNode> records,
                                                           List<TrafficReplayPosition> positions,
                                                           ZonedDateTime receivedAt) {
        Map<String, Integer> counts = new LinkedHashMap<>();
        for (JsonNode record : records) {
            String airport = text(record, "orig_icao", "orig_iata", "airport", "origin");
            if (!blank(airport)) {
                counts.merge(airport.toUpperCase(Locale.US), 1, Integer::sum);
            }
        }
        if (counts.isEmpty() && !positions.isEmpty()) {
            counts.put("FR24-AREA", positions.size());
        }
        List<TrafficReplayAirportDemand> out = new ArrayList<>();
        for (Map.Entry<String, Integer> entry : counts.entrySet()) {
            int count = entry.getValue();
            out.add(TrafficReplayAirportDemand.builder()
                    .airportId(entry.getKey())
                    .timestamp(receivedAt.toString())
                    .offsetMinutes(0)
                    .departureDemandPerHour(count)
                    .arrivalDemandPerHour(count)
                    .departureCapacityPerHour(Math.max(count, count * 2))
                    .arrivalCapacityPerHour(Math.max(count, count * 2))
                    .departureQueueDepth(0)
                    .arrivalQueueDepth(0)
                    .averageDelaySeconds(0)
                    .runwayConfiguration("FR24_DERIVED_TRAFFIC_CONTEXT")
                    .sourceRefs(List.of("FR24:derived-airport-demand:" + entry.getKey()))
                    .build());
        }
        return out;
    }

    private List<TrafficReplaySectorDemand> sectorDemand(List<JsonNode> records,
                                                         List<TrafficReplayPosition> positions,
                                                         ZonedDateTime receivedAt) {
        int active = positions.isEmpty() ? records.size() : positions.size();
        if (active <= 0) {
            return List.of();
        }
        return List.of(TrafficReplaySectorDemand.builder()
                .sectorId("FR24-QUERY-AREA")
                .timestamp(receivedAt.toString())
                .offsetMinutes(0)
                .activeAircraft(active)
                .baselineCapacity(Math.max(1, (int) Math.ceil(active * 1.5)))
                .handoffQueueDepth(0)
                .frequencyUtilization(Math.min(0.95, active / (double) Math.max(1, (int) Math.ceil(active * 1.5))))
                .estimatedHandoffDelaySeconds(0)
                .sourceRefs(List.of("FR24:derived-sector-demand:query-area"))
                .build());
    }

    private String url(FlightAwarenessFeedPollRequest request) {
        String endpoint = endpoint(request);
        StringBuilder out = new StringBuilder(baseUrl).append(endpoint);
        Map<String, String> params = params(request);
        if (!params.isEmpty()) {
            out.append('?');
            boolean first = true;
            for (Map.Entry<String, String> entry : params.entrySet()) {
                if (!first) {
                    out.append('&');
                }
                out.append(encode(entry.getKey())).append('=').append(encode(entry.getValue()));
                first = false;
            }
        }
        return out.toString();
    }

    private String endpoint(FlightAwarenessFeedPollRequest request) {
        String mode = normalize(request.getMode());
        boolean historic = request.getTimestamp() != null || mode.contains("HISTORIC");
        boolean full = request.isFull();
        if (historic) {
            return full ? "/historic/flight-positions/full" : "/historic/flight-positions/light";
        }
        return full ? "/live/flight-positions/full" : "/live/flight-positions/light";
    }

    private Map<String, String> params(FlightAwarenessFeedPollRequest request) {
        Map<String, String> params = new LinkedHashMap<>();
        put(params, "bounds", request.getBounds());
        put(params, "flights", joined(request.getFlights()));
        put(params, "callsigns", joined(request.getCallsigns()));
        put(params, "registrations", joined(request.getRegistrations()));
        put(params, "airports", joined(request.getAirports()));
        put(params, "routes", joined(request.getRoutes()));
        put(params, "aircraft", joined(request.getAircraft()));
        put(params, "altitude_ranges", joined(request.getAltitudeRanges()));
        put(params, "categories", joined(request.getCategories()));
        if (request.getTimestamp() != null) {
            params.put("timestamp", String.valueOf(request.getTimestamp()));
        }
        params.put("limit", String.valueOf(Math.min(value(request.getMaxResults(), maxResults), maxResults)));
        return params;
    }

    private boolean hasQueryFilter(FlightAwarenessFeedPollRequest request) {
        return !blank(request.getBounds())
                || !empty(request.getFlights())
                || !empty(request.getCallsigns())
                || !empty(request.getRegistrations())
                || !empty(request.getAirports())
                || !empty(request.getRoutes())
                || !empty(request.getAircraft())
                || !empty(request.getAltitudeRanges())
                || !empty(request.getCategories());
    }

    private String endpointMode(FlightAwarenessFeedPollRequest request) {
        String mode = endpoint(request).replace("/", ":");
        return mode.startsWith(":") ? mode.substring(1) : mode;
    }

    private String flightId(JsonNode record, int index) {
        return value(text(record, "fr24_id", "flight_id", "id", "hex", "flight", "callsign"), "fr24-record-" + index);
    }

    private double routeProgress(List<JsonNode> all, JsonNode record, int index) {
        String id = flightId(record, index);
        long total = 0;
        long beforeOrAt = 0;
        for (int i = 0; i < all.size(); i++) {
            if (flightId(all.get(i), i).equals(id)) {
                total++;
                if (i <= index) {
                    beforeOrAt++;
                }
            }
        }
        if (total <= 1) {
            return 0;
        }
        return Math.min(1.0, Math.max(0.0, (beforeOrAt - 1) / (double) (total - 1)));
    }

    private String phase(JsonNode record) {
        double altitude = number(record, "alt", "altitude", "altitudeFeet");
        double speed = number(record, "gspeed", "speed", "groundSpeedKnots");
        if (altitude <= 1000 && speed <= 60) {
            return "SURFACE";
        }
        if (altitude < 18000 && speed > 60) {
            return "CLIMB_OR_DESCENT";
        }
        return "ENROUTE";
    }

    private FlightTrackingSourceType trackingSourceType(JsonNode record) {
        return FlightTrackingSourceNormalizer.normalize(text(record, "source", "tracking_source", "trackingSource", "surveillance_source"));
    }

    private String trackingSourceLabel(JsonNode record) {
        return FlightTrackingSourceNormalizer.label(text(record, "source", "tracking_source", "trackingSource", "surveillance_source"));
    }

    private String surveillanceQuality(JsonNode record) {
        String source = trackingSourceType(record).name();
        if (source.equals(FlightTrackingSourceType.ADS_B.name())) {
            return "HIGH_POSITION_RATE_DEPENDS_ON_AIRCRAFT_GNSS";
        }
        if (source.equals(FlightTrackingSourceType.SATELLITE_ADSB.name())) {
            return "GLOBAL_COVERAGE_VARIABLE_LATENCY";
        }
        if (source.equals(FlightTrackingSourceType.MLAT.name())) {
            return "DERIVED_FROM_RECEIVER_TIMING";
        }
        if (source.equals(FlightTrackingSourceType.RADAR.name()) || source.equals(FlightTrackingSourceType.FAA_SWIM.name())) {
            return "OPERATIONAL_PROVIDER_DEPENDENT";
        }
        if (source.equals(FlightTrackingSourceType.UNKNOWN.name())) {
            return "SOURCE_UNSPECIFIED_REVIEW";
        }
        return "PROVIDER_DEPENDENT";
    }

    private List<String> trackingSourceTypes(List<JsonNode> records, List<TrafficReplayPosition> positions) {
        Set<String> out = new LinkedHashSet<>();
        for (TrafficReplayPosition position : positions) {
            if (!blank(position.getTrackingSourceType())) {
                out.add(position.getTrackingSourceType());
            }
        }
        if (out.isEmpty() && !records.isEmpty()) {
            out.add(FlightTrackingSourceType.UNKNOWN.name());
        }
        return new ArrayList<>(out);
    }

    private List<String> providerTransports(boolean liveCallAttempted) {
        List<String> transports = new ArrayList<>();
        transports.add(liveCallAttempted ? "REST_LIVE_HTTP" : "RECORDED_JSON");
        transports.add("SDK_COMPATIBLE_SHAPE");
        transports.add("MCP_COMPATIBLE_SHAPE");
        return transports;
    }

    private List<String> coverageWarnings(List<JsonNode> records, List<String> trackingSources, boolean liveCallAttempted) {
        List<String> warnings = new ArrayList<>();
        if (!liveCallAttempted) {
            warnings.add("Recorded replay sample; no live aircraft position query was performed.");
        }
        if (trackingSources.contains(FlightTrackingSourceType.UNKNOWN.name())) {
            warnings.add("At least one position lacked explicit tracking-source provenance.");
        }
        if (trackingSources.contains(FlightTrackingSourceType.MLAT.name())) {
            warnings.add("MLAT positions are derived from receiver timing and should retain provider quality metadata.");
        }
        if (trackingSources.contains(FlightTrackingSourceType.SATELLITE_ADSB.name())) {
            warnings.add("Satellite ADS-B coverage and latency can vary by provider and orbital availability.");
        }
        if (!records.isEmpty() && records.stream().anyMatch(record -> blank(text(record, "timestamp", "time", "first_seen", "last_seen")))) {
            warnings.add("At least one position lacked a timestamp; simulation offsets may be approximate.");
        }
        return warnings;
    }

    private Integer maxAltitude(List<JsonNode> records) {
        return records.stream()
                .mapToInt(record -> (int) Math.round(number(record, "alt", "altitude", "altitudeFeet")))
                .max()
                .orElse(0);
    }

    private double maxSpeed(List<JsonNode> records) {
        return records.stream()
                .mapToDouble(record -> number(record, "gspeed", "speed", "groundSpeedKnots"))
                .max()
                .orElse(0);
    }

    private AircraftClass aircraftClass(String type) {
        String value = normalize(type);
        if (value.contains("KC") || value.contains("C17") || value.contains("C-17") || value.contains("MIL")) {
            return AircraftClass.MILITARY_REFUELING;
        }
        if (value.contains("A388") || value.contains("A35") || value.contains("A34")
                || value.contains("B74") || value.contains("B77") || value.contains("B78")) {
            return AircraftClass.HEAVY_JET;
        }
        if (value.contains("CRJ") || value.contains("E17") || value.contains("E19")) {
            return AircraftClass.REGIONAL_JET;
        }
        if (value.contains("DH8") || value.contains("AT7") || value.contains("BE")) {
            return AircraftClass.TURBOPROP;
        }
        if (!blank(type)) {
            return AircraftClass.NARROWBODY;
        }
        return AircraftClass.GENERIC;
    }

    private Instant instant(JsonNode record) {
        String timestamp = text(record, "timestamp", "time", "first_seen", "last_seen");
        if (blank(timestamp)) {
            return null;
        }
        try {
            return Instant.parse(timestamp);
        } catch (DateTimeParseException ignored) {
            return null;
        }
    }

    private double number(JsonNode node, String... keys) {
        for (String key : keys) {
            JsonNode value = node.path(key);
            if (!value.isMissingNode() && !value.isNull()) {
                if (value.isNumber()) {
                    return value.asDouble();
                }
                try {
                    return Double.parseDouble(value.asText());
                } catch (NumberFormatException ignored) {
                    return 0;
                }
            }
        }
        return 0;
    }

    private String text(JsonNode node, String... keys) {
        for (String key : keys) {
            JsonNode value = node.path(key);
            if (!value.isMissingNode() && !value.isNull() && !blank(value.asText())) {
                return value.asText();
            }
        }
        return null;
    }

    private String routeText(JsonNode record) {
        String origin = text(record, "orig_icao", "orig_iata", "origin");
        String destination = text(record, "dest_icao", "dest_iata", "destination");
        if (blank(origin) && blank(destination)) {
            return "FR24 TRACK";
        }
        return value(origin, "UNKNOWN") + " DCT " + value(destination, "UNKNOWN");
    }

    private FlightAwarenessFeedPollResult result(boolean accepted,
                                                boolean setupRequired,
                                                boolean liveCallAttempted,
                                                String sourceId,
                                                String sourceMode,
                                                String providerReceiptId,
                                                ZonedDateTime receivedAt,
                                                TrafficReplayBundle bundle,
                                                List<String> diagnostics) {
        return FlightAwarenessFeedPollResult.builder()
                .accepted(accepted)
                .setupRequired(setupRequired)
                .liveCallAttempted(liveCallAttempted)
                .sourceId(sourceId)
                .sourceMode(sourceMode)
                .providerReceiptId(providerReceiptId)
                .receivedAt(receivedAt)
                .trafficReplay(bundle)
                .providerTransports(bundle == null ? List.of() : bundle.getProviderTransports())
                .trackingSourceTypes(bundle == null ? List.of() : bundle.getTrackingSourceTypes())
                .coverageWarnings(bundle == null ? List.of() : bundle.getCoverageWarnings())
                .diagnostics(diagnostics)
                .build();
    }

    private String sourceMode(boolean liveCallAttempted) {
        return liveCallAttempted ? "AUTHENTICATED_EXTERNAL" : "AUTHENTICATED_EXTERNAL_REPLAY_SAMPLE";
    }

    private String sourceId(FlightAwarenessFeedPollRequest request) {
        return value(request.getSourceId(), "flightradar24-api");
    }

    private String receiptId(FlightAwarenessFeedPollRequest request, ZonedDateTime receivedAt) {
        return "fr24-" + sourceId(request) + "-" + receivedAt.toInstant().toEpochMilli();
    }

    private void put(Map<String, String> params, String key, String value) {
        if (!blank(value)) {
            params.put(key, value);
        }
    }

    private String joined(List<String> values) {
        if (values == null || values.isEmpty()) {
            return null;
        }
        Set<String> cleaned = new LinkedHashSet<>();
        for (String value : values) {
            if (!blank(value)) {
                cleaned.add(value.trim());
            }
        }
        return cleaned.isEmpty() ? null : String.join(",", cleaned);
    }

    private boolean empty(List<String> values) {
        return joined(values) == null;
    }

    private String encode(String value) {
        return URLEncoder.encode(value, StandardCharsets.UTF_8);
    }

    private String normalize(String value) {
        return value == null ? "" : value.trim().toUpperCase(Locale.US);
    }

    private String trimTrailingSlash(String value) {
        String out = value.trim();
        while (out.endsWith("/")) {
            out = out.substring(0, out.length() - 1);
        }
        return out;
    }

    private boolean blank(String value) {
        return value == null || value.trim().isEmpty();
    }

    private String value(String value, String fallback) {
        return blank(value) ? fallback : value;
    }

    private int value(Integer value, int fallback) {
        return value == null || value <= 0 ? fallback : value;
    }
}
