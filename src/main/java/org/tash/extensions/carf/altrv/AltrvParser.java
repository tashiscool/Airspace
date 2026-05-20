package org.tash.extensions.carf.altrv;

import org.tash.data.GeoCoordinate;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.time.LocalDateTime;
import java.time.Month;
import java.time.ZoneId;
import java.time.ZonedDateTime;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class AltrvParser {
    private static final ZoneId DEFAULT_ZONE = ZoneId.of("America/New_York");
    private static final Pattern FIELD = Pattern.compile("(?m)^([A-G])\\.[ \\t]*(.*?)(?=\\n\\s*[A-G]\\.[ \\t]|\\z)",
            Pattern.DOTALL);
    private static final Pattern COORDINATE_POINT = Pattern.compile("(\\d{4}[NS])\\s+(\\d{4,5}[EW])\\s+(\\d{4})",
            Pattern.CASE_INSENSITIVE);
    private static final Pattern NAMED_POINT = Pattern.compile("\\b([A-Z][A-Z0-9]{1,7})(?:\\s+(\\d{3})/(\\d{3}))?\\s+(\\d{4})\\b");
    private static final Pattern FLIGHT_LEVEL = Pattern.compile("\\bFL\\d{3}B\\d{3}\\b", Pattern.CASE_INSENSITIVE);
    private static final Pattern ETD = Pattern.compile("\\bETD\\s+(\\d{2})(\\d{2})(\\d{2})\\s+([A-Z]{3})\\s+(\\d{4})",
            Pattern.CASE_INSENSITIVE);
    private static final Pattern AVANA = Pattern.compile("\\bAVANA\\s+(\\d{2})(\\d{2})(\\d{2})\\b", Pattern.CASE_INSENSITIVE);

    public AltrvParseResult parse(String raw) {
        String normalized = raw == null ? "" : raw.replace("\r", "");
        List<String> diagnostics = new ArrayList<>();
        if (normalized.trim().isEmpty()) {
            diagnostics.add("ALTRV message is empty");
            return result(false, null, diagnostics);
        }

        Map<String, String> sections = sections(normalized);
        Map<String, String> stationarySections = stationarySections(normalized);
        List<AltrvSectionResult> sectionResults = sectionResults(normalized, sections, stationarySections);
        List<AltrvDiagnostic> typedDiagnostics = new ArrayList<>();
        boolean stationaryOnly = !stationarySections.isEmpty() && sections.isEmpty();
        if (!sections.containsKey("A")) {
            addDiagnostic(diagnostics, typedDiagnostics, AltrvDiagnosticSeverity.ERROR, "A",
                    "Missing ALTRV section A activity name", spanOf(normalized, "A"));
        }
        if (!sections.containsKey("D")) {
            addDiagnostic(diagnostics, typedDiagnostics, AltrvDiagnosticSeverity.ERROR, "D",
                    "Missing ALTRV section D route text", spanOf(normalized, "D"));
        }
        if (!sections.containsKey("F")) {
            addDiagnostic(diagnostics, typedDiagnostics, AltrvDiagnosticSeverity.ERROR, "F",
                    "Missing ALTRV section F timing text", spanOf(normalized, "F"));
        }
        if (!sections.containsKey("G")) {
            addDiagnostic(diagnostics, typedDiagnostics, AltrvDiagnosticSeverity.ERROR, "G",
                    "Missing ALTRV section G aircraft text", spanOf(normalized, "G"));
        }
        if (stationaryOnly) {
            diagnostics.removeIf(d -> d.startsWith("Missing ALTRV section A")
                    || d.startsWith("Missing ALTRV section D")
                    || d.startsWith("Missing ALTRV section F")
                    || d.startsWith("Missing ALTRV section G"));
            typedDiagnostics.removeIf(d -> d.getMessage().startsWith("Missing ALTRV section A")
                    || d.getMessage().startsWith("Missing ALTRV section D")
                    || d.getMessage().startsWith("Missing ALTRV section F")
                    || d.getMessage().startsWith("Missing ALTRV section G"));
        }

        String routeText = sections.getOrDefault("D", "");
        List<AltrvRouteEvent> events = detectEvents(normalized);
        List<AltrvRoutePoint> points = routePoints(routeText, diagnostics);
        List<AltrvArea> areas = areas(normalized);
        AltrvStationaryReservation stationaryReservation = stationarySections.isEmpty() ? null
                : AltrvStationaryReservation.builder()
                .reservationText(stationarySections.get("1"))
                .commentsText(stationarySections.get("2"))
                .areas(areas)
                .build();
        List<AltrvRoute> routes = Collections.singletonList(AltrvRoute.builder()
                .id("R0")
                .kind(routeKind(routeText, stationaryOnly))
                .rawText(routeText)
                .points(points)
                .events(events)
                .states(routeStates(events))
                .reverse(containsAny(routeText, " REVERSE ", " REV "))
                .partial(containsAny(routeText, " PARTIAL "))
                .common(containsAny(routeText, " COMMON "))
                .branch(containsAny(routeText, " BRANCH "))
                .implicit(points.size() > 1 && !containsAny(routeText, " ROUTE ", " VIA "))
                .build());

        AltrvMessage message = AltrvMessage.builder()
                .rawText(normalized)
                .sections(Collections.unmodifiableMap(sections))
                .activityName(sections.get("A"))
                .mission(sections.get("B"))
                .location(sections.get("C"))
                .routeText(routeText)
                .timingText(sections.get("F"))
                .aircraftText(sections.get("G"))
                .callsigns(callsigns(sections.get("A")))
                .aircraftTypes(aircraftTypes(sections.get("B")))
                .departureGroups(departureGroups(sections.get("F")))
                .areas(areas)
                .destinations(destinations(sections.get("E")))
                .exits(exits(normalized))
                .sectionResults(sectionResults)
                .stationaryReservation(stationaryReservation)
                .firstDepartureTime(firstDeparture(sections.get("F")))
                .avanaTime(avana(sections.get("F"), firstDeparture(sections.get("F"))))
                .routeGroups(Collections.singletonList(AltrvRouteGroup.builder()
                        .id("G0")
                        .routes(routes)
                        .build()))
                .events(events)
                .diagnostics(diagnostics)
                .build();
        for (String validation : new AltrvMessageValidator().validate(message)) {
            addDiagnostic(diagnostics, typedDiagnostics, validationSeverity(validation), null, validation,
                    span(0, normalized.length(), normalized));
        }
        return result(diagnostics.stream().noneMatch(d -> d.startsWith("Missing ALTRV section")), message,
                diagnostics, sectionResults, typedDiagnostics, spans(sectionResults, events, areas));
    }

    private AltrvParseResult result(boolean accepted, AltrvMessage message, List<String> diagnostics) {
        return result(accepted, message, diagnostics, Collections.emptyList(), Collections.emptyList(), Collections.emptyList());
    }

    private AltrvParseResult result(boolean accepted, AltrvMessage message, List<String> diagnostics,
                                   List<AltrvSectionResult> sectionResults,
                                   List<AltrvDiagnostic> typedDiagnostics,
                                   List<AltrvSourceSpan> sourceSpans) {
        return AltrvParseResult.builder()
                .accepted(accepted)
                .message(message)
                .diagnostics(Collections.unmodifiableList(new ArrayList<>(diagnostics)))
                .sectionResults(Collections.unmodifiableList(new ArrayList<>(sectionResults)))
                .typedDiagnostics(Collections.unmodifiableList(new ArrayList<>(typedDiagnostics)))
                .sourceSpans(Collections.unmodifiableList(new ArrayList<>(sourceSpans)))
                .build();
    }

    private Map<String, String> sections(String text) {
        Map<String, String> sections = new LinkedHashMap<>();
        Matcher matcher = FIELD.matcher(text);
        while (matcher.find()) {
            sections.put(matcher.group(1), matcher.group(2).trim());
        }
        return sections;
    }

    private List<AltrvSectionResult> sectionResults(String text, Map<String, String> sections,
                                                    Map<String, String> stationarySections) {
        List<AltrvSectionResult> results = new ArrayList<>();
        boolean stationaryOnly = !stationarySections.isEmpty() && sections.isEmpty();
        if (!stationaryOnly) {
            for (String required : Arrays.asList("A", "D", "F", "G")) {
                if (!sections.containsKey(required)) {
                    results.add(AltrvSectionResult.builder()
                            .name(required)
                            .rawText("")
                            .sourceSpan(spanOf(text, required))
                            .accepted(false)
                            .diagnostics(Collections.singletonList(AltrvDiagnostic.builder()
                                    .severity(AltrvDiagnosticSeverity.ERROR)
                                    .sectionName(required)
                                    .message("Missing ALTRV section " + required)
                                    .sourceSpan(spanOf(text, required))
                                    .build()))
                            .build());
                }
            }
        }
        for (Map.Entry<String, String> entry : sections.entrySet()) {
            List<AltrvDiagnostic> sectionDiagnostics = new ArrayList<>();
            if (entry.getValue().trim().isEmpty()) {
                sectionDiagnostics.add(AltrvDiagnostic.builder()
                        .severity(AltrvDiagnosticSeverity.ERROR)
                        .sectionName(entry.getKey())
                        .message("ALTRV section " + entry.getKey() + " is empty")
                        .sourceSpan(spanOf(text, entry.getKey()))
                        .build());
            }
            results.add(AltrvSectionResult.builder()
                    .name(entry.getKey())
                    .rawText(entry.getValue())
                    .sourceSpan(spanOf(text, entry.getKey()))
                    .accepted(sectionDiagnostics.isEmpty())
                    .diagnostics(Collections.unmodifiableList(sectionDiagnostics))
                    .build());
        }
        for (Map.Entry<String, String> entry : stationarySections.entrySet()) {
            List<AltrvDiagnostic> sectionDiagnostics = new ArrayList<>();
            if (entry.getValue().trim().isEmpty()) {
                sectionDiagnostics.add(AltrvDiagnostic.builder()
                        .severity(AltrvDiagnosticSeverity.ERROR)
                        .sectionName(entry.getKey())
                        .message("Stationary section " + entry.getKey() + " is empty")
                        .sourceSpan(stationarySpanOf(text, entry.getKey()))
                        .build());
            }
            results.add(AltrvSectionResult.builder()
                    .name(entry.getKey())
                    .rawText(entry.getValue())
                    .sourceSpan(stationarySpanOf(text, entry.getKey()))
                    .accepted(sectionDiagnostics.isEmpty())
                    .diagnostics(Collections.unmodifiableList(sectionDiagnostics))
                    .build());
        }
        return Collections.unmodifiableList(results);
    }

    private Map<String, String> stationarySections(String text) {
        Map<String, String> sections = new LinkedHashMap<>();
        Matcher matcher = Pattern.compile("(?m)^([12])\\.[ \\t]*(.*?)(?=\\n\\s*[12]\\.[ \\t]|\\z)", Pattern.DOTALL).matcher(text);
        while (matcher.find()) {
            sections.put(matcher.group(1), matcher.group(2).trim());
        }
        return sections;
    }

    private List<AltrvRouteEvent> detectEvents(String text) {
        String normalized = " " + text.toUpperCase(Locale.US).replaceAll("\\s+", " ") + " ";
        List<AltrvRouteEvent> events = new ArrayList<>();
        add(events, normalized, "ADMIS MITO", AltrvRouteEventType.ADMIS_MITO);
        if (Pattern.compile("\\bADMIS\\s+\\d+\\s+SEC\\b").matcher(normalized).find()) {
            events.add(event(AltrvRouteEventType.ADMIS_SECONDS, "ADMIS SEC"));
        }
        add(events, normalized, "AVANA", AltrvRouteEventType.AVANA);
        add(events, normalized, "LVLOF BY", AltrvRouteEventType.LVLOF_BY);
        add(events, normalized, "LVLOF W/I", AltrvRouteEventType.LVLOF_WITHIN);
        add(events, normalized, "CLMB", AltrvRouteEventType.CLIMB);
        add(events, normalized, "CLIMB", AltrvRouteEventType.CLIMB);
        add(events, normalized, "DSND", AltrvRouteEventType.DESCEND);
        add(events, normalized, "DESCEND", AltrvRouteEventType.DESCEND);
        add(events, normalized, "LVLOF", AltrvRouteEventType.LEVEL_OFF);
        add(events, normalized, "AIRFL", AltrvRouteEventType.AIRFL);
        add(events, normalized, "AIRFL BEGINS", AltrvRouteEventType.AIR_REFUELING_BEGIN);
        add(events, normalized, "BEGIN AIRFL", AltrvRouteEventType.AIR_REFUELING_BEGIN);
        add(events, normalized, "AIRFL ENDS", AltrvRouteEventType.AIR_REFUELING_END);
        add(events, normalized, "END AIRFL", AltrvRouteEventType.AIR_REFUELING_END);
        add(events, normalized, "AIRFL DRCT", AltrvRouteEventType.AIR_REFUELING_DIRECT);
        add(events, normalized, "ARCP", AltrvRouteEventType.AIR_REFUELING_CONTROL_POINT);
        add(events, normalized, "ARIP", AltrvRouteEventType.AIR_REFUELING_INITIAL_POINT);
        add(events, normalized, "IFPFP", AltrvRouteEventType.IFPFP);
        add(events, normalized, "ALTRV ENDS", AltrvRouteEventType.ALTRV_ENDS);
        add(events, normalized, "JOIN", AltrvRouteEventType.JOIN);
        add(events, normalized, "JOIN CMN RTE", AltrvRouteEventType.JOIN_COMMON_ROUTE);
        add(events, normalized, "RCVR CMN RTE", AltrvRouteEventType.RECEIVER_COMMON_ROUTE);
        add(events, normalized, "RECEIVER CMN RTE", AltrvRouteEventType.RECEIVER_COMMON_ROUTE);
        add(events, normalized, "LEAVE", AltrvRouteEventType.LEAVE);
        add(events, normalized, "CLOSE BRANCH", AltrvRouteEventType.BRANCH_CLOSE);
        add(events, normalized, "MERGE", AltrvRouteEventType.BRANCH_MERGE);
        add(events, normalized, "LAND", AltrvRouteEventType.LAND);
        add(events, normalized, "ORBIT", AltrvRouteEventType.ORBIT);
        add(events, normalized, "TIMING TRIANGLE", AltrvRouteEventType.TIMING_TRIANGLE);
        add(events, normalized, "MANEUVER AREA", AltrvRouteEventType.MANEUVER_AREA);
        add(events, normalized, "STATIONARY", AltrvRouteEventType.STATIONARY_AREA);
        add(events, normalized, "BROAD FRONT", AltrvRouteEventType.BROAD_FRONT);
        add(events, normalized, "SUPERSONIC", AltrvRouteEventType.SUPERSONIC);
        add(events, normalized, "CROSS ABOVE", AltrvRouteEventType.CROSS_ABOVE);
        add(events, normalized, "CROSS BELOW", AltrvRouteEventType.CROSS_BELOW);
        add(events, normalized, "AIR REFUEL", AltrvRouteEventType.AIR_REFUELING);
        add(events, normalized, "STREAM", AltrvRouteEventType.STREAM_FORMATION);
        add(events, normalized, "CELL", AltrvRouteEventType.CELL_FORMATION);
        add(events, normalized, "CANADA ENTER", AltrvRouteEventType.CANADA_ENTER);
        add(events, normalized, "CANADA EXIT", AltrvRouteEventType.CANADA_EXIT);
        add(events, normalized, "ENCAN", AltrvRouteEventType.CANADA_ENTER);
        add(events, normalized, "EXCAN", AltrvRouteEventType.CANADA_EXIT);
        add(events, normalized, "RAVEC", AltrvRouteEventType.RAVEC);
        add(events, normalized, "BRANCH", AltrvRouteEventType.ROUTE_BRANCH);
        add(events, normalized, "COMMON", AltrvRouteEventType.ROUTE_COMMON);
        add(events, normalized, "PARTIAL", AltrvRouteEventType.ROUTE_PARTIAL);
        add(events, normalized, "REVERSE", AltrvRouteEventType.ROUTE_REVERSE);
        add(events, normalized, "ALT DEP", AltrvRouteEventType.ALTERNATE_DEPARTURE_ROUTE);
        add(events, normalized, "ALTERNATE DEPARTURE", AltrvRouteEventType.ALTERNATE_DEPARTURE_ROUTE);
        if (Pattern.compile("\\b[A-Z]{2,5}\\s+\\d{3}/\\d{3}\\b").matcher(normalized).find()) {
            events.add(event(AltrvRouteEventType.RADIAL_DME, "radial/dme"));
        }
        if (Pattern.compile("\\bJ\\d+\\b").matcher(normalized).find()) {
            events.add(event(AltrvRouteEventType.JET_ROUTE, "J-route"));
            events.add(event(AltrvRouteEventType.AIRWAY, "airway"));
        }
        if (Pattern.compile("\\bV\\d+\\b").matcher(normalized).find()) {
            events.add(event(AltrvRouteEventType.VICTOR_ROUTE, "V-route"));
            events.add(event(AltrvRouteEventType.AIRWAY, "airway"));
        }
        if (Pattern.compile("\\b[A-Z]{2,5}\\s+\\d{4}\\b").matcher(normalized).find()) {
            events.add(event(AltrvRouteEventType.NAMED_FIX, "named fix"));
            events.add(event(AltrvRouteEventType.NAVAID, "navaid/fix candidate"));
        }
        for (int i = 0; i < events.size(); i++) {
            events.get(i).setEventId("EV" + i);
        }
        return Collections.unmodifiableList(events);
    }

    private List<AltrvDestination> destinations(String text) {
        if (text == null) {
            return Collections.emptyList();
        }
        List<AltrvDestination> destinations = new ArrayList<>();
        Matcher matcher = Pattern.compile("\\b([A-Z][A-Z0-9]{1,7})\\b").matcher(text.toUpperCase(Locale.US));
        while (matcher.find()) {
            destinations.add(AltrvDestination.builder()
                    .rawText(matcher.group())
                    .fixId(matcher.group(1))
                    .callsigns(callsigns(text))
                    .sourceSpan(span(matcher.start(), matcher.end(), matcher.group()))
                    .build());
        }
        return Collections.unmodifiableList(destinations);
    }

    private List<AltrvExit> exits(String text) {
        List<AltrvExit> exits = new ArrayList<>();
        String normalized = text == null ? "" : text.toUpperCase(Locale.US);
        if (normalized.contains("ORBIT")) {
            exits.add(AltrvExit.builder().type("ORBIT").rawText("ORBIT").sourceSpan(firstSpan(normalized, "ORBIT")).build());
        }
        if (normalized.contains("MANEUVER AREA")) {
            exits.add(AltrvExit.builder().type("MANEUVER_AREA").rawText("MANEUVER AREA").sourceSpan(firstSpan(normalized, "MANEUVER AREA")).build());
        }
        if (normalized.contains("STATIONARY")) {
            exits.add(AltrvExit.builder().type("STATIONARY_RESERVATION").rawText("STATIONARY").sourceSpan(firstSpan(normalized, "STATIONARY")).build());
        }
        return Collections.unmodifiableList(exits);
    }

    private List<AltrvCallsign> callsigns(String text) {
        return callsigns(text, true);
    }

    private List<AltrvCallsign> callsigns(String text, boolean allowBareTokens) {
        if (text == null) {
            return Collections.emptyList();
        }
        List<AltrvCallsign> callsigns = new ArrayList<>();
        Matcher matcher = Pattern.compile("\\b([A-Z]{1,8})(\\d{1,3})\\b").matcher(text.toUpperCase(Locale.US));
        while (matcher.find()) {
            callsigns.add(AltrvCallsign.builder().name(matcher.group(1)).number(matcher.group(2)).build());
        }
        if (allowBareTokens && callsigns.isEmpty() && text.trim().matches("[A-Z0-9 -]+")) {
            for (String token : text.trim().toUpperCase(Locale.US).split("[\\s,]+")) {
                if (token.matches("[A-Z0-9]{1,7}")) {
                    callsigns.add(AltrvCallsign.builder().name(token).number("").build());
                }
            }
        }
        return Collections.unmodifiableList(callsigns);
    }

    private List<AltrvAircraftType> aircraftTypes(String text) {
        if (text == null) {
            return Collections.emptyList();
        }
        List<AltrvAircraftType> types = new ArrayList<>();
        Matcher slash = Pattern.compile("\\b(\\d+)\\s*/\\s*([A-Z][A-Z0-9-]{1,10})\\b").matcher(text.toUpperCase(Locale.US));
        while (slash.find()) {
            types.add(AltrvAircraftType.builder()
                    .count(Integer.parseInt(slash.group(1)))
                    .type(slash.group(2))
                    .build());
        }
        Matcher spaced = Pattern.compile("\\b(\\d+)\\s+([A-Z][A-Z0-9-]{1,10})\\b").matcher(text.toUpperCase(Locale.US));
        while (types.isEmpty() && spaced.find()) {
            types.add(AltrvAircraftType.builder()
                    .count(Integer.parseInt(spaced.group(1)))
                    .type(spaced.group(2))
                    .build());
        }
        return Collections.unmodifiableList(types);
    }

    private List<AltrvDepartureGroup> departureGroups(String text) {
        if (text == null) {
            return Collections.emptyList();
        }
        return Collections.singletonList(AltrvDepartureGroup.builder()
                .rawText(text)
                .callsigns(callsigns(text, false))
                .build());
    }

    private List<AltrvArea> areas(String text) {
        String normalized = text == null ? "" : text.toUpperCase(Locale.US);
        List<AltrvArea> areas = new ArrayList<>();
        if (normalized.contains("BNDD BY") || normalized.contains("AREA BNDD BY")) {
            areas.add(AltrvArea.builder()
                    .type(AltrvAreaType.POLYGON)
                    .rawText("BNDD BY")
                    .boundaryPoints(routePoints(normalized, new ArrayList<>()))
                    .sourceSpan(firstSpan(normalized, "BNDD BY"))
                    .build());
        }
        Matcher circle = Pattern.compile("\\bWITHIN\\s+(\\d+(?:\\.\\d+)?)\\s+(?:NM\\s+)?RADIUS\\b").matcher(normalized);
        if (circle.find()) {
            areas.add(AltrvArea.builder()
                    .type(AltrvAreaType.CIRCLE)
                    .rawText(circle.group())
                    .radiusNauticalMiles(Double.parseDouble(circle.group(1)))
                    .boundaryPoints(routePoints(normalized, new ArrayList<>()))
                    .sourceSpan(span(circle.start(), circle.end(), circle.group()))
                    .build());
        }
        Matcher line = Pattern.compile("\\b(\\d+(?:\\.\\d+)?)\\s+(?:NM\\s+)?WIDE\\b").matcher(normalized);
        if (line.find()) {
            areas.add(AltrvArea.builder()
                    .type(AltrvAreaType.LINE)
                    .rawText(line.group())
                    .widthNauticalMiles(Double.parseDouble(line.group(1)))
                    .boundaryPoints(routePoints(normalized, new ArrayList<>()))
                    .sourceSpan(span(line.start(), line.end(), line.group()))
                    .build());
        }
        if (normalized.contains("MANEUVER AREA")) {
            areas.add(AltrvArea.builder().type(AltrvAreaType.MANEUVER).rawText("MANEUVER AREA")
                    .sourceSpan(firstSpan(normalized, "MANEUVER AREA")).build());
        }
        if (normalized.contains("TIMING TRIANGLE")) {
            areas.add(AltrvArea.builder().type(AltrvAreaType.TIMING_TRIANGLE).rawText("TIMING TRIANGLE")
                    .sourceSpan(firstSpan(normalized, "TIMING TRIANGLE")).build());
        }
        if (normalized.matches("(?s).*\\n?1\\.\\s+.*")) {
            areas.add(AltrvArea.builder().type(AltrvAreaType.STATIONARY).rawText("stationary section")
                    .sourceSpan(firstSpan(normalized, "1.")).build());
        }
        return Collections.unmodifiableList(areas);
    }

    private ZonedDateTime firstDeparture(String fSection) {
        if (fSection == null) {
            return null;
        }
        Matcher matcher = ETD.matcher(fSection);
        if (!matcher.find()) {
            return null;
        }
        return dateTime(matcher.group(1), matcher.group(2), matcher.group(3), matcher.group(4), matcher.group(5));
    }

    private ZonedDateTime avana(String fSection, ZonedDateTime firstDeparture) {
        if (fSection == null || firstDeparture == null) {
            return null;
        }
        Matcher matcher = AVANA.matcher(fSection);
        if (!matcher.find()) {
            return null;
        }
        ZonedDateTime parsed = dateTime(matcher.group(1), matcher.group(2), matcher.group(3),
                firstDeparture.getMonth().name(), Integer.toString(firstDeparture.getYear()));
        if (parsed.isBefore(firstDeparture)) {
            parsed = parsed.plusMonths(1);
        }
        return parsed;
    }

    private void add(List<AltrvRouteEvent> events, String text, String marker, AltrvRouteEventType type) {
        if (text.contains(" " + marker + " ")) {
            events.add(event(type, marker));
        }
    }

    private AltrvRouteEvent event(AltrvRouteEventType type, String source) {
        return AltrvRouteEvent.builder()
                .type(type)
                .sourceText(source)
                .sourceSpan(span(-1, -1, source))
                .build();
    }

    private List<AltrvRoutePoint> routePoints(String routeText, List<String> diagnostics) {
        List<AltrvRoutePoint> points = new ArrayList<>();
        Matcher coordinate = COORDINATE_POINT.matcher(routeText);
        while (coordinate.find()) {
            String raw = coordinate.group(1).toUpperCase(Locale.US) + " " + coordinate.group(2).toUpperCase(Locale.US);
            points.add(AltrvRoutePoint.builder()
                    .id(raw)
                    .rawText(coordinate.group())
                    .timeOffset(coordinate.group(3))
                    .coordinate(coordinate(coordinate.group(1), coordinate.group(2)))
                    .coordinateLiteral(true)
                    .sourceSpan(span(coordinate.start(), coordinate.end(), coordinate.group()))
                    .build());
        }
        if (!points.isEmpty()) {
            return Collections.unmodifiableList(points);
        }

        String scrubbed = FLIGHT_LEVEL.matcher(routeText.toUpperCase(Locale.US)).replaceAll(" ");
        Matcher named = NAMED_POINT.matcher(scrubbed);
        List<String> controls = Arrays.asList("ADMISSION", "ADMIS", "AVANA", "LVLOF", "LAND", "JOIN", "LEAVE", "ENDS");
        while (named.find()) {
            String id = named.group(1).toUpperCase(Locale.US);
            if (controls.contains(id)) {
                continue;
            }
            points.add(AltrvRoutePoint.builder()
                    .id(id)
                    .rawText(named.group())
                    .radialDegrees(named.group(2) == null ? null : Double.parseDouble(named.group(2)))
                    .dmeNauticalMiles(named.group(3) == null ? null : Double.parseDouble(named.group(3)))
                    .timeOffset(named.group(4))
                    .coordinateLiteral(false)
                    .sourceSpan(span(named.start(), named.end(), named.group()))
                    .build());
        }
        if (points.size() < 2) {
            diagnostics.add("ALTRV route text contains fewer than two parseable route points");
        }
        return Collections.unmodifiableList(points);
    }

    private boolean containsAny(String routeText, String... markers) {
        String normalized = " " + (routeText == null ? "" : routeText.toUpperCase(Locale.US)).replaceAll("\\s+", " ") + " ";
        for (String marker : markers) {
            if (normalized.contains(marker)) {
                return true;
            }
        }
        return false;
    }

    private AltrvRouteKind routeKind(String routeText, boolean stationaryOnly) {
        if (stationaryOnly) {
            return AltrvRouteKind.STATIONARY;
        }
        if (containsAny(routeText, " BRANCH ", " BEGIN BRANCH RTE ", " BR ")) {
            return AltrvRouteKind.BRANCH;
        }
        if (containsAny(routeText, " COMMON ", " BEGIN CMN RTE ", " CMN RTE ")) {
            return AltrvRouteKind.COMMON;
        }
        if (containsAny(routeText, " PARTIAL ", " BEGIN PARTIAL RTE ", " PR ", " PR2 ")) {
            return AltrvRouteKind.PARTIAL;
        }
        if (containsAny(routeText, " REVERSE ", " REV ")) {
            return AltrvRouteKind.REVERSE;
        }
        if (containsAny(routeText, " ALT DEP ", " ALTERNATE DEPARTURE ")) {
            return AltrvRouteKind.ALTERNATE_DEPARTURE;
        }
        if (routeText != null && !routeText.trim().isEmpty() && !containsAny(routeText, " ROUTE ", " VIA ")) {
            return AltrvRouteKind.IMPLICIT;
        }
        return AltrvRouteKind.STANDARD;
    }

    private List<AltrvRouteState> routeStates(List<AltrvRouteEvent> events) {
        List<AltrvRouteState> states = new ArrayList<>();
        states.add(AltrvRouteState.OPEN);
        for (AltrvRouteEvent event : events) {
            if (event.getType() == AltrvRouteEventType.AIR_REFUELING
                    || event.getType() == AltrvRouteEventType.AIR_REFUELING_BEGIN) {
                states.add(AltrvRouteState.IN_AIR_REFUELING);
            }
            if (event.getType() == AltrvRouteEventType.BROAD_FRONT) {
                states.add(AltrvRouteState.IN_BROAD_FRONT);
            }
            if (event.getType() == AltrvRouteEventType.SUPERSONIC) {
                states.add(AltrvRouteState.IN_SUPERSONIC);
            }
            if (event.getType() == AltrvRouteEventType.ALTRV_ENDS) {
                states.add(AltrvRouteState.CLOSED);
            }
        }
        return Collections.unmodifiableList(states);
    }

    private AltrvDiagnosticSeverity validationSeverity(String message) {
        return message.contains("must")
                || message.contains("cannot")
                || message.contains("Duplicate")
                || message.contains("different")
                || message.contains("do not match")
                ? AltrvDiagnosticSeverity.ERROR
                : AltrvDiagnosticSeverity.WARNING;
    }

    private GeoCoordinate coordinate(String latitude, String longitude) {
        return GeoCoordinate.builder()
                .latitude(parseLatitude(latitude.toUpperCase(Locale.US)))
                .longitude(parseLongitude(longitude.toUpperCase(Locale.US)))
                .altitude(0)
                .build();
    }

    private double parseLatitude(String value) {
        int degrees = Integer.parseInt(value.substring(0, 2));
        int minutes = Integer.parseInt(value.substring(2, 4));
        double decimal = degrees + minutes / 60.0;
        return value.charAt(4) == 'S' ? -decimal : decimal;
    }

    private double parseLongitude(String value) {
        int degreeDigits = value.length() == 5 ? 2 : 3;
        int degrees = Integer.parseInt(value.substring(0, degreeDigits));
        int minutes = Integer.parseInt(value.substring(degreeDigits, degreeDigits + 2));
        double decimal = degrees + minutes / 60.0;
        return value.charAt(value.length() - 1) == 'W' ? -decimal : decimal;
    }

    private ZonedDateTime dateTime(String day, String hour, String minute, String month, String year) {
        LocalDateTime local = LocalDateTime.of(Integer.parseInt(year), parseMonth(month),
                Integer.parseInt(day), Integer.parseInt(hour), Integer.parseInt(minute));
        return local.atZone(DEFAULT_ZONE);
    }

    private int parseMonth(String value) {
        String normalized = value.toUpperCase(Locale.US);
        for (Month month : Month.values()) {
            if (month.name().startsWith(normalized)) {
                return month.getValue();
            }
        }
        return Month.valueOf(normalized).getValue();
    }

    private void addDiagnostic(List<String> diagnostics, List<AltrvDiagnostic> typedDiagnostics,
                               AltrvDiagnosticSeverity severity, String section, String message,
                               AltrvSourceSpan sourceSpan) {
        diagnostics.add(message);
        typedDiagnostics.add(AltrvDiagnostic.builder()
                .severity(severity)
                .sectionName(section)
                .message(message)
                .sourceSpan(sourceSpan)
                .build());
    }

    private AltrvSourceSpan spanOf(String text, String section) {
        Matcher matcher = Pattern.compile("(?m)^" + Pattern.quote(section) + "\\.[ \\t]*(.*?)(?=\\n\\s*[A-G]\\.[ \\t]|\\z)",
                Pattern.DOTALL).matcher(text);
        if (matcher.find()) {
            return span(matcher.start(), matcher.end(), matcher.group());
        }
        return span(0, Math.min(text.length(), 1), "");
    }

    private AltrvSourceSpan stationarySpanOf(String text, String section) {
        Matcher matcher = Pattern.compile("(?m)^" + Pattern.quote(section) + "\\.[ \\t]*(.*?)(?=\\n\\s*[12]\\.[ \\t]|\\z)",
                Pattern.DOTALL).matcher(text);
        if (matcher.find()) {
            return span(matcher.start(), matcher.end(), matcher.group());
        }
        return span(0, Math.min(text.length(), 1), "");
    }

    private AltrvSourceSpan firstSpan(String text, String marker) {
        int start = text.indexOf(marker);
        return start < 0 ? span(-1, -1, marker) : span(start, start + marker.length(), marker);
    }

    private AltrvSourceSpan span(int start, int end, String text) {
        return AltrvSourceSpan.builder()
                .startOffset(start)
                .endOffset(end)
                .text(text)
                .build();
    }

    private List<AltrvSourceSpan> spans(List<AltrvSectionResult> sections,
                                        List<AltrvRouteEvent> events,
                                        List<AltrvArea> areas) {
        List<AltrvSourceSpan> spans = new ArrayList<>();
        for (AltrvSectionResult section : sections) {
            if (section.getSourceSpan() != null) {
                spans.add(section.getSourceSpan());
            }
        }
        for (AltrvRouteEvent event : events) {
            if (event.getSourceSpan() != null) {
                spans.add(event.getSourceSpan());
            }
        }
        for (AltrvArea area : areas) {
            if (area.getSourceSpan() != null) {
                spans.add(area.getSourceSpan());
            }
        }
        return Collections.unmodifiableList(spans);
    }
}
