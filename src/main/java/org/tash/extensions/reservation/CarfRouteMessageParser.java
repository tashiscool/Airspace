package org.tash.extensions.reservation;

import org.tash.data.GeoCoordinate;

import java.time.LocalDateTime;
import java.time.Month;
import java.time.ZoneId;
import java.time.Duration;
import java.time.ZonedDateTime;
import java.time.temporal.ChronoUnit;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class CarfRouteMessageParser {
    private static final ZoneId DEFAULT_ZONE = ZoneId.of("America/New_York");
    private static final Pattern FIELD = Pattern.compile("(?m)^([A-G])\\.\\s*(.*?)(?=\\n\\s*[A-G]\\.\\s|\\z)",
            Pattern.DOTALL);
    private static final Pattern ETD = Pattern.compile(
            "ETD\\b.*?(\\d{2})(\\d{2})(\\d{2})\\s+([A-Z]{3})\\s+(\\d{4})(?:\\s+ADMIS\\s+(?:(\\d+)\\s+(MIN|SEC)|MITO))?",
            Pattern.CASE_INSENSITIVE);
    private static final Pattern AVANA = Pattern.compile("\\[?\\bAVANA\\s+(\\d{2})(\\d{2})(\\d{2})\\]?", Pattern.CASE_INSENSITIVE);
    private static final Pattern TAS = Pattern.compile("TAS\\s*:\\s*(\\d+)\\s*K(?:TAS|TS)?", Pattern.CASE_INSENSITIVE);
    private static final Pattern COORDINATE = Pattern.compile("(\\d{4}[NS])\\s+(\\d{4,5}[EW])\\s+(\\d{4})");
    private static final Pattern VERTICAL_RANGE = Pattern.compile("FL(\\d+)B(\\d+)", Pattern.CASE_INSENSITIVE);
    private static final Pattern CLEAN_TOKEN_BOUNDARY = Pattern.compile("[\\[\\](),]+");
    private static final Set<String> ROUTE_CONTROL_WORDS = new HashSet<>(Arrays.asList(
            "PR", "CLMB", "CLIMB", "DSND", "DESCEND", "LVLOF", "BY", "WI", "W", "I", "NM",
            "JOIN", "LEAVE", "LAND", "ALTRV", "ENDS", "IFPFP", "TO", "FROM", "BEGINS",
            "BEGIN", "END", "AIRFL", "CRUISE", "REQ", "FL", "DCT", "CMPS"));

    private final CarfWaypointResolver waypointResolver;
    private final CarfRouteSeparationResolver separationResolver;
    private final ProtectedRouteGeometryFactory geometryFactory = new ProtectedRouteGeometryFactory();

    public CarfRouteMessageParser() {
        this(new DefaultCarfWaypointResolver());
    }

    public CarfRouteMessageParser(CarfWaypointResolver waypointResolver) {
        this(waypointResolver, new DefaultCarfRouteSeparationResolver());
    }

    public CarfRouteMessageParser(CarfWaypointResolver waypointResolver, CarfRouteSeparationResolver separationResolver) {
        this.waypointResolver = waypointResolver;
        this.separationResolver = separationResolver;
    }

    public CarfRouteMessage parse(String text) {
        String normalized = text.replace("\r", "");
        String activityName = field(normalized, "A");
        String routeText = field(normalized, "D");
        String fField = field(normalized, "F");
        ZonedDateTime etd = parseEtd(fField);
        ZonedDateTime avana = parseAvana(fField, etd);
        int admissionSeconds = parseAdmissionSeconds(fField);

        List<RouteLeg> legs = parseRouteLegs(routeText);
        List<AirspaceReservation> reservations = new ArrayList<>();
        double avanaMinutes = avana == null ? 0 : Math.max(0, ChronoUnit.MINUTES.between(etd, avana));
        for (int i = 0; i < legs.size(); i++) {
            RouteLeg leg = legs.get(i);
            ZonedDateTime legStart = etd.plusHours(leg.startOffsetHours).plusMinutes(leg.startOffsetMinutes);
            ZonedDateTime legEnd = etd.plusHours(leg.endOffsetHours).plusMinutes(leg.endOffsetMinutes).plusHours(1);
            reservations.addAll(routeReservations(activityName, i, leg, legStart, legEnd, avanaMinutes));
        }

        return CarfRouteMessage.builder()
                .activityName(activityName)
                .mission(field(normalized, "B"))
                .location(field(normalized, "C"))
                .routeText(routeText)
                .estimatedDepartureTime(etd)
                .avanaTime(avana)
                .admissionMinutes(admissionSeconds / 60)
                .admissionSeconds(admissionSeconds)
                .trueAirspeedKnots(parseTrueAirspeedKnots(field(normalized, "G")))
                .projectOfficer(labeledValue(normalized, "PROJECT OFFICER"))
                .artccsConcerned(labeledValue(normalized, "ARTCCS CONCERNED"))
                .additionalInfo(labeledValue(normalized, "ADDITIONAL INFO"))
                .resolvedWaypointNames(resolvedWaypointNames(legs))
                .unresolvedWaypointNames(unresolvedWaypointNames(legs))
                .reservations(reservations)
                .build();
    }

    private List<AirspaceReservation> routeReservations(String activityName, int legIndex, RouteLeg leg,
                                                        ZonedDateTime legStart, ZonedDateTime legEnd,
                                                        double avanaMinutes) {
        CarfSeparationStandard startStandard = separationResolver.resolve(
                leg.start.coordinate, leg.verticalRange.lower, leg.verticalRange.upper);
        CarfSeparationStandard endStandard = separationResolver.resolve(
                leg.end.coordinate, leg.verticalRange.lower, leg.verticalRange.upper);
        if (startStandard.sameAs(endStandard)) {
            return Collections.singletonList(routeReservation(
                    activityName + "-R" + legIndex,
                    leg,
                    leg.start.coordinate,
                    leg.end.coordinate,
                    legStart,
                    legEnd,
                    0.0,
                    1.0,
                    avanaMinutes,
                    startStandard));
        }

        double transitionRatio = findTransitionRatio(leg, startStandard, endStandard);
        GeoCoordinate transitionPoint = leg.start.coordinate.interpolate(leg.end.coordinate, transitionRatio);
        ZonedDateTime transitionTime = interpolateTime(legStart, legEnd, transitionRatio);

        List<AirspaceReservation> split = new ArrayList<>();
        split.add(routeReservation(
                activityName + "-R" + legIndex + "A",
                leg,
                leg.start.coordinate,
                transitionPoint,
                legStart,
                transitionTime,
                0.0,
                transitionRatio,
                avanaMinutes,
                startStandard));
        split.add(routeReservation(
                activityName + "-R" + legIndex + "B",
                leg,
                transitionPoint,
                leg.end.coordinate,
                transitionTime,
                legEnd,
                transitionRatio,
                1.0,
                avanaMinutes,
                endStandard));
        return split;
    }

    private AirspaceReservation routeReservation(String id, RouteLeg leg,
                                                 GeoCoordinate routeStart, GeoCoordinate routeEnd,
                                                 ZonedDateTime startTime, ZonedDateTime endTime,
                                                 double sourceRatioStart, double sourceRatioEnd,
                                                 double avanaMinutes,
                                                 CarfSeparationStandard standard) {
        AirspaceReservation reservation = AirspaceReservation.builder()
                .id(id)
                .startTime(startTime)
                .endTime(endTime)
                .deconflictionEndTime(null)
                .lowerAltitudeFeet(leg.verticalRange.lower)
                .upperAltitudeFeet(leg.verticalRange.upper)
                .deconflictionLowerAltitudeFeet(leg.verticalRange.lower - standard.getVerticalSeparationFeet() / 2.0)
                .deconflictionUpperAltitudeFeet(leg.verticalRange.upper + standard.getVerticalSeparationFeet() / 2.0)
                .verticalSeparationFeet(standard.getVerticalSeparationFeet())
                .lateralSeparationNauticalMiles(standard.getLateralSeparationNauticalMiles())
                .longitudinalSeparationNauticalMiles(0)
                .avanaMinutes(avanaMinutes)
                .longitudinalSeparationMinutes(standard.getLongitudinalSeparationMinutes())
                .routeWidthNauticalMiles(standard.getRouteWidthNauticalMiles())
                .sourceRatioStart(sourceRatioStart)
                .sourceRatioEnd(sourceRatioEnd)
                .reservationType("ROUTE_SEGMENT")
                .routeStartFix(leg.start.name)
                .routeEndFix(leg.end.name)
                .sourceFixes(Arrays.asList(leg.start.name, leg.end.name))
                .sourceText(leg.start.rawToken + " " + leg.end.rawToken)
                .displayShapeIntent("DISP_NORM")
                .deconflictionShapeIntent("DECON")
                .routeStart(routeStart)
                .routeEnd(routeEnd)
                .build();
        reservation.setProtectedVolume(geometryFactory.routeSegmentVolume(id, reservation));
        return reservation;
    }

    private double findTransitionRatio(RouteLeg leg,
                                       CarfSeparationStandard startStandard,
                                       CarfSeparationStandard endStandard) {
        double low = 0.0;
        double high = 1.0;
        for (int i = 0; i < 20; i++) {
            double mid = (low + high) / 2.0;
            GeoCoordinate point = leg.start.coordinate.interpolate(leg.end.coordinate, mid);
            CarfSeparationStandard midStandard = separationResolver.resolve(
                    point, leg.verticalRange.lower, leg.verticalRange.upper);
            if (midStandard.sameAs(startStandard)) {
                low = mid;
            } else if (midStandard.sameAs(endStandard)) {
                high = mid;
            } else {
                high = mid;
            }
        }
        return (low + high) / 2.0;
    }

    private ZonedDateTime interpolateTime(ZonedDateTime start, ZonedDateTime end, double ratio) {
        long millis = Duration.between(start, end).toMillis();
        return start.plus((long) (millis * ratio), ChronoUnit.MILLIS);
    }

    private List<RouteLeg> parseRouteLegs(String routeText) {
        List<RoutePoint> points = new ArrayList<>();
        List<RouteLeg> legs = new ArrayList<>();
        String[] tokens = CLEAN_TOKEN_BOUNDARY.matcher(routeText.replace("...", " ")).replaceAll(" ").trim().split("\\s+");

        VerticalRange currentRange = null;
        VerticalRange pendingTargetRange = null;
        boolean climbPending = false;
        for (int i = 0; i < tokens.length; i++) {
            String token = clean(tokens[i]);
            if (token.isEmpty()) {
                continue;
            }
            Matcher vertical = VERTICAL_RANGE.matcher(token);
            if (vertical.matches()) {
                VerticalRange range = verticalRange(vertical);
                if (climbPending) {
                    pendingTargetRange = range;
                    climbPending = false;
                } else if (currentRange == null) {
                    currentRange = range;
                } else {
                    currentRange = range;
                }
                continue;
            }
            if ("CLMB".equals(token) || "CLIMB".equals(token)) {
                climbPending = true;
                continue;
            }
            RoutePoint point = null;
            int consumed = 0;
            if (token.matches("\\d{4}[NS]") && i + 2 < tokens.length) {
                String candidate = token + " " + clean(tokens[i + 1]) + " " + clean(tokens[i + 2]);
                Matcher coordinate = COORDINATE.matcher(candidate);
                if (coordinate.matches()) {
                    point = routePoint(coordinate);
                    consumed = 2;
                }
            } else if (isNamedFixCandidate(token) && i + 2 < tokens.length
                    && clean(tokens[i + 1]).matches("\\d{3}/\\d{3}") && clean(tokens[i + 2]).matches("\\d{4}")) {
                point = radialRoutePoint(token, clean(tokens[i + 1]), clean(tokens[i + 2]));
                consumed = 2;
            } else if (isNamedFixCandidate(token) && i + 1 < tokens.length && clean(tokens[i + 1]).matches("\\d{4}")) {
                point = namedRoutePoint(token, clean(tokens[i + 1]));
                consumed = 1;
            } else if (isNamedFixCandidate(token) && i + 2 < tokens.length
                    && isNamedFixCandidate(clean(tokens[i + 1])) && clean(tokens[i + 2]).matches("\\d{4}")) {
                RoutePoint inferred = namedRoutePoint(token, "0000");
                if (points.isEmpty()) {
                    points.add(inferred);
                }
                continue;
            }

            if (point == null) {
                continue;
            }

            if (!points.isEmpty()) {
                VerticalRange legRange = currentRange;
                if (pendingTargetRange != null) {
                    legRange = currentRange.merge(pendingTargetRange);
                    currentRange = pendingTargetRange;
                    pendingTargetRange = null;
                }
                legs.add(new RouteLeg(points.get(points.size() - 1), point, legRange));
            }
            points.add(point);
            i += consumed;
        }

        if (currentRange == null || legs.isEmpty()) {
            throw new IllegalArgumentException("CARF route message is missing route legs or vertical ranges");
        }
        return legs;
    }

    private String field(String text, String name) {
        Matcher matcher = FIELD.matcher(text);
        while (matcher.find()) {
            if (name.equals(matcher.group(1))) {
                return matcher.group(2).trim();
            }
        }
        throw new IllegalArgumentException("Missing CARF field " + name);
    }

    private String labeledValue(String text, String label) {
        Matcher matcher = Pattern.compile("(?m)^" + Pattern.quote(label) + ":\\s*(.*)$").matcher(text);
        return matcher.find() ? matcher.group(1).trim() : null;
    }

    private ZonedDateTime parseEtd(String field) {
        Matcher matcher = ETD.matcher(field);
        if (!matcher.find()) {
            throw new IllegalArgumentException("Invalid CARF ETD field: " + field);
        }
        return dateTime(matcher.group(1), matcher.group(2), matcher.group(3), matcher.group(4), matcher.group(5));
    }

    private ZonedDateTime parseAvana(String field, ZonedDateTime etd) {
        Matcher matcher = AVANA.matcher(field);
        if (!matcher.find()) {
            return null;
        }
        return dateTime(matcher.group(1), matcher.group(2), matcher.group(3),
                etd.getMonth().name(), Integer.toString(etd.getYear()));
    }

    private int parseAdmissionSeconds(String field) {
        Matcher matcher = ETD.matcher(field);
        if (!matcher.find()) {
            return 0;
        }
        if (matcher.group(6) == null) {
            return field.toUpperCase(Locale.US).contains("ADMIS MITO") ? 0 : 0;
        }
        int value = Integer.parseInt(matcher.group(6));
        String unit = matcher.group(7);
        return unit != null && unit.equalsIgnoreCase("SEC") ? value : value * 60;
    }

    private int parseTrueAirspeedKnots(String field) {
        Matcher matcher = TAS.matcher(field);
        if (!matcher.find()) {
            return 0;
        }
        return Integer.parseInt(matcher.group(1));
    }

    private ZonedDateTime dateTime(String day, String hour, String minute, String month, String year) {
        LocalDateTime local = LocalDateTime.of(Integer.parseInt(year), parseMonth(month),
                Integer.parseInt(day), Integer.parseInt(hour), Integer.parseInt(minute));
        return local.atZone(DEFAULT_ZONE);
    }

    private int parseMonth(String value) {
        String normalized = value.toUpperCase(Locale.US);
        if (normalized.length() == 3) {
            for (Month month : Month.values()) {
                if (month.name().startsWith(normalized)) {
                    return month.getValue();
                }
            }
        }
        return Month.valueOf(normalized).getValue();
    }

    private VerticalRange verticalRange(Matcher matcher) {
        return new VerticalRange(Double.parseDouble(matcher.group(1)) * 100,
                Double.parseDouble(matcher.group(2)) * 100);
    }

    private RoutePoint routePoint(Matcher matcher) {
        String name = matcher.group(1) + " " + matcher.group(2);
        return new RoutePoint(name, name + " " + matcher.group(3),
                coordinate(matcher.group(1), matcher.group(2)), matcher.group(3), true);
    }

    private RoutePoint namedRoutePoint(String name, String offset) {
        java.util.Optional<GeoCoordinate> resolved = waypointResolver.resolve(name);
        GeoCoordinate coordinate = resolved.orElseGet(() -> deterministicFallbackCoordinate(name));
        return new RoutePoint(name, name + " " + offset, coordinate, offset, resolved.isPresent());
    }

    private RoutePoint radialRoutePoint(String name, String radialDistance, String offset) {
        java.util.Optional<GeoCoordinate> resolved = waypointResolver.resolve(name);
        GeoCoordinate base = resolved.orElseGet(() -> deterministicFallbackCoordinate(name));
        String[] parts = radialDistance.split("/");
        double bearing = Double.parseDouble(parts[0]);
        double distanceNauticalMiles = Double.parseDouble(parts[1]);
        GeoCoordinate coordinate = destination(base, bearing, distanceNauticalMiles);
        return new RoutePoint(name + radialDistance, name + " " + radialDistance + " " + offset, coordinate, offset, resolved.isPresent());
    }

    private List<String> resolvedWaypointNames(List<RouteLeg> legs) {
        return waypointNames(legs, true);
    }

    private List<String> unresolvedWaypointNames(List<RouteLeg> legs) {
        return waypointNames(legs, false);
    }

    private List<String> waypointNames(List<RouteLeg> legs, boolean resolved) {
        Set<String> names = new LinkedHashSet<>();
        for (RouteLeg leg : legs) {
            addWaypointName(names, leg.start, resolved);
            addWaypointName(names, leg.end, resolved);
        }
        return new ArrayList<>(names);
    }

    private void addWaypointName(Set<String> names, RoutePoint point, boolean resolved) {
        if (point.resolvedByResolver != resolved || point.name.contains(" ")) {
            return;
        }
        names.add(point.name.replaceAll("\\d{3}/\\d{3}$", ""));
    }

    private boolean isNamedFixCandidate(String token) {
        return token.matches("[A-Z][A-Z0-9]{1,7}") && !ROUTE_CONTROL_WORDS.contains(token) && !token.matches("J\\d+");
    }

    private String clean(String value) {
        return value == null ? "" : value.toUpperCase(Locale.US).replaceAll("^[^A-Z0-9]+|[^A-Z0-9]+$", "");
    }

    private GeoCoordinate deterministicFallbackCoordinate(String name) {
        int hash = Math.abs(name.hashCode());
        double latitude = 20.0 + (hash % 3000) / 100.0;
        double longitude = -130.0 + ((hash / 3000) % 7000) / 100.0;
        return GeoCoordinate.builder().latitude(latitude).longitude(longitude).altitude(0).build();
    }

    private GeoCoordinate destination(GeoCoordinate start, double bearingDegrees, double distanceNauticalMiles) {
        double angularDistance = (distanceNauticalMiles * 1852.0) / 6371000.0;
        double bearing = Math.toRadians(bearingDegrees);
        double lat1 = Math.toRadians(start.getLatitude());
        double lon1 = Math.toRadians(start.getLongitude());
        double lat2 = Math.asin(Math.sin(lat1) * Math.cos(angularDistance)
                + Math.cos(lat1) * Math.sin(angularDistance) * Math.cos(bearing));
        double lon2 = lon1 + Math.atan2(Math.sin(bearing) * Math.sin(angularDistance) * Math.cos(lat1),
                Math.cos(angularDistance) - Math.sin(lat1) * Math.sin(lat2));
        return GeoCoordinate.builder()
                .latitude(Math.toDegrees(lat2))
                .longitude(Math.toDegrees(lon2))
                .altitude(start.getAltitude())
                .build();
    }

    private GeoCoordinate coordinate(String latitude, String longitude) {
        return GeoCoordinate.builder()
                .latitude(parseLatitude(latitude))
                .longitude(parseLongitude(longitude))
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

    private static class RoutePoint {
        private final String name;
        private final String rawToken;
        private final GeoCoordinate coordinate;
        private final int offsetHours;
        private final int offsetMinutes;
        private final boolean resolvedByResolver;

        private RoutePoint(String name, String rawToken, GeoCoordinate coordinate, String offset, boolean resolvedByResolver) {
            this.name = name;
            this.rawToken = rawToken;
            this.coordinate = coordinate;
            this.offsetHours = Integer.parseInt(offset.substring(0, 2));
            this.offsetMinutes = Integer.parseInt(offset.substring(2, 4));
            this.resolvedByResolver = resolvedByResolver;
        }
    }

    private static class RouteLeg {
        private final RoutePoint start;
        private final RoutePoint end;
        private final VerticalRange verticalRange;
        private final int startOffsetHours;
        private final int startOffsetMinutes;
        private final int endOffsetHours;
        private final int endOffsetMinutes;

        private RouteLeg(RoutePoint start, RoutePoint end, VerticalRange verticalRange) {
            this.start = start;
            this.end = end;
            this.verticalRange = verticalRange;
            this.startOffsetHours = start.offsetHours;
            this.startOffsetMinutes = start.offsetMinutes;
            this.endOffsetHours = end.offsetHours;
            this.endOffsetMinutes = end.offsetMinutes;
        }
    }

    private static class VerticalRange {
        private final double lower;
        private final double upper;

        private VerticalRange(double lower, double upper) {
            this.lower = lower;
            this.upper = upper;
        }

        private VerticalRange merge(VerticalRange other) {
            return new VerticalRange(Math.min(lower, other.lower), Math.max(upper, other.upper));
        }
    }
}
