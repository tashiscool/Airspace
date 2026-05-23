package org.tash.extensions.carf.altrv;

import org.tash.data.GeoCoordinate;
import org.tash.extensions.carf.refdata.CarfReferenceDataProvider;
import org.tash.extensions.reservation.AirspaceReservation;
import org.tash.extensions.reservation.CarfEventReservationMapper;
import org.tash.extensions.reservation.CarfReservationEvent;
import org.tash.extensions.reservation.CarfReservationEventType;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class AltrvSpatialMapper {
    private static final Pattern FLIGHT_LEVEL = Pattern.compile("\\bFL(\\d{3})B(\\d{3})\\b", Pattern.CASE_INSENSITIVE);
    private static final Pattern SINGLE_FLIGHT_LEVEL = Pattern.compile("\\bFL(\\d{3})\\b", Pattern.CASE_INSENSITIVE);
    private static final Pattern SURFACE_TO_FLIGHT_LEVEL = Pattern.compile("\\b(?:SFC|SURFACE)\\s+(?:TO|-)?\\s*FL(\\d{3})\\b", Pattern.CASE_INSENSITIVE);
    private static final Pattern SURFACE_TO_UNLIMITED = Pattern.compile("\\b(?:SFC|SURFACE)\\s+(?:TO|-)?\\s*(?:UNL|UNLIMITED)\\b", Pattern.CASE_INSENSITIVE);
    private static final Pattern ABOVE_FLIGHT_LEVEL = Pattern.compile("\\b(?:ABV|ABOVE)\\s+FL(\\d{3})\\b", Pattern.CASE_INSENSITIVE);
    private static final Pattern BELOW_FLIGHT_LEVEL = Pattern.compile("\\b(?:BLW|BELOW)\\s+FL(\\d{3})\\b", Pattern.CASE_INSENSITIVE);

    public List<CarfReservationEvent> toReservationEvents(AltrvMessage message, CarfReferenceDataProvider referenceDataProvider) {
        if (message == null) {
            return Collections.emptyList();
        }
        List<CarfReservationEvent> events = new ArrayList<>();
        for (AltrvRouteGroup group : safe(message.getRouteGroups())) {
            for (AltrvRoute route : safe(group.getRoutes())) {
                List<GeoCoordinate> points = coordinates(route.getPoints(), referenceDataProvider);
                if (points.size() >= 2) {
                    double[] routeAltitudes = altitudes(message);
                    events.add(CarfReservationEvent.builder()
                            .type(CarfReservationEventType.ROUTE_SEGMENT)
                            .startTime(startTime(message))
                            .endTime(endTime(message))
                            .lowerAltitudeFeet(routeAltitudes[0])
                            .upperAltitudeFeet(routeAltitudes[1])
                            .routeWidthNauticalMiles(width(message))
                            .points(points)
                            .sourceFixes(route.getPoints().stream().map(AltrvRoutePoint::getId).collect(Collectors.toList()))
                            .routeGraphNodeIds(Collections.singletonList(route.getId()))
                            .diagnostics(diagnostics(message, route.getPoints(), referenceDataProvider))
                            .avanaMinutes(avanaMinutes(message))
                            .longitudinalSeparationMinutes(0)
                            .shapeIntent("ALTRV_ROUTE")
                            .sourceText(route.getRawText())
                            .build());
                }
            }
        }
        for (AltrvArea area : safe(message.getAreas())) {
            List<GeoCoordinate> points = coordinates(area, referenceDataProvider);
            if (points.isEmpty()) {
                continue;
            }
            double[] areaAltitudes = areaAltitudes(message, area);
            events.add(CarfReservationEvent.builder()
                    .type(eventType(area))
                    .startTime(startTime(message))
                    .endTime(endTime(message))
                    .lowerAltitudeFeet(areaAltitudes[0])
                    .upperAltitudeFeet(areaAltitudes[1])
                    .protectedRadiusNauticalMiles(area.getRadiusNauticalMiles())
                    .routeWidthNauticalMiles(area.getWidthNauticalMiles())
                    .points(points)
                    .sourceFixes(area.getBoundaryPoints() == null ? Collections.emptyList()
                            : area.getBoundaryPoints().stream().map(AltrvRoutePoint::getId).collect(Collectors.toList()))
                    .diagnostics(diagnostics(message, area.getBoundaryPoints(), referenceDataProvider))
                    .avanaMinutes(avanaMinutes(message))
                    .shapeIntent(areaShapeIntent(area))
                    .sourceText(areaSourceText(area))
                    .build());
        }
        return Collections.unmodifiableList(events);
    }

    public List<AirspaceReservation> toReservations(String idPrefix,
                                                    AltrvMessage message,
                                                    CarfReferenceDataProvider referenceDataProvider) {
        List<AirspaceReservation> reservations = new ArrayList<>();
        CarfEventReservationMapper mapper = new CarfEventReservationMapper();
        int index = 0;
        for (CarfReservationEvent event : toReservationEvents(message, referenceDataProvider)) {
            reservations.addAll(mapper.toReservations(idPrefix + "-A" + index++, event));
        }
        return Collections.unmodifiableList(reservations);
    }

    private CarfReservationEventType eventType(AltrvArea area) {
        switch (area.getType()) {
            case MANEUVER:
                return CarfReservationEventType.MANEUVER_AREA;
            case TIMING_TRIANGLE:
                return CarfReservationEventType.TIMING_TRIANGLE;
            case STATIONARY:
                return CarfReservationEventType.STATIONARY_AREA;
            case CIRCLE:
                return CarfReservationEventType.ORBIT;
            case LINE:
                return CarfReservationEventType.ROUTE_SEGMENT;
            case POLYGON:
            default:
                return CarfReservationEventType.STATIONARY_AREA;
        }
    }

    private List<GeoCoordinate> coordinates(AltrvArea area, CarfReferenceDataProvider referenceDataProvider) {
        return coordinates(area.getBoundaryPoints(), referenceDataProvider);
    }

    private List<GeoCoordinate> coordinates(List<AltrvRoutePoint> routePoints, CarfReferenceDataProvider referenceDataProvider) {
        List<GeoCoordinate> points = new ArrayList<>();
        for (AltrvRoutePoint point : safe(routePoints)) {
            if (point.getCoordinate() != null) {
                points.add(point.getCoordinate());
            } else if (referenceDataProvider != null) {
                referenceDataProvider.resolveFixOrNavaid(point.getId()).ifPresent(fix -> {
                    if (point.getRadialDegrees() != null && point.getDmeNauticalMiles() != null) {
                        points.add(fix.destinationPoint(point.getDmeNauticalMiles(), point.getRadialDegrees()));
                    } else {
                        points.add(fix);
                    }
                });
            }
        }
        return points;
    }

    private List<String> diagnostics(AltrvMessage message,
                                     List<AltrvRoutePoint> points,
                                     CarfReferenceDataProvider referenceDataProvider) {
        Set<String> diagnostics = new LinkedHashSet<>();
        if (message != null && message.getDiagnostics() != null) {
            diagnostics.addAll(message.getDiagnostics());
        }
        if (referenceDataProvider != null) {
            for (AltrvRoutePoint point : safe(points)) {
                if (point.getCoordinate() == null && point.getId() != null
                        && !referenceDataProvider.resolveFixOrNavaid(point.getId()).isPresent()) {
                    diagnostics.add("Unresolved ALTRV fix/navaid " + point.getId());
                }
            }
        }
        return new ArrayList<>(diagnostics);
    }

    private String areaShapeIntent(AltrvArea area) {
        if (area == null || blank(area.getGeometryIntent())) {
            return "ALTRV_AREA";
        }
        return area.getGeometryIntent();
    }

    private String areaSourceText(AltrvArea area) {
        if (area == null) {
            return "";
        }
        List<String> parts = new ArrayList<>();
        if (!blank(area.getRawText())) {
            parts.add(area.getRawText());
        }
        if (!blank(area.getGeometryIntent())) {
            parts.add("geometryIntent=" + area.getGeometryIntent());
        }
        if (!blank(area.getEnterExitAssociation())) {
            parts.add("association=" + area.getEnterExitAssociation());
        }
        if (!blank(area.getTimingText())) {
            parts.add("timing=" + area.getTimingText());
        }
        if (!blank(area.getLowerFlightLevel()) || !blank(area.getUpperFlightLevel())) {
            parts.add("altitude=" + value(area.getLowerFlightLevel()) + "-" + value(area.getUpperFlightLevel()));
        }
        if (area.getRadiusNauticalMiles() > 0) {
            parts.add("radiusNm=" + area.getRadiusNauticalMiles());
        }
        if (area.getWidthNauticalMiles() > 0) {
            parts.add("corridorWidthNm=" + area.getWidthNauticalMiles());
        }
        if (area.getMetadata() != null && !area.getMetadata().isEmpty()) {
            parts.add("metadata=" + area.getMetadata());
        }
        return String.join(" | ", parts);
    }

    private double[] areaAltitudes(AltrvMessage message, AltrvArea area) {
        double[] defaults = altitudes(message);
        if (area == null || (blank(area.getLowerFlightLevel()) && blank(area.getUpperFlightLevel()))) {
            return defaults;
        }
        double lower = altitude(area.getLowerFlightLevel(), defaults[0], true);
        double upper = altitude(area.getUpperFlightLevel(), defaults[1], false);
        if (upper < lower) {
            return defaults;
        }
        return new double[]{lower, upper};
    }

    private double altitude(String value, double defaultValue, boolean lower) {
        if (blank(value)) {
            return defaultValue;
        }
        String normalized = value.trim().toUpperCase(java.util.Locale.US);
        if ("SFC".equals(normalized) || "SURFACE".equals(normalized)) {
            return 0;
        }
        if ("UNL".equals(normalized) || "UNLIMITED".equals(normalized)) {
            return 100000;
        }
        if (normalized.startsWith("FL")) {
            normalized = normalized.substring(2);
        }
        if (normalized.startsWith("ABV")) {
            normalized = normalized.replace("ABV", "").replace("FL", "").trim();
            return parseFlightLevel(normalized, defaultValue);
        }
        if (normalized.startsWith("BLW")) {
            normalized = normalized.replace("BLW", "").replace("FL", "").trim();
            return lower ? 0 : parseFlightLevel(normalized, defaultValue);
        }
        return parseFlightLevel(normalized, defaultValue);
    }

    private double parseFlightLevel(String value, double defaultValue) {
        try {
            return Double.parseDouble(value) * 100.0;
        } catch (NumberFormatException ignored) {
            return defaultValue;
        }
    }

    private String value(String value) {
        return value == null ? "" : value;
    }

    private boolean blank(String value) {
        return value == null || value.trim().isEmpty();
    }

    private ZonedDateTime startTime(AltrvMessage message) {
        return message.getFirstDepartureTime() == null
                ? ZonedDateTime.parse("2010-01-01T00:00:00Z")
                : message.getFirstDepartureTime();
    }

    private ZonedDateTime endTime(AltrvMessage message) {
        if (message.getAvanaTime() != null && message.getFirstDepartureTime() != null
                && message.getAvanaTime().isAfter(message.getFirstDepartureTime())) {
            return message.getAvanaTime();
        }
        return startTime(message).plusHours(1);
    }

    private double avanaMinutes(AltrvMessage message) {
        if (message.getFirstDepartureTime() == null || message.getAvanaTime() == null) {
            return 0;
        }
        return Math.max(0, java.time.Duration.between(message.getFirstDepartureTime(), message.getAvanaTime()).toMinutes());
    }

    private double width(AltrvMessage message) {
        String raw = message.getRawText() == null ? "" : message.getRawText();
        Matcher eitherSide = Pattern.compile("\\b(\\d+(?:\\.\\d+)?)\\s*(?:NM|NMR)?\\s+EITHER\\s+SIDE\\b",
                Pattern.CASE_INSENSITIVE).matcher(raw);
        if (eitherSide.find()) {
            return Double.parseDouble(eitherSide.group(1)) * 2.0;
        }
        Matcher wide = Pattern.compile("\\b(\\d+(?:\\.\\d+)?)\\s+(?:NM\\s+)?WIDE\\b",
                Pattern.CASE_INSENSITIVE).matcher(raw);
        return wide.find() ? Double.parseDouble(wide.group(1)) : 0;
    }

    private double[] altitudes(AltrvMessage message) {
        String raw = message.getRawText() == null ? "" : message.getRawText();
        Matcher range = FLIGHT_LEVEL.matcher(raw);
        if (range.find()) {
            return new double[]{Double.parseDouble(range.group(1)) * 100.0,
                    Double.parseDouble(range.group(2)) * 100.0};
        }
        Matcher surfaceToUnlimited = SURFACE_TO_UNLIMITED.matcher(raw);
        if (surfaceToUnlimited.find()) {
            return new double[]{0, 100000};
        }
        Matcher surfaceToFlightLevel = SURFACE_TO_FLIGHT_LEVEL.matcher(raw);
        if (surfaceToFlightLevel.find()) {
            return new double[]{0, Double.parseDouble(surfaceToFlightLevel.group(1)) * 100.0};
        }
        Matcher aboveFlightLevel = ABOVE_FLIGHT_LEVEL.matcher(raw);
        if (aboveFlightLevel.find()) {
            return new double[]{Double.parseDouble(aboveFlightLevel.group(1)) * 100.0, 100000};
        }
        Matcher belowFlightLevel = BELOW_FLIGHT_LEVEL.matcher(raw);
        if (belowFlightLevel.find()) {
            return new double[]{0, Double.parseDouble(belowFlightLevel.group(1)) * 100.0};
        }
        Matcher single = SINGLE_FLIGHT_LEVEL.matcher(raw);
        if (single.find()) {
            double altitude = Double.parseDouble(single.group(1)) * 100.0;
            return new double[]{altitude, altitude};
        }
        return new double[]{0, 60000};
    }

    private <T> List<T> safe(List<T> values) {
        return values == null ? Collections.emptyList() : values;
    }
}
