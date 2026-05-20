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
import java.util.List;
import java.util.stream.Collectors;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class AltrvSpatialMapper {
    private static final Pattern FLIGHT_LEVEL = Pattern.compile("\\bFL(\\d{3})B(\\d{3})\\b", Pattern.CASE_INSENSITIVE);

    public List<CarfReservationEvent> toReservationEvents(AltrvMessage message, CarfReferenceDataProvider referenceDataProvider) {
        if (message == null) {
            return Collections.emptyList();
        }
        List<CarfReservationEvent> events = new ArrayList<>();
        for (AltrvRouteGroup group : safe(message.getRouteGroups())) {
            for (AltrvRoute route : safe(group.getRoutes())) {
                List<GeoCoordinate> points = coordinates(route.getPoints(), referenceDataProvider);
                if (points.size() >= 2) {
                    events.add(CarfReservationEvent.builder()
                            .type(CarfReservationEventType.ROUTE_SEGMENT)
                            .startTime(startTime(message))
                            .endTime(endTime(message))
                            .lowerAltitudeFeet(altitudes(message)[0])
                            .upperAltitudeFeet(altitudes(message)[1])
                            .routeWidthNauticalMiles(width(message))
                            .points(points)
                            .sourceFixes(route.getPoints().stream().map(AltrvRoutePoint::getId).collect(Collectors.toList()))
                            .routeGraphNodeIds(Collections.singletonList(route.getId()))
                            .diagnostics(message.getDiagnostics())
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
            events.add(CarfReservationEvent.builder()
                    .type(eventType(area))
                    .startTime(startTime(message))
                    .endTime(endTime(message))
                    .lowerAltitudeFeet(altitudes(message)[0])
                    .upperAltitudeFeet(altitudes(message)[1])
                    .protectedRadiusNauticalMiles(area.getRadiusNauticalMiles())
                    .routeWidthNauticalMiles(area.getWidthNauticalMiles())
                    .points(points)
                    .sourceFixes(area.getBoundaryPoints() == null ? Collections.emptyList()
                            : area.getBoundaryPoints().stream().map(AltrvRoutePoint::getId).collect(Collectors.toList()))
                    .diagnostics(message.getDiagnostics())
                    .avanaMinutes(avanaMinutes(message))
                    .shapeIntent("ALTRV_AREA")
                    .sourceText(area.getRawText())
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
        Matcher matcher = Pattern.compile("\\b(\\d+(?:\\.\\d+)?)\\s+(?:NM\\s+)?WIDE\\b",
                Pattern.CASE_INSENSITIVE).matcher(message.getRawText() == null ? "" : message.getRawText());
        return matcher.find() ? Double.parseDouble(matcher.group(1)) : 0;
    }

    private double[] altitudes(AltrvMessage message) {
        Matcher matcher = FLIGHT_LEVEL.matcher(message.getRawText() == null ? "" : message.getRawText());
        if (matcher.find()) {
            return new double[]{Double.parseDouble(matcher.group(1)) * 100.0,
                    Double.parseDouble(matcher.group(2)) * 100.0};
        }
        return new double[]{0, 60000};
    }

    private <T> List<T> safe(List<T> values) {
        return values == null ? Collections.emptyList() : values;
    }
}
