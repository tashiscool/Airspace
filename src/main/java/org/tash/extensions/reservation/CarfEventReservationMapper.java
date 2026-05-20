package org.tash.extensions.reservation;

import org.tash.data.GeoCoordinate;
import org.tash.spatial.SpatialCircle;
import org.tash.spatial.SpatialPoint;
import org.tash.spatial.SpatialPolygon;
import org.tash.spatial.SpatialVolume;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.UUID;

public class CarfEventReservationMapper {
    private final ProtectedRouteGeometryFactory geometryFactory = new ProtectedRouteGeometryFactory();

    public List<AirspaceReservation> toReservations(String idPrefix, CarfReservationEvent event) {
        if (event.getPoints() == null || event.getPoints().isEmpty()) {
            return Collections.emptyList();
        }
        switch (event.getType()) {
            case ROUTE_SEGMENT:
                return lineOrAreaReservations(idPrefix, event);
            case TIMING_TRIANGLE:
                validateTimingTriangle(event);
                return areaReservation(idPrefix, event, polygonVolume(idPrefix, event));
            case MANEUVER_AREA:
                return areaReservation(idPrefix, event, areaVolume(idPrefix, event));
            case ORBIT:
                validateOrbit(event);
                return areaReservation(idPrefix, event, circleVolume(idPrefix, event, event.getProtectedRadiusNauticalMiles()));
            case STATIONARY_AREA:
                return areaReservation(idPrefix, event, areaVolume(idPrefix, event));
            default:
                return Collections.emptyList();
        }
    }

    private List<AirspaceReservation> lineOrAreaReservations(String idPrefix, CarfReservationEvent event) {
        List<GeoCoordinate> points = event.getPoints();
        List<AirspaceReservation> reservations = new ArrayList<>();
        for (int i = 0; i < points.size() - 1; i++) {
            GeoCoordinate start = points.get(i);
            GeoCoordinate end = points.get(i + 1);
            double routeWidth = event.getRouteWidthNauticalMiles() > 0
                    ? event.getRouteWidthNauticalMiles()
                    : event.getProtectedRadiusNauticalMiles() * 2.0;
            reservations.add(baseBuilder(idPrefix + "-E" + i, event)
                    .routeStart(start)
                    .routeEnd(end)
                    .routeStartFix("P" + i)
                    .routeEndFix("P" + (i + 1))
                    .sourceFixes(event.getSourceFixes() == null ? asFixes(i, i + 1) : event.getSourceFixes())
                    .protectedVolume(geometryFactory.routeSegmentVolume(
                            idPrefix + "-E" + i,
                            start,
                            end,
                            Math.max(routeWidth / 2.0, 0.1),
                            event.getLowerAltitudeFeet(),
                            event.getUpperAltitudeFeet(),
                            event.getStartTime(),
                            event.getEndTime()))
                    .build());
        }
        return reservations;
    }

    private List<AirspaceReservation> areaReservation(String idPrefix, CarfReservationEvent event, SpatialVolume volume) {
        GeoCoordinate point = volume.getBasePolygon().getSamplePoint().getCoordinate();
        return Collections.singletonList(baseBuilder(idPrefix + "-E0", event)
                .routeStart(point)
                .routeEnd(point)
                .routeStartFix("P0")
                .routeEndFix("P0")
                .sourceFixes(event.getSourceFixes() == null ? allFixes(event.getPoints().size()) : event.getSourceFixes())
                .protectedVolume(volume)
                .build());
    }

    private AirspaceReservation.AirspaceReservationBuilder baseBuilder(String id, CarfReservationEvent event) {
        double width = Math.max(event.getRouteWidthNauticalMiles(), event.getProtectedRadiusNauticalMiles());
        return AirspaceReservation.builder()
                .id(id)
                .startTime(event.getStartTime())
                .endTime(event.getEndTime())
                .lowerAltitudeFeet(event.getLowerAltitudeFeet())
                .upperAltitudeFeet(event.getUpperAltitudeFeet())
                .deconflictionLowerAltitudeFeet(event.getLowerAltitudeFeet() - 249.5)
                .deconflictionUpperAltitudeFeet(event.getUpperAltitudeFeet() + 249.5)
                .verticalSeparationFeet(499)
                .lateralSeparationNauticalMiles(width > 0 ? width : 100.0)
                .routeWidthNauticalMiles(width)
                .avanaMinutes(event.getAvanaMinutes())
                .longitudinalSeparationMinutes(event.getLongitudinalSeparationMinutes())
                .reservationType(event.getType().name())
                .sourceText(event.getSourceText())
                .sourceFixes(event.getSourceFixes() == null ? null : event.getSourceFixes())
                .routeGraphNodeIds(event.getRouteGraphNodeIds())
                .diagnostics(event.getDiagnostics())
                .displayShapeIntent(event.getShapeIntent() == null ? "DISP_NORM" : event.getShapeIntent())
                .deconflictionShapeIntent("DECON");
    }

    private List<String> asFixes(int first, int second) {
        List<String> fixes = new ArrayList<>();
        fixes.add("P" + first);
        fixes.add("P" + second);
        return fixes;
    }

    private List<String> allFixes(int count) {
        List<String> fixes = new ArrayList<>();
        for (int i = 0; i < count; i++) {
            fixes.add("P" + i);
        }
        return fixes;
    }

    private SpatialVolume areaVolume(String idPrefix, CarfReservationEvent event) {
        if (event.getPoints().size() >= 3) {
            return polygonVolume(idPrefix, event);
        }
        double radius = event.getProtectedRadiusNauticalMiles() > 0 ? event.getProtectedRadiusNauticalMiles() : 1.0;
        return circleVolume(idPrefix, event, radius);
    }

    private SpatialVolume polygonVolume(String idPrefix, CarfReservationEvent event) {
        return volume(idPrefix, event, geometryFactory.paddedPolygon(
                idPrefix,
                event.getPoints(),
                areaPaddingNauticalMiles(event)));
    }

    private SpatialVolume circleVolume(String idPrefix, CarfReservationEvent event, double radiusNauticalMiles) {
        SpatialPoint center = point(idPrefix + "-CENTER", event.getPoints().get(0));
        SpatialCircle circle = SpatialCircle.builder()
                .id(idPrefix + "-CIRCLE-" + UUID.randomUUID())
                .center(center)
                .radius(radiusNauticalMiles)
                .build();
        return volume(idPrefix, event, circle.toPolygon(36));
    }

    private SpatialVolume volume(String idPrefix, CarfReservationEvent event, SpatialPolygon polygon) {
        return SpatialVolume.builder()
                .id(idPrefix + "-VOLUME-" + UUID.randomUUID())
                .basePolygon(polygon)
                .lowerAltitude(event.getLowerAltitudeFeet())
                .upperAltitude(event.getUpperAltitudeFeet())
                .startTime(event.getStartTime())
                .endTime(event.getEndTime())
                .build();
    }

    private double areaPaddingNauticalMiles(CarfReservationEvent event) {
        if (event.getProtectedRadiusNauticalMiles() > 0) {
            return event.getProtectedRadiusNauticalMiles();
        }
        if (event.getRouteWidthNauticalMiles() > 0) {
            return event.getRouteWidthNauticalMiles() / 2.0;
        }
        return 0.0;
    }

    private SpatialPoint point(String id, GeoCoordinate coordinate) {
        return SpatialPoint.builder()
                .id(id)
                .coordinate(coordinate)
                .build();
    }

    private void validateTimingTriangle(CarfReservationEvent event) {
        if (event.getPoints().size() != 3) {
            throw new IllegalArgumentException("TIMING_TRIANGLE requires exactly three area points");
        }
    }

    private void validateOrbit(CarfReservationEvent event) {
        if (event.getProtectedRadiusNauticalMiles() < 0 || event.getProtectedRadiusNauticalMiles() > 999) {
            throw new IllegalArgumentException("ORBIT radius must be between 0 and 999 NM");
        }
    }
}
