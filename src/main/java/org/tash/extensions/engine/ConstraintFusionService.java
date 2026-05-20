package org.tash.extensions.engine;

import org.tash.data.GeoCoordinate;
import org.tash.extensions.notam.NotamAirspaceRestriction;
import org.tash.extensions.reservation.AirspaceReservation;
import org.tash.extensions.reservation.ReservationConflict;
import org.tash.extensions.weather.HazardSeverity;
import org.tash.extensions.weather.avoid.WeatherCell;
import org.tash.extensions.weather.decision.RouteBlockagePrediction;
import org.tash.extensions.weather.decision.WeatherDecisionSeverity;
import org.tash.extensions.weather.pirep.PirepIngestResult;
import org.tash.extensions.weather.pirep.PirepIntensity;
import org.tash.extensions.weather.product.WeatherProduct;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class ConstraintFusionService {
    private final OperationalGeometryService geometryService;

    public ConstraintFusionService() {
        this(new OperationalGeometryService());
    }

    public ConstraintFusionService(OperationalGeometryService geometryService) {
        this.geometryService = geometryService == null ? new OperationalGeometryService() : geometryService;
    }

    public ConstraintFusionResult fuse(ConstraintFusionRequest request) {
        List<OperationalConstraint> constraints = new ArrayList<>();
        if (request == null) {
            return ConstraintFusionResult.builder().constraints(constraints).build();
        }
        for (AirspaceReservation reservation : safe(request.getReservations())) {
            constraints.add(OperationalConstraint.builder()
                    .id(reservation.getId())
                    .type(OperationalConstraintType.CARF_RESERVATION)
                    .startTime(reservation.getStartTime())
                    .endTime(reservation.getEffectiveDeconflictionEndTime())
                    .lowerAltitudeFeet(reservation.getLowerAltitudeFeet())
                    .upperAltitudeFeet(reservation.getUpperAltitudeFeet())
                    .severity(WeatherDecisionSeverity.INFO)
                    .confidence(1.0)
                    .rationale("CARF reservation constraint")
                    .geometry(points(reservation.getRouteStart(), reservation.getRouteEnd()))
                    .sources(Collections.singletonList(source("reservation", reservation.getId())))
                    .build());
        }
        for (ReservationConflict conflict : safe(request.getConflicts())) {
            constraints.add(OperationalConstraint.builder()
                    .id(conflict.getFirst().getId() + "__" + conflict.getSecond().getId())
                    .type(OperationalConstraintType.CARF_CONFLICT)
                    .startTime(conflict.getStartTime())
                    .endTime(conflict.getEndTime())
                    .lowerAltitudeFeet(Math.max(conflict.getFirst().getLowerAltitudeFeet(), conflict.getSecond().getLowerAltitudeFeet()))
                    .upperAltitudeFeet(Math.min(conflict.getFirst().getUpperAltitudeFeet(), conflict.getSecond().getUpperAltitudeFeet()))
                    .severity(WeatherDecisionSeverity.WARNING)
                    .confidence(1.0)
                    .rationale(conflict.getExplanation())
                    .geometry(points(conflict.getFirstStartPoint(), conflict.getSecondStartPoint()))
                    .sources(Arrays.asList(source("reservation", conflict.getFirst().getId()), source("reservation", conflict.getSecond().getId())))
                    .build());
        }
        for (NotamAirspaceRestriction notam : safe(request.getNotamRestrictions())) {
            constraints.add(OperationalConstraint.builder()
                    .id(notam.getId())
                    .type(OperationalConstraintType.NOTAM_RESTRICTION)
                    .startTime(notam.getEffectiveStart())
                    .endTime(notam.getEffectiveEnd())
                    .lowerAltitudeFeet(notam.getLowerAltitudeFeet())
                    .upperAltitudeFeet(notam.getUpperAltitudeFeet())
                    .severity(WeatherDecisionSeverity.ADVISORY)
                    .confidence(1.0)
                    .rationale(notam.getDescription())
                    .geometry(notamGeometry(notam))
                    .sources(Collections.singletonList(source("notam", notam.getId())))
                    .build());
        }
        for (WeatherProduct product : safe(request.getWeatherProducts())) {
            constraints.add(OperationalConstraint.builder()
                    .id(product.getId())
                    .type(OperationalConstraintType.WEATHER_HAZARD)
                    .startTime(product.getValidity() == null ? null : product.getValidity().getValidStart())
                    .endTime(product.getValidity() == null ? null : product.getValidity().getValidEnd())
                    .lowerAltitudeFeet(product.getLowerAltitudeFeet() == null ? 0.0 : product.getLowerAltitudeFeet())
                    .upperAltitudeFeet(product.getUpperAltitudeFeet() == null ? 60000.0 : product.getUpperAltitudeFeet())
                    .severity(severity(product.getHazard() instanceof WeatherCell ? ((WeatherCell) product.getHazard()).getSeverity() : HazardSeverity.MODERATE))
                    .confidence(product.confidenceValue())
                    .rationale("Weather product constraint: " + product.getType())
                    .geometry(product.getGeometry())
                    .sources(Collections.singletonList(source("weatherProduct", product.getId())))
                    .build());
        }
        for (RouteBlockagePrediction blockage : safe(request.getRouteBlockages())) {
            if (blockage.isBlocked()) {
                constraints.add(OperationalConstraint.builder()
                        .id("route-blockage-" + blockage.getForecastHour())
                        .type(OperationalConstraintType.ROUTE_BLOCKAGE)
                        .startTime(blockage.getForecastTime())
                        .endTime(blockage.getForecastTime())
                        .severity(blockage.getSeverity())
                        .confidence(blockage.getConfidence())
                        .rationale(blockage.getRationale())
                        .geometry(blockageGeometry(blockage))
                        .sources(Collections.singletonList(source("weatherHazard", blockage.getPrimaryHazardId())))
                        .build());
            }
        }
        for (PirepIngestResult pirep : safe(request.getPirepResults())) {
            if (pirep.getReport() != null && pirep.isAccepted() && pirep.getReport().getLocation() != null) {
                constraints.add(OperationalConstraint.builder()
                        .id(pirep.getReport().getId())
                        .type(OperationalConstraintType.PIREP_HAZARD)
                        .startTime(pirep.getReport().getObservationTime())
                        .endTime(pirep.getReport().getObservationTime() == null ? null : pirep.getReport().getObservationTime().plusHours(1))
                        .lowerAltitudeFeet(pirep.getReport().getAltitudeFeet() == null ? 0.0 : pirep.getReport().getAltitudeFeet())
                        .upperAltitudeFeet(pirep.getReport().getAltitudeFeet() == null ? 60000.0 : pirep.getReport().getAltitudeFeet())
                        .severity(pirepSeverity(pirep.getReport().getIntensity()))
                        .confidence(pirep.getReport().getCodingQuality() == null ? 0.8 : pirep.getReport().getCodingQuality())
                        .rationale((pirep.getReport().isUrgent() ? "Urgent" : "Observed")
                                + " PIREP: " + pirep.getReport().getPhenomenon())
                        .geometry(points(pirep.getReport().getLocation()))
                        .sources(Collections.singletonList(source("pirep", pirep.getReport().getId())))
                .build());
            }
        }
        constraints.addAll(compoundRisks(constraints));
        return ConstraintFusionResult.builder().constraints(constraints).build();
    }

    private List<OperationalConstraint> compoundRisks(List<OperationalConstraint> constraints) {
        List<OperationalConstraint> risks = new ArrayList<>();
        for (int i = 0; i < constraints.size(); i++) {
            OperationalConstraint left = constraints.get(i);
            if (!isRiskSource(left)) {
                continue;
            }
            for (int j = i + 1; j < constraints.size(); j++) {
                OperationalConstraint right = constraints.get(j);
                if (!isRiskSource(right) || left.getType() == right.getType()) {
                    continue;
                }
                if (overlaps(left, right)) {
                    risks.add(OperationalConstraint.builder()
                            .id("compound-" + left.getId() + "-" + right.getId())
                            .type(OperationalConstraintType.COMPOUND_OPERATIONAL_RISK)
                            .startTime(later(left.getStartTime(), right.getStartTime()))
                            .endTime(earlier(left.getEndTime(), right.getEndTime()))
                            .lowerAltitudeFeet(Math.max(left.getLowerAltitudeFeet(), right.getLowerAltitudeFeet()))
                            .upperAltitudeFeet(Math.min(left.getUpperAltitudeFeet(), right.getUpperAltitudeFeet()))
                            .severity(max(left.getSeverity(), right.getSeverity()))
                            .confidence(Math.min(left.getConfidence(), right.getConfidence()))
                            .rationale("Compounded " + left.getType() + " and " + right.getType()
                                    + " overlap in time, altitude, and route/airspace geometry")
                            .geometry(left.getGeometry().isEmpty() ? right.getGeometry() : left.getGeometry())
                            .sources(combine(left.getSources(), right.getSources()))
                            .componentConstraintIds(Arrays.asList(left.getId(), right.getId()))
                            .overlapDimensions(overlapDimensions(left, right))
                            .build());
                }
            }
        }
        return risks;
    }

    private boolean isRiskSource(OperationalConstraint constraint) {
        if (constraint == null) return false;
        return constraint.getType() == OperationalConstraintType.CARF_CONFLICT
                || constraint.getType() == OperationalConstraintType.NOTAM_RESTRICTION
                || constraint.getType() == OperationalConstraintType.WEATHER_HAZARD
                || constraint.getType() == OperationalConstraintType.ROUTE_BLOCKAGE
                || constraint.getType() == OperationalConstraintType.PIREP_HAZARD;
    }

    private boolean overlaps(OperationalConstraint left, OperationalConstraint right) {
        return timeOverlaps(left, right)
                && altitudeOverlaps(left, right)
                && geometryOverlaps(left, right);
    }

    private List<String> overlapDimensions(OperationalConstraint left, OperationalConstraint right) {
        List<String> dimensions = new ArrayList<>();
        if (timeOverlaps(left, right)) dimensions.add("time");
        if (altitudeOverlaps(left, right)) dimensions.add("altitude");
        if (geometryOverlaps(left, right)) dimensions.add("geometry");
        return dimensions;
    }

    private boolean timeOverlaps(OperationalConstraint left, OperationalConstraint right) {
        if (left.getStartTime() == null || left.getEndTime() == null
                || right.getStartTime() == null || right.getEndTime() == null) {
            return true;
        }
        return !left.getEndTime().isBefore(right.getStartTime())
                && !right.getEndTime().isBefore(left.getStartTime());
    }

    private boolean altitudeOverlaps(OperationalConstraint left, OperationalConstraint right) {
        return left.getUpperAltitudeFeet() >= right.getLowerAltitudeFeet()
                && right.getUpperAltitudeFeet() >= left.getLowerAltitudeFeet();
    }

    private boolean geometryOverlaps(OperationalConstraint left, OperationalConstraint right) {
        return geometryService.overlaps(left.getGeometry(), right.getGeometry());
    }

    private WeatherDecisionSeverity max(WeatherDecisionSeverity left, WeatherDecisionSeverity right) {
        if (left == WeatherDecisionSeverity.CRITICAL || right == WeatherDecisionSeverity.CRITICAL) return WeatherDecisionSeverity.CRITICAL;
        if (left == WeatherDecisionSeverity.WARNING || right == WeatherDecisionSeverity.WARNING) return WeatherDecisionSeverity.WARNING;
        if (left == WeatherDecisionSeverity.ADVISORY || right == WeatherDecisionSeverity.ADVISORY) return WeatherDecisionSeverity.ADVISORY;
        return WeatherDecisionSeverity.INFO;
    }

    private java.time.ZonedDateTime later(java.time.ZonedDateTime left, java.time.ZonedDateTime right) {
        if (left == null) return right;
        if (right == null) return left;
        return left.isAfter(right) ? left : right;
    }

    private java.time.ZonedDateTime earlier(java.time.ZonedDateTime left, java.time.ZonedDateTime right) {
        if (left == null) return right;
        if (right == null) return left;
        return left.isBefore(right) ? left : right;
    }

    private List<DecisionSourceRef> combine(List<DecisionSourceRef> left, List<DecisionSourceRef> right) {
        List<DecisionSourceRef> refs = new ArrayList<>();
        if (left != null) refs.addAll(left);
        if (right != null) refs.addAll(right);
        return refs;
    }

    private List<GeoCoordinate> notamGeometry(NotamAirspaceRestriction notam) {
        if (notam == null || (notam.getCenterLatitude() == 0.0 && notam.getCenterLongitude() == 0.0)) {
            return Collections.emptyList();
        }
        double radiusDegrees = Math.max(1.0, notam.getRadiusNauticalMiles()) / 60.0;
        return Arrays.asList(
                GeoCoordinate.builder().latitude(notam.getCenterLatitude() - radiusDegrees).longitude(notam.getCenterLongitude() - radiusDegrees).build(),
                GeoCoordinate.builder().latitude(notam.getCenterLatitude() - radiusDegrees).longitude(notam.getCenterLongitude() + radiusDegrees).build(),
                GeoCoordinate.builder().latitude(notam.getCenterLatitude() + radiusDegrees).longitude(notam.getCenterLongitude() + radiusDegrees).build(),
                GeoCoordinate.builder().latitude(notam.getCenterLatitude() + radiusDegrees).longitude(notam.getCenterLongitude() - radiusDegrees).build());
    }

    private List<GeoCoordinate> blockageGeometry(RouteBlockagePrediction blockage) {
        List<GeoCoordinate> points = new ArrayList<>();
        if (blockage != null) {
            blockage.getIntersections().forEach(intersection -> {
                if (intersection.getEstimatedEntryPoint() != null) points.add(intersection.getEstimatedEntryPoint());
                if (intersection.getEstimatedExitPoint() != null) points.add(intersection.getEstimatedExitPoint());
            });
        }
        return points;
    }

    private WeatherDecisionSeverity severity(HazardSeverity severity) {
        if (severity == HazardSeverity.EXTREME) return WeatherDecisionSeverity.CRITICAL;
        if (severity == HazardSeverity.SEVERE) return WeatherDecisionSeverity.WARNING;
        if (severity == HazardSeverity.MODERATE) return WeatherDecisionSeverity.ADVISORY;
        return WeatherDecisionSeverity.INFO;
    }

    private WeatherDecisionSeverity pirepSeverity(PirepIntensity intensity) {
        if (intensity == PirepIntensity.EXTREME) return WeatherDecisionSeverity.CRITICAL;
        if (intensity == PirepIntensity.SEVERE) return WeatherDecisionSeverity.WARNING;
        if (intensity == PirepIntensity.MODERATE) return WeatherDecisionSeverity.ADVISORY;
        return WeatherDecisionSeverity.INFO;
    }

    private DecisionSourceRef source(String type, String id) {
        return DecisionSourceRef.builder().type(type).id(id).description(type + ":" + id).build();
    }

    private List<GeoCoordinate> points(GeoCoordinate... points) {
        List<GeoCoordinate> list = new ArrayList<>();
        if (points != null) {
            for (GeoCoordinate point : points) {
                if (point != null) list.add(point);
            }
        }
        return list;
    }

    private <T> List<T> safe(List<T> values) {
        return values == null ? Collections.emptyList() : values;
    }
}
