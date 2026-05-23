package org.tash.extensions.visualization;

import jakarta.enterprise.context.ApplicationScoped;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.notam.NotamAirspaceRestriction;
import org.tash.extensions.reservation.AirspaceReservation;
import org.tash.extensions.reservation.ReservationConflict;
import org.tash.extensions.weather.HazardousWeather;
import org.tash.extensions.weather.avoid.CircularWeatherCell;
import org.tash.extensions.weather.avoid.WeatherCell;
import org.tash.extensions.weather.decision.RouteHazardIntersection;
import org.tash.extensions.weather.decision.RouteWeatherAdvisory;
import org.tash.extensions.weather.decision.WeatherHazardSnapshot;
import org.tash.extensions.weather.product.WeatherProduct;
import org.tash.spatial.SpatialPoint;
import org.tash.spatial.SpatialPolygon;
import org.tash.spatial.SpatialVolume;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

@ApplicationScoped
public class AirspaceVisualizationService {
    public AirspaceFeatureCollection featuresForReservations(Collection<AirspaceReservation> reservations) {
        List<AirspaceFeature> features = new ArrayList<>();
        if (reservations != null) {
            for (AirspaceReservation reservation : reservations) {
                features.add(featureForReservation(reservation));
                AirspaceFeature routeFeature = featureForReservationRoute(reservation);
                if (routeFeature != null) {
                    features.add(routeFeature);
                }
            }
        }
        return AirspaceFeatureCollection.builder().features(features).build();
    }

    public AirspaceFeatureCollection featuresForConflicts(Collection<ReservationConflict> conflicts) {
        List<AirspaceFeature> features = new ArrayList<>();
        if (conflicts != null) {
            for (ReservationConflict conflict : conflicts) {
                features.add(featureForConflict(conflict));
            }
        }
        return AirspaceFeatureCollection.builder().features(features).build();
    }

    public AirspaceFeatureCollection featuresForNotams(Collection<NotamAirspaceRestriction> restrictions) {
        List<AirspaceFeature> features = new ArrayList<>();
        if (restrictions != null) {
            for (NotamAirspaceRestriction restriction : restrictions) {
                features.add(featureForNotam(restriction));
            }
        }
        return AirspaceFeatureCollection.builder().features(features).build();
    }

    public AirspaceFeatureCollection featuresForWeather(Collection<WeatherHazardSnapshot> hazards) {
        List<AirspaceFeature> features = new ArrayList<>();
        if (hazards != null) {
            for (WeatherHazardSnapshot hazard : hazards) {
                features.add(featureForWeather(hazard));
            }
        }
        return AirspaceFeatureCollection.builder().features(features).build();
    }

    public AirspaceFeatureCollection featuresForWeatherProducts(Collection<WeatherProduct> products) {
        List<AirspaceFeature> features = new ArrayList<>();
        if (products != null) {
            for (WeatherProduct product : products) {
                features.add(featureForWeatherProduct(product));
            }
        }
        return AirspaceFeatureCollection.builder().features(features).build();
    }

    public AirspaceFeatureCollection featuresForWeatherAdvisory(RouteWeatherAdvisory advisory) {
        List<AirspaceFeature> features = new ArrayList<>();
        if (advisory != null) {
            int index = 0;
            for (RouteHazardIntersection intersection : advisory.getIntersections()) {
                features.add(featureForRouteWeatherIntersection(intersection, index++));
            }
        }
        return AirspaceFeatureCollection.builder().features(features).build();
    }

    public AirspaceFeatureCollection combined(VisualizationRequest request) {
        List<AirspaceFeature> features = new ArrayList<>();
        if (request != null) {
            features.addAll(featuresForReservations(request.getReservations()).getFeatures());
            features.addAll(featuresForConflicts(request.getConflicts()).getFeatures());
            features.addAll(featuresForNotams(request.getNotamRestrictions()).getFeatures());
            features.addAll(featuresForWeatherProducts(request.getWeatherProducts()).getFeatures());
            if (request.getWeatherAdvisories() != null) {
                for (RouteWeatherAdvisory advisory : request.getWeatherAdvisories()) {
                    features.addAll(featuresForWeatherAdvisory(advisory).getFeatures());
                }
            }
        }
        return AirspaceFeatureCollection.builder().features(features).build();
    }

    private AirspaceFeature featureForReservation(AirspaceReservation reservation) {
        Map<String, Object> properties = baseProperties("reservation", reservation.getId(),
                reservation.getStartTime(), reservation.getEffectiveDeconflictionEndTime(),
                reservation.getLowerAltitudeFeet(), reservation.getUpperAltitudeFeet());
        properties.put("reservationType", reservation.getReservationType());
        properties.put("routeStartFix", reservation.getRouteStartFix());
        properties.put("routeEndFix", reservation.getRouteEndFix());
        properties.put("sourceFixes", reservation.getSourceFixes());
        properties.put("routeGraphNodeIds", reservation.getRouteGraphNodeIds());
        properties.put("sourceRatioStart", reservation.getSourceRatioStart());
        properties.put("sourceRatioEnd", reservation.getSourceRatioEnd());
        properties.put("routeWidthNauticalMiles", reservation.getRouteWidthNauticalMiles());
        properties.put("routeSegmentDistanceNauticalMiles", reservation.getRouteSegmentDistanceNauticalMiles());
        properties.put("avanaMinutes", reservation.getAvanaMinutes());
        properties.put("longitudinalSeparationMinutes", reservation.getLongitudinalSeparationMinutes());
        properties.put("displayShapeIntent", reservation.getDisplayShapeIntent());
        properties.put("geometryIntent", reservation.getDisplayShapeIntent());
        properties.put("deconflictionShapeIntent", reservation.getDeconflictionShapeIntent());
        properties.put("sourceText", reservation.getSourceText());
        properties.put("diagnostics", reservation.getDiagnostics());
        properties.put("sourceFamily", "CARF_ALTRV");
        properties.put("displayLayer", "reservations");
        properties.put("constraintType", "CARF_RESERVATION");
        properties.put("operationalRole", "protected-volume");
        properties.put("isNotam", false);
        properties.put("isAltrv", true);
        properties.put("style", style("#1f78b4", "#1f78b4", 2.0, 0.18));

        return AirspaceFeature.builder()
                .id(reservation.getId())
                .geometry(geometryForReservation(reservation))
                .properties(properties)
                .build();
    }

    private AirspaceFeature featureForReservationRoute(AirspaceReservation reservation) {
        if (reservation == null || reservation.getRouteStart() == null || reservation.getRouteEnd() == null) {
            return null;
        }
        String id = reservation.getId() + "-route";
        Map<String, Object> properties = baseProperties("flight-path", id,
                reservation.getStartTime(), reservation.getEffectiveDeconflictionEndTime(),
                reservation.getLowerAltitudeFeet(), reservation.getUpperAltitudeFeet());
        properties.put("sourceFamily", "CARF_ALTRV");
        properties.put("displayLayer", "flight-paths");
        properties.put("constraintType", "CARF_ROUTE");
        properties.put("operationalRole", "route-centerline");
        properties.put("reservationId", reservation.getId());
        properties.put("routeStartFix", reservation.getRouteStartFix());
        properties.put("routeEndFix", reservation.getRouteEndFix());
        properties.put("sourceFixes", reservation.getSourceFixes());
        properties.put("routeGraphNodeIds", reservation.getRouteGraphNodeIds());
        properties.put("sourceRatioStart", reservation.getSourceRatioStart());
        properties.put("sourceRatioEnd", reservation.getSourceRatioEnd());
        properties.put("routeWidthNauticalMiles", reservation.getRouteWidthNauticalMiles());
        properties.put("routeSegmentDistanceNauticalMiles", reservation.getRouteSegmentDistanceNauticalMiles());
        properties.put("displayShapeIntent", reservation.getDisplayShapeIntent());
        properties.put("geometryIntent", reservation.getDisplayShapeIntent());
        properties.put("sourceText", reservation.getSourceText());
        properties.put("diagnostics", reservation.getDiagnostics());
        properties.put("polylineMergeKey", polylineMergeKey(reservation));
        properties.put("polylineSegmentKey", reservation.getRouteStartFix() + "->" + reservation.getRouteEndFix());
        properties.put("polylineSegmentIndex", polylineSegmentIndex(reservation.getId()));
        properties.put("style", style("#0891b2", "#0891b2", 3.0, 0.0));
        return AirspaceFeature.builder()
                .id(id)
                .geometry(lineGeometry(reservation.getRouteStart(), reservation.getRouteEnd()))
                .properties(properties)
                .build();
    }

    private String polylineMergeKey(AirspaceReservation reservation) {
        if (reservation.getRouteGraphNodeIds() != null && !reservation.getRouteGraphNodeIds().isEmpty()) {
            return String.join("|", reservation.getRouteGraphNodeIds());
        }
        String id = reservation.getId();
        int splitMarker = id == null ? -1 : id.lastIndexOf("-E");
        if (splitMarker > 0 && splitMarker + 2 < id.length() && allDigits(id.substring(splitMarker + 2))) {
            return id.substring(0, splitMarker);
        }
        return id;
    }

    private int polylineSegmentIndex(String id) {
        int splitMarker = id == null ? -1 : id.lastIndexOf("-E");
        if (splitMarker > 0 && splitMarker + 2 < id.length()) {
            String segment = id.substring(splitMarker + 2);
            if (allDigits(segment)) {
                return Integer.parseInt(segment);
            }
        }
        return 0;
    }

    private boolean allDigits(String value) {
        if (value == null || value.isEmpty()) {
            return false;
        }
        for (int i = 0; i < value.length(); i++) {
            if (!Character.isDigit(value.charAt(i))) {
                return false;
            }
        }
        return true;
    }

    private AirspaceFeature featureForWeather(WeatherHazardSnapshot snapshot) {
        HazardousWeather hazard = snapshot == null ? null : snapshot.getHazard();
        String id = hazard == null ? "weather-unknown" : hazard.getId();
        Map<String, Object> properties = baseProperties("weather", id,
                hazard instanceof WeatherCell ? ((WeatherCell) hazard).getStartTime() : null,
                hazard instanceof WeatherCell ? ((WeatherCell) hazard).getEndTime() : null,
                hazard instanceof WeatherCell ? ((WeatherCell) hazard).getMinAltitude() : 0,
                hazard instanceof WeatherCell ? ((WeatherCell) hazard).getMaxAltitude() : 0);
        if (hazard != null) {
            properties.put("hazardType", hazard.getType());
            properties.put("hazardSeverity", hazard.getSeverity());
            properties.put("costFactor", hazard.getCostFactor());
        }
        if (snapshot != null) {
            properties.put("productId", snapshot.getProductId());
            properties.put("sourceProduct", snapshot.getSourceProduct());
            properties.put("provider", snapshot.getProvider());
            properties.put("issuedAt", snapshot.getIssuedAt() == null ? null : snapshot.getIssuedAt().toString());
            properties.put("receivedAt", snapshot.getReceivedAt() == null ? null : snapshot.getReceivedAt().toString());
            properties.put("confidence", snapshot.getConfidence());
            properties.put("provenance", snapshot.getProvenance());
        }
        properties.put("sourceFamily", "WEATHER");
        properties.put("displayLayer", "weather");
        properties.put("constraintType", "WEATHER_HAZARD");
        properties.put("operationalRole", "weather-hazard");
        properties.put("style", style("#6a3d9a", "#6a3d9a", 2.0, 0.22));
        return AirspaceFeature.builder()
                .id(id)
                .geometry(geometryForWeather(hazard))
                .properties(properties)
                .build();
    }

    private AirspaceFeature featureForWeatherProduct(WeatherProduct product) {
        WeatherHazardSnapshot snapshot = product == null ? null : product.toSnapshotAt(product.getValidity() == null
                ? product.getIssuedAt()
                : product.getValidity().getValidStart());
        AirspaceFeature feature = featureForWeather(snapshot);
        Map<String, Object> properties = feature.getProperties();
        properties.put("featureKind", "weather-product");
        properties.put("displayLayer", "weather");
        properties.put("sourceFamily", "WEATHER");
        properties.put("constraintType", "WEATHER_PRODUCT");
        if (product != null) {
            properties.put("productType", product.getType());
            properties.put("productSource", product.getSource());
            properties.put("forecastHour", product.getForecastHour());
            properties.put("validStart", product.getValidity() == null || product.getValidity().getValidStart() == null
                    ? null : product.getValidity().getValidStart().toString());
            properties.put("validEnd", product.getValidity() == null || product.getValidity().getValidEnd() == null
                    ? null : product.getValidity().getValidEnd().toString());
            properties.put("latencySeconds", product.latency().getSeconds());
            properties.put("movementSpeedNauticalMilesPerHour", product.getMovement() == null
                    ? null : product.getMovement().getSpeedNauticalMilesPerHour());
            properties.put("movementBearingDegrees", product.getMovement() == null
                    ? null : product.getMovement().getBearingDegrees());
            properties.put("movementVerticalFeetPerMinute", product.getMovement() == null
                    ? null : product.getMovement().getVerticalFeetPerMinute());
        }
        return feature;
    }

    private AirspaceFeature featureForRouteWeatherIntersection(RouteHazardIntersection intersection, int index) {
        String id = "weather-intersection-" + index;
        Map<String, Object> properties = baseProperties("weather-intersection", id,
                intersection.getEstimatedEntryTime(), intersection.getEstimatedExitTime(),
                intersection.getEstimatedEntryPoint() == null ? 0 : intersection.getEstimatedEntryPoint().getAltitude(),
                intersection.getEstimatedExitPoint() == null ? 0 : intersection.getEstimatedExitPoint().getAltitude());
        properties.put("hazardId", intersection.getHazardId());
        properties.put("productId", intersection.getProductId());
        properties.put("hazardType", intersection.getHazardType());
        properties.put("hazardSeverity", intersection.getHazardSeverity());
        properties.put("segmentIndex", intersection.getSegmentIndex());
        properties.put("confidence", intersection.getConfidence());
        properties.put("productStatus", intersection.getProductStatus());
        properties.put("sourceProduct", intersection.getSourceProduct());
        properties.put("provenance", intersection.getProvenance());
        properties.put("forecastHour", intersection.getForecastHour());
        properties.put("productType", intersection.getProductType());
        properties.put("recommendedAction", intersection.getRecommendedAction());
        properties.put("rationale", intersection.getRationale());
        properties.put("movementSpeedNauticalMilesPerHour", intersection.getMovement() == null
                ? null : intersection.getMovement().getSpeedNauticalMilesPerHour());
        properties.put("movementBearingDegrees", intersection.getMovement() == null
                ? null : intersection.getMovement().getBearingDegrees());
        properties.put("sourceFamily", "WEATHER_ROUTE_IMPACT");
        properties.put("displayLayer", "route-impacts");
        properties.put("constraintType", "ROUTE_BLOCKAGE");
        properties.put("operationalRole", "impacted-route-segment");
        properties.put("style", style("#b15928", "#b15928", 3.0, 0.26));
        return AirspaceFeature.builder()
                .id(id)
                .geometry(lineGeometry(intersection.getEstimatedEntryPoint(), intersection.getEstimatedExitPoint()))
                .properties(properties)
                .build();
    }

    private AirspaceFeature featureForConflict(ReservationConflict conflict) {
        String id = conflict.getFirst().getId() + "__" + conflict.getSecond().getId();
        Map<String, Object> properties = baseProperties("conflict", id, conflict.getStartTime(), conflict.getEndTime(),
                Math.max(conflict.getFirst().getLowerAltitudeFeet(), conflict.getSecond().getLowerAltitudeFeet()),
                Math.min(conflict.getFirst().getUpperAltitudeFeet(), conflict.getSecond().getUpperAltitudeFeet()));
        properties.put("firstReservationId", conflict.getFirst().getId());
        properties.put("secondReservationId", conflict.getSecond().getId());
        properties.put("minimumLateralDistanceNauticalMiles", conflict.getMinimumLateralDistanceNauticalMiles());
        properties.put("longitudinalSeparationNauticalMiles", conflict.getLongitudinalSeparationNauticalMiles());
        properties.put("verticalSeparationFeet", conflict.getVerticalSeparationFeet());
        properties.put("requiredLateralSeparationNauticalMiles", conflict.getRequiredLateralSeparationNauticalMiles());
        properties.put("requiredLongitudinalSeparationNauticalMiles", conflict.getRequiredLongitudinalSeparationNauticalMiles());
        properties.put("requiredVerticalSeparationFeet", conflict.getRequiredVerticalSeparationFeet());
        properties.put("durationSeconds", conflict.getDurationSeconds());
        properties.put("firstConflictStartRatio", conflict.getFirstConflictStartRatio());
        properties.put("firstConflictEndRatio", conflict.getFirstConflictEndRatio());
        properties.put("secondConflictStartRatio", conflict.getSecondConflictStartRatio());
        properties.put("secondConflictEndRatio", conflict.getSecondConflictEndRatio());
        properties.put("firstFactoredStartRatio", conflict.getFirstFactoredStartRatio());
        properties.put("firstFactoredEndRatio", conflict.getFirstFactoredEndRatio());
        properties.put("secondFactoredStartRatio", conflict.getSecondFactoredStartRatio());
        properties.put("secondFactoredEndRatio", conflict.getSecondFactoredEndRatio());
        properties.put("distanceAtStartNauticalMiles", conflict.getDistanceAtStartNauticalMiles());
        properties.put("distanceAtEndNauticalMiles", conflict.getDistanceAtEndNauticalMiles());
        properties.put("angleBetweenRoutesDegrees", conflict.getAngleBetweenRoutesDegrees());
        properties.put("verticalSeparationMet", conflict.isVerticalSeparationMet());
        properties.put("lateralSeparationMet", conflict.isLateralSeparationMet());
        properties.put("longitudinalSeparationMet", conflict.isLongitudinalSeparationMet());
        properties.put("belowMinimumDuration", conflict.isBelowMinimumDuration());
        properties.put("explanation", conflict.getExplanation());
        properties.put("sourceFamily", "CARF_ALTRV");
        properties.put("displayLayer", "conflicts");
        properties.put("constraintType", "CARF_CONFLICT");
        properties.put("operationalRole", "deconfliction-warning");
        properties.put("style", style("#e31a1c", "#e31a1c", 3.0, 0.28));

        return AirspaceFeature.builder()
                .id(id)
                .geometry(lineGeometry(conflict.getFirstStartPoint(), conflict.getSecondStartPoint()))
                .properties(properties)
                .build();
    }

    private AirspaceFeature featureForNotam(NotamAirspaceRestriction restriction) {
        Map<String, Object> properties = baseProperties("notam", restriction.getId(), restriction.getEffectiveStart(),
                restriction.getEffectiveEnd(), restriction.getLowerAltitudeFeet(), restriction.getUpperAltitudeFeet());
        properties.put("notamType", restriction.getNotamType());
        properties.put("accountability", restriction.getAccountability());
        properties.put("affectedLocation", restriction.getAffectedLocation());
        properties.put("qCode", restriction.getQCode());
        properties.put("radiusNauticalMiles", restriction.getRadiusNauticalMiles());
        properties.put("description", restriction.getDescription());
        properties.put("sourceFamily", "NOTAM");
        properties.put("displayLayer", "notams");
        properties.put("constraintType", "NOTAM_RESTRICTION");
        properties.put("operationalRole", "airspace-restriction");
        properties.put("isNotam", true);
        properties.put("isAltrv", false);
        properties.put("style", style("#ff7f00", "#ff7f00", 2.0, 0.16));
        return AirspaceFeature.builder()
                .id(restriction.getId())
                .geometry(geometryForVolume(restriction.getVolume()))
                .properties(properties)
                .build();
    }

    private AirspaceGeometry geometryForReservation(AirspaceReservation reservation) {
        SpatialVolume volume = reservation.getProtectedVolume();
        if (volume != null) {
            return geometryForVolume(volume);
        }
        if (reservation.getRouteStart() != null && reservation.getRouteEnd() != null) {
            return lineGeometry(reservation.getRouteStart(), reservation.getRouteEnd());
        }
        return AirspaceGeometry.builder().type("GeometryCollection").coordinates(new ArrayList<>()).build();
    }

    private AirspaceGeometry geometryForVolume(SpatialVolume volume) {
        if (volume == null || volume.getBasePolygon() == null) {
            return AirspaceGeometry.builder().type("GeometryCollection").coordinates(new ArrayList<>()).build();
        }
        return polygonGeometry(volume.getBasePolygon());
    }

    private AirspaceGeometry polygonGeometry(SpatialPolygon polygon) {
        List<List<Double>> ring = new ArrayList<>();
        for (SpatialPoint vertex : polygon.getVertices()) {
            ring.add(position(vertex.getCoordinate()));
        }
        if (!ring.isEmpty() && !ring.get(0).equals(ring.get(ring.size() - 1))) {
            ring.add(new ArrayList<>(ring.get(0)));
        }
        List<List<List<Double>>> coordinates = new ArrayList<>();
        coordinates.add(ring);
        return AirspaceGeometry.builder().type("Polygon").coordinates(coordinates).build();
    }

    private AirspaceGeometry lineGeometry(GeoCoordinate first, GeoCoordinate second) {
        List<List<Double>> coordinates = new ArrayList<>();
        if (first != null) {
            coordinates.add(position(first));
        }
        if (second != null) {
            coordinates.add(position(second));
        }
        return AirspaceGeometry.builder().type("LineString").coordinates(coordinates).build();
    }

    private AirspaceGeometry geometryForWeather(HazardousWeather hazard) {
        if (hazard instanceof CircularWeatherCell) {
            CircularWeatherCell cell = (CircularWeatherCell) hazard;
            return polygonFromCoordinates(cell.getBoundaryPoints());
        }
        if (hazard instanceof WeatherCell) {
            return polygonFromCoordinates(((WeatherCell) hazard).getBoundaryPoints());
        }
        return AirspaceGeometry.builder().type("GeometryCollection").coordinates(new ArrayList<>()).build();
    }

    private AirspaceGeometry polygonFromCoordinates(List<GeoCoordinate> points) {
        List<List<Double>> ring = new ArrayList<>();
        if (points != null) {
            for (GeoCoordinate point : points) {
                ring.add(position(point));
            }
        }
        if (!ring.isEmpty() && !ring.get(0).equals(ring.get(ring.size() - 1))) {
            ring.add(new ArrayList<>(ring.get(0)));
        }
        List<List<List<Double>>> coordinates = new ArrayList<>();
        coordinates.add(ring);
        return AirspaceGeometry.builder().type("Polygon").coordinates(coordinates).build();
    }

    private Map<String, Object> baseProperties(String featureKind, String id, ZonedDateTime start, ZonedDateTime end,
                                               double lowerAltitudeFeet, double upperAltitudeFeet) {
        Map<String, Object> properties = new LinkedHashMap<>();
        properties.put("featureKind", featureKind);
        properties.put("id", id);
        properties.put("startTime", start == null ? null : start.toString());
        properties.put("endTime", end == null ? null : end.toString());
        properties.put("lowerAltitudeFeet", lowerAltitudeFeet);
        properties.put("upperAltitudeFeet", upperAltitudeFeet);
        return properties;
    }

    private AirspaceStyle style(String stroke, String fill, double strokeWidth, double fillOpacity) {
        return AirspaceStyle.builder()
                .stroke(stroke)
                .fill(fill)
                .strokeWidth(strokeWidth)
                .fillOpacity(fillOpacity)
                .build();
    }

    private List<Double> position(GeoCoordinate coordinate) {
        List<Double> position = new ArrayList<>();
        position.add(coordinate.getLongitude());
        position.add(coordinate.getLatitude());
        position.add(coordinate.getAltitude());
        return position;
    }
}
