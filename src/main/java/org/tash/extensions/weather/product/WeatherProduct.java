package org.tash.extensions.weather.product;

import lombok.Builder;
import lombok.Data;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.weather.HazardSeverity;
import org.tash.extensions.weather.HazardousWeather;
import org.tash.extensions.weather.avoid.CircularWeatherCell;
import org.tash.extensions.weather.avoid.PolygonalWeatherCell;
import org.tash.extensions.weather.avoid.WeatherCell;
import org.tash.extensions.weather.decision.WeatherDecisionSeverity;
import org.tash.extensions.weather.decision.WeatherHazardSnapshot;

import java.time.Duration;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Data
@Builder(toBuilder = true)
public class WeatherProduct {
    private final String id;
    private final WeatherProductType type;
    private final WeatherProductSource source;
    private final String provider;
    private final String sourceProduct;
    private final String rawText;
    private final WeatherValidityWindow validity;
    private final ZonedDateTime issuedAt;
    private final ZonedDateTime receivedAt;
    private final Integer forecastHour;
    private final WeatherConfidence confidence;
    private final String provenance;
    private final HazardousWeather hazard;
    private final List<GeoCoordinate> geometry;
    private final Double lowerAltitudeFeet;
    private final Double upperAltitudeFeet;
    private final WeatherMovementVector movement;
    private final Double echoTopFeet;
    private final Double growthTrend;
    private final String stormPhase;
    private final Double ceilingFeet;
    private final Double visibilityStatuteMiles;
    private final String stationId;
    private final Integer windDirectionDegrees;
    private final Double windSpeedKnots;
    private final Double windGustKnots;
    private final Double altimeterInchesHg;
    private final Double temperatureCelsius;
    private final Double dewpointCelsius;
    private final Double runwayVisualRangeFeet;
    private final Double lineWidthNauticalMiles;
    private final Boolean amended;
    private final Boolean corrected;
    private final Integer variableWindFromDegrees;
    private final Integer variableWindToDegrees;
    private final String remarks;
    @Builder.Default
    private final List<String> cloudLayers = new ArrayList<>();
    @Builder.Default
    private final List<String> forecastChangeGroups = new ArrayList<>();
    @Builder.Default
    private final List<String> weatherPhenomena = new ArrayList<>();
    @Builder.Default
    private final List<WeatherForecastSlice> forecastSlices = new ArrayList<>();

    public double confidenceValue() {
        return confidence == null ? 0.0 : confidence.normalized();
    }

    public Duration latency() {
        if (issuedAt == null || receivedAt == null) {
            return Duration.ZERO;
        }
        return Duration.between(issuedAt, receivedAt);
    }

    public Duration ageAt(ZonedDateTime decisionTime) {
        if (issuedAt == null || decisionTime == null) {
            return Duration.ZERO;
        }
        return Duration.between(issuedAt, decisionTime);
    }

    public List<GeoCoordinate> getGeometry() {
        if (geometry != null && !geometry.isEmpty()) {
            return Collections.unmodifiableList(geometry);
        }
        if (hazard instanceof WeatherCell) {
            return ((WeatherCell) hazard).getBoundaryPoints();
        }
        return Collections.emptyList();
    }

    public List<String> getWeatherPhenomena() {
        return Collections.unmodifiableList(weatherPhenomena == null ? Collections.emptyList() : weatherPhenomena);
    }

    public List<String> getForecastChangeGroups() {
        return Collections.unmodifiableList(forecastChangeGroups == null ? Collections.emptyList() : forecastChangeGroups);
    }

    public List<WeatherForecastSlice> getForecastSlices() {
        return Collections.unmodifiableList(forecastSlices == null ? Collections.emptyList() : forecastSlices);
    }

    public List<String> getCloudLayers() {
        return Collections.unmodifiableList(cloudLayers == null ? Collections.emptyList() : cloudLayers);
    }

    public List<WeatherProduct> expandForecastSlices() {
        if (forecastSlices == null || forecastSlices.isEmpty()) {
            return Collections.singletonList(this);
        }
        List<WeatherProduct> products = new ArrayList<>();
        for (WeatherForecastSlice slice : forecastSlices) {
            WeatherValidityWindow sliceValidity = WeatherValidityWindow.builder()
                    .validStart(slice.getValidStart())
                    .validEnd(slice.getValidEnd())
                    .build();
            products.add(toBuilder()
                    .id(id + "#slice-" + slice.getSequence())
                    .validity(sliceValidity)
                    .forecastHour(slice.getForecastHour())
                    .ceilingFeet(slice.getCeilingFeet() == null ? ceilingFeet : slice.getCeilingFeet())
                    .visibilityStatuteMiles(slice.getVisibilityStatuteMiles() == null ? visibilityStatuteMiles : slice.getVisibilityStatuteMiles())
                    .confidence(slice.getConfidence() == null ? confidence : WeatherConfidence.builder()
                            .value(slice.getConfidence())
                            .basis("forecast slice " + slice.getGroupType())
                            .build())
                    .weatherPhenomena(slice.getWeatherPhenomena().isEmpty() ? weatherPhenomena : new ArrayList<>(slice.getWeatherPhenomena()))
                    .geometry(slice.getGeometry().isEmpty() ? geometry : new ArrayList<>(slice.getGeometry()))
                    .build());
        }
        return products;
    }

    public List<WeatherProductDiagnostic> diagnosticsAt(ZonedDateTime decisionTime, Duration staleAfter) {
        List<WeatherProductDiagnostic> diagnostics = new ArrayList<>();
        if (validity == null) {
            diagnostics.add(diagnostic(WeatherDiagnosticType.MISSING_VALIDITY, WeatherDecisionSeverity.ADVISORY,
                    "Weather product " + id + " has no validity window"));
        } else if (validity.isExpiredAt(decisionTime)) {
            diagnostics.add(diagnostic(WeatherDiagnosticType.EXPIRED, WeatherDecisionSeverity.WARNING,
                    "Weather product " + id + " is expired"));
        }
        if (issuedAt != null && decisionTime != null && issuedAt.isAfter(decisionTime)) {
            diagnostics.add(diagnostic(WeatherDiagnosticType.FUTURE_ISSUED, WeatherDecisionSeverity.WARNING,
                    "Weather product " + id + " is issued after decision time"));
        }
        if (issuedAt != null && decisionTime != null && staleAfter != null
                && issuedAt.plus(staleAfter).isBefore(decisionTime)) {
            diagnostics.add(diagnostic(WeatherDiagnosticType.STALE, WeatherDecisionSeverity.ADVISORY,
                    "Weather product " + id + " is stale"));
        }
        if (confidence != null && confidence.isLow()) {
            diagnostics.add(diagnostic(WeatherDiagnosticType.LOW_CONFIDENCE, WeatherDecisionSeverity.ADVISORY,
                    "Weather product " + id + " has low confidence"));
        }
        if (getGeometry().isEmpty() && hazard == null) {
            diagnostics.add(diagnostic(WeatherDiagnosticType.MISSING_GEOMETRY, WeatherDecisionSeverity.ADVISORY,
                    "Weather product " + id + " has no geometry or hazard"));
        }
        if (lowerAltitudeFeet == null && upperAltitudeFeet == null && !(hazard instanceof WeatherCell)) {
            diagnostics.add(diagnostic(WeatherDiagnosticType.MISSING_ALTITUDE_BAND, WeatherDecisionSeverity.INFO,
                    "Weather product " + id + " has no altitude band"));
        }
        if (provenance == null || provenance.trim().isEmpty()) {
            diagnostics.add(diagnostic(WeatherDiagnosticType.MISSING_PROVENANCE, WeatherDecisionSeverity.INFO,
                    "Weather product " + id + " has no provenance"));
        }
        return diagnostics;
    }

    public WeatherHazardSnapshot toSnapshotAt(ZonedDateTime decisionTime) {
        HazardousWeather movedHazard = movedHazard(decisionTime);
        return WeatherHazardSnapshot.builder()
                .product(this)
                .productId(id)
                .sourceProduct(sourceProduct)
                .provider(provider)
                .hazard(movedHazard)
                .issuedAt(issuedAt)
                .receivedAt(receivedAt)
                .confidence(confidenceValue())
                .provenance(provenance)
                .forecastHour(forecastHour)
                .movement(movement)
                .echoTopFeet(echoTopFeet)
                .growthTrend(growthTrend)
                .stormPhase(stormPhase)
                .build();
    }

    private HazardousWeather movedHazard(ZonedDateTime decisionTime) {
        if (movement == null || hazard == null || validity == null || validity.getValidStart() == null || decisionTime == null) {
            return hazard;
        }
        Duration elapsed = Duration.between(validity.getValidStart(), decisionTime);
        if (elapsed.isNegative() || elapsed.isZero()) {
            return hazard;
        }
        if (hazard instanceof CircularWeatherCell) {
            CircularWeatherCell cell = (CircularWeatherCell) hazard;
            return CircularWeatherCell.builder()
                    .id(cell.getId())
                    .type(cell.getType())
                    .severity(cell.getSeverity())
                    .startTime(cell.getStartTime())
                    .endTime(cell.getEndTime())
                    .minAltitude(cell.getMinAltitude())
                    .maxAltitude(cell.getMaxAltitude())
                    .center(movement.project(cell.getCenter(), elapsed))
                    .radius(cell.getRadius())
                    .build();
        }
        if (hazard instanceof PolygonalWeatherCell) {
            PolygonalWeatherCell cell = (PolygonalWeatherCell) hazard;
            List<GeoCoordinate> moved = new ArrayList<>();
            for (GeoCoordinate point : cell.getVertices()) {
                moved.add(movement.project(point, elapsed));
            }
            return PolygonalWeatherCell.builder()
                    .id(cell.getId())
                    .type(cell.getType())
                    .severity(cell.getSeverity())
                    .startTime(cell.getStartTime())
                    .endTime(cell.getEndTime())
                    .minAltitude(cell.getMinAltitude())
                    .maxAltitude(cell.getMaxAltitude())
                    .vertices(moved)
                    .build();
        }
        return hazard;
    }

    private WeatherProductDiagnostic diagnostic(WeatherDiagnosticType type,
                                                WeatherDecisionSeverity severity,
                                                String message) {
        return WeatherProductDiagnostic.builder()
                .type(type)
                .severity(severity)
                .message(message)
                .build();
    }

    public static WeatherProduct fromHazard(String id,
                                            WeatherProductType productType,
                                            HazardousWeather hazard,
                                            ZonedDateTime issuedAt,
                                            ZonedDateTime receivedAt,
                                            double confidence,
                                            String provenance) {
        WeatherValidityWindow validity = null;
        Double lower = null;
        Double upper = null;
        if (hazard instanceof WeatherCell) {
            WeatherCell cell = (WeatherCell) hazard;
            validity = WeatherValidityWindow.builder()
                    .validStart(cell.getStartTime())
                    .validEnd(cell.getEndTime())
                    .build();
            lower = cell.getMinAltitude();
            upper = cell.getMaxAltitude();
        }
        return WeatherProduct.builder()
                .id(id)
                .type(productType)
                .source(WeatherProductSource.UNKNOWN)
                .sourceProduct(productType == null ? null : productType.name())
                .hazard(hazard)
                .validity(validity)
                .issuedAt(issuedAt)
                .receivedAt(receivedAt)
                .confidence(WeatherConfidence.builder().value(confidence).basis("source").build())
                .provenance(provenance)
                .lowerAltitudeFeet(lower)
                .upperAltitudeFeet(upper)
                .build();
    }
}
