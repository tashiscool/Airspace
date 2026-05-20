package org.tash.extensions.weather.decision;

import lombok.Builder;
import lombok.Data;
import org.tash.extensions.weather.HazardousWeather;
import org.tash.extensions.weather.product.WeatherMovementVector;
import org.tash.extensions.weather.product.WeatherProduct;
import org.tash.extensions.weather.product.WeatherProductType;

import java.time.Duration;
import java.time.ZonedDateTime;

@Data
@Builder
public class WeatherHazardSnapshot {
    private final WeatherProduct product;
    private final String productId;
    private final String sourceProduct;
    private final String provider;
    private final HazardousWeather hazard;
    private final ZonedDateTime issuedAt;
    private final ZonedDateTime receivedAt;
    private final double confidence;
    private final String provenance;
    private final Integer forecastHour;
    private final WeatherMovementVector movement;
    private final Double echoTopFeet;
    private final Double growthTrend;
    private final String stormPhase;

    public WeatherProductStatus statusAt(ZonedDateTime decisionTime, Duration staleAfter) {
        ZonedDateTime effectiveIssuedAt = getIssuedAt();
        double effectiveConfidence = getConfidence();
        if (decisionTime == null || effectiveIssuedAt == null) {
            return WeatherProductStatus.CURRENT;
        }
        if (effectiveIssuedAt.isAfter(decisionTime)) {
            return WeatherProductStatus.FUTURE;
        }
        if (staleAfter != null && effectiveIssuedAt.plus(staleAfter).isBefore(decisionTime)) {
            return WeatherProductStatus.STALE;
        }
        if (effectiveConfidence > 0.0 && effectiveConfidence < 0.5) {
            return WeatherProductStatus.LOW_CONFIDENCE;
        }
        return WeatherProductStatus.CURRENT;
    }

    public String getProductId() {
        return product != null && product.getId() != null ? product.getId() : productId;
    }

    public String getSourceProduct() {
        return product != null && product.getSourceProduct() != null ? product.getSourceProduct() : sourceProduct;
    }

    public String getProvider() {
        return product != null && product.getProvider() != null ? product.getProvider() : provider;
    }

    public ZonedDateTime getIssuedAt() {
        return product != null && product.getIssuedAt() != null ? product.getIssuedAt() : issuedAt;
    }

    public ZonedDateTime getReceivedAt() {
        return product != null && product.getReceivedAt() != null ? product.getReceivedAt() : receivedAt;
    }

    public double getConfidence() {
        return product != null && product.getConfidence() != null ? product.confidenceValue() : confidence;
    }

    public String getProvenance() {
        return product != null && product.getProvenance() != null ? product.getProvenance() : provenance;
    }

    public Integer getForecastHour() {
        return product != null && product.getForecastHour() != null ? product.getForecastHour() : forecastHour;
    }

    public WeatherMovementVector getMovement() {
        return product != null && product.getMovement() != null ? product.getMovement() : movement;
    }

    public Double getEchoTopFeet() {
        return product != null && product.getEchoTopFeet() != null ? product.getEchoTopFeet() : echoTopFeet;
    }

    public Double getGrowthTrend() {
        return product != null && product.getGrowthTrend() != null ? product.getGrowthTrend() : growthTrend;
    }

    public String getStormPhase() {
        return product != null && product.getStormPhase() != null ? product.getStormPhase() : stormPhase;
    }

    public WeatherProductType getProductType() {
        return product == null ? null : product.getType();
    }
}
