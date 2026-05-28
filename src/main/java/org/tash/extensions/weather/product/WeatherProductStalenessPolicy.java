package org.tash.extensions.weather.product;

import java.time.Duration;
import java.time.ZonedDateTime;
import java.util.Locale;

/**
 * Product-family freshness policy for decision-support warnings. Values are
 * conservative prototype defaults, not certified meteorological validity.
 */
public class WeatherProductStalenessPolicy {
    public Duration staleAfter(WeatherProduct product) {
        if (product == null) {
            return Duration.ofMinutes(90);
        }
        WeatherProductType type = product.getType();
        String raw = String.valueOf(product.getRawText()).toUpperCase(Locale.US);
        if (type == WeatherProductType.TAF || raw.startsWith("TAF ")) {
            return Duration.ofHours(6);
        }
        if (type == WeatherProductType.AIRMET || raw.contains("G-AIRMET") || raw.contains("G AIRMET")) {
            return Duration.ofHours(4);
        }
        if (type == WeatherProductType.SIGMET || raw.contains("SIGMET")) {
            return Duration.ofHours(2);
        }
        if (type == WeatherProductType.NEXRAD_POLYGON || raw.contains("CWA") || raw.contains("CENTER WEATHER ADVISORY")
                || raw.contains("CWAP") || raw.contains("CWAF")) {
            return Duration.ofMinutes(45);
        }
        if (type == WeatherProductType.PIREP_DERIVED || raw.contains("PIREP") || raw.startsWith("UA ") || raw.startsWith("UUA ")) {
            return Duration.ofMinutes(60);
        }
        if (type == WeatherProductType.METAR || raw.startsWith("METAR ") || raw.startsWith("SPECI ")) {
            return Duration.ofMinutes(90);
        }
        return Duration.ofMinutes(90);
    }

    public boolean isStale(WeatherProduct product, ZonedDateTime decisionTime) {
        if (product == null || decisionTime == null) {
            return false;
        }
        ZonedDateTime basis = product.getIssuedAt() == null ? product.getReceivedAt() : product.getIssuedAt();
        return basis != null && basis.plus(staleAfter(product)).isBefore(decisionTime);
    }
}
