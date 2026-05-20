package org.tash.extensions.weather.product;

public enum WeatherDiagnosticType {
    STALE,
    FUTURE_ISSUED,
    EXPIRED,
    LOW_CONFIDENCE,
    MISSING_VALIDITY,
    MISSING_GEOMETRY,
    MISSING_ALTITUDE_BAND,
    MISSING_PROVENANCE
}
