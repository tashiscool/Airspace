package org.tash.extensions.feed;

import java.util.Locale;

public final class FlightTrackingSourceNormalizer {
    private FlightTrackingSourceNormalizer() {
    }

    public static FlightTrackingSourceType normalize(String raw) {
        if (raw == null || raw.isBlank()) {
            return FlightTrackingSourceType.UNKNOWN;
        }
        String value = raw.trim().toUpperCase(Locale.US)
                .replace('-', '_')
                .replace(' ', '_');
        if (value.contains("SAT") && value.contains("ADS")) {
            return FlightTrackingSourceType.SATELLITE_ADSB;
        }
        if (value.equals("ADSB") || value.equals("ADS_B") || value.contains("TERRESTRIAL_ADSB")) {
            return FlightTrackingSourceType.ADS_B;
        }
        if (value.contains("MLAT") || value.contains("MULTILATERATION")) {
            return FlightTrackingSourceType.MLAT;
        }
        if (value.contains("RADAR")) {
            return FlightTrackingSourceType.RADAR;
        }
        if (value.contains("UAT")) {
            return FlightTrackingSourceType.UAT;
        }
        if (value.contains("FLARM") || value.contains("OGN")) {
            return FlightTrackingSourceType.FLARM;
        }
        if (value.contains("ADS_C") || value.contains("ADSC")) {
            return FlightTrackingSourceType.ADS_C;
        }
        if (value.contains("SWIM") || value.contains("TFMS") || value.contains("FAA")) {
            return FlightTrackingSourceType.FAA_SWIM;
        }
        if (value.contains("MULTI") || value.contains("FUSED")) {
            return FlightTrackingSourceType.MULTISOURCE;
        }
        return FlightTrackingSourceType.UNKNOWN;
    }

    public static String label(String raw) {
        FlightTrackingSourceType type = normalize(raw);
        if (type == FlightTrackingSourceType.UNKNOWN && raw != null && !raw.isBlank()) {
            return raw.trim();
        }
        return switch (type) {
            case ADS_B -> "ADS-B terrestrial";
            case SATELLITE_ADSB -> "Satellite ADS-B";
            case MLAT -> "MLAT";
            case RADAR -> "Radar";
            case UAT -> "UAT";
            case FLARM -> "FLARM";
            case ADS_C -> "ADS-C";
            case FAA_SWIM -> "FAA SWIM/TFMS";
            case MULTISOURCE -> "Multisource fused";
            case UNKNOWN -> "Unknown tracking source";
        };
    }
}
