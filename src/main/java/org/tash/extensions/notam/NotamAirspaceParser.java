package org.tash.extensions.notam;

import org.tash.data.GeoCoordinate;
import org.tash.spatial.SpatialCircle;
import org.tash.spatial.SpatialPoint;
import org.tash.spatial.SpatialPolygon;
import org.tash.spatial.SpatialVolume;

import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.UUID;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Lightweight parser for ICAO NOTAM fields that carry airspace applicability.
 *
 * The legacy 2010 NAIMES/USNS code treated Q-code, A/B/C/D/E/F/G fields as the
 * canonical NOTAM structure. This parser keeps that contract but returns
 * project-native spatial models instead of database-backed FAA message objects.
 */
public class NotamAirspaceParser {
    private static final Pattern NOTAM_TYPE = Pattern.compile("\\b(NOTAM[NRC])\\b", Pattern.CASE_INSENSITIVE);
    private static final Pattern FIELD_MARKER = Pattern.compile("(?m)(^|\\s)(Q|A|B|C|D|E|F|G)\\)\\s*");
    private static final Pattern Q_COORDINATE = Pattern.compile("(\\d{4}[NS]\\d{5}[EW])(\\d{3})?");

    public NotamAirspaceRestriction parse(String rawNotam) {
        if (rawNotam == null || rawNotam.trim().isEmpty()) {
            throw new IllegalArgumentException("NOTAM text is required");
        }

        String normalized = rawNotam.replace("\r", "").trim();
        String qField = field(normalized, "Q");
        String aField = field(normalized, "A");
        String bField = field(normalized, "B");
        String cField = field(normalized, "C");
        String dField = field(normalized, "D");
        String eField = field(normalized, "E");
        String fField = field(normalized, "F");
        String gField = field(normalized, "G");

        String[] qParts = qField == null ? new String[0] : qField.split("/", -1);
        String accountability = qParts.length > 0 ? emptyToNull(qParts[0]) : null;
        String qCode = qParts.length > 1 ? emptyToNull(qParts[1]) : null;
        String traffic = qParts.length > 2 ? emptyToNull(qParts[2]) : null;
        String purpose = qParts.length > 3 ? emptyToNull(qParts[3]) : null;
        String scope = qParts.length > 4 ? emptyToNull(qParts[4]) : null;
        double lower = qParts.length > 5 ? altitudeFromFlightLevel(qParts[5]) : altitudeFromText(fField, 0);
        double upper = qParts.length > 6 ? altitudeFromFlightLevel(qParts[6]) : altitudeFromText(gField, 100000);

        CoordinateRadius coordinateRadius = extractCoordinateRadius(qField, eField);
        if (coordinateRadius == null) {
            throw new IllegalArgumentException("NOTAM does not include a Q-field coordinate/radius");
        }

        ZonedDateTime start = parseNotamTime(bField, ZonedDateTime.now(ZoneOffset.UTC));
        ZonedDateTime end = parseNotamTime(cField, start.plusYears(1));
        if (end.isBefore(start)) {
            throw new IllegalArgumentException("NOTAM end time is before start time");
        }

        SpatialPoint center = SpatialPoint.builder()
                .id("NOTAM-CENTER-" + UUID.randomUUID())
                .coordinate(GeoCoordinate.builder()
                        .latitude(coordinateRadius.latitude)
                        .longitude(coordinateRadius.longitude)
                        .altitude(lower)
                        .build())
                .build();
        SpatialCircle circle = SpatialCircle.builder()
                .id("NOTAM-CIRCLE-" + UUID.randomUUID())
                .center(center)
                .radius(coordinateRadius.radiusNauticalMiles)
                .build();
        SpatialPolygon polygon = circle.toPolygon(36);
        SpatialVolume volume = SpatialVolume.builder()
                .id("NOTAM-VOLUME-" + UUID.randomUUID())
                .basePolygon(polygon)
                .lowerAltitude(lower)
                .upperAltitude(upper)
                .startTime(start)
                .endTime(end)
                .build();

        return NotamAirspaceRestriction.builder()
                .id("NOTAM-" + UUID.randomUUID())
                .notamType(extractNotamType(normalized))
                .accountability(accountability)
                .affectedLocation(emptyToNull(aField))
                .qCode(qCode)
                .traffic(traffic)
                .purpose(purpose)
                .scope(scope)
                .effectiveStart(start)
                .effectiveEnd(end)
                .lowerAltitudeFeet(lower)
                .upperAltitudeFeet(upper)
                .centerLatitude(coordinateRadius.latitude)
                .centerLongitude(coordinateRadius.longitude)
                .radiusNauticalMiles(coordinateRadius.radiusNauticalMiles)
                .scheduleText(emptyToNull(dField))
                .description(emptyToNull(eField))
                .volume(volume)
                .build();
    }

    private String field(String text, String name) {
        Matcher matcher = FIELD_MARKER.matcher(text);
        while (matcher.find()) {
            if (!matcher.group(2).equalsIgnoreCase(name)) {
                continue;
            }
            int start = matcher.end();
            int end = text.length();
            Matcher next = FIELD_MARKER.matcher(text);
            if (next.find(start)) {
                end = next.start();
            }
            return text.substring(start, end).trim();
        }
        return null;
    }

    private CoordinateRadius extractCoordinateRadius(String qField, String eField) {
        CoordinateRadius fromQ = extractCoordinateRadius(qField);
        return fromQ != null ? fromQ : extractCoordinateRadius(eField);
    }

    private CoordinateRadius extractCoordinateRadius(String text) {
        if (text == null) {
            return null;
        }
        Matcher matcher = Q_COORDINATE.matcher(text.replace(" ", ""));
        if (!matcher.find()) {
            return null;
        }
        String coordinate = matcher.group(1);
        double radius = matcher.group(2) == null ? 0 : Double.parseDouble(matcher.group(2));
        return new CoordinateRadius(parseLatitude(coordinate.substring(0, 5)),
                parseLongitude(coordinate.substring(5, 11)),
                radius);
    }

    private double parseLatitude(String value) {
        int degrees = Integer.parseInt(value.substring(0, 2));
        int minutes = Integer.parseInt(value.substring(2, 4));
        double decimal = degrees + minutes / 60.0;
        return value.charAt(4) == 'S' ? -decimal : decimal;
    }

    private double parseLongitude(String value) {
        int degrees = Integer.parseInt(value.substring(0, 3));
        int minutes = Integer.parseInt(value.substring(3, 5));
        double decimal = degrees + minutes / 60.0;
        return value.charAt(5) == 'W' ? -decimal : decimal;
    }

    private ZonedDateTime parseNotamTime(String value, ZonedDateTime fallback) {
        String text = emptyToNull(value);
        if (text == null || "WIE".equalsIgnoreCase(text)) {
            return fallback;
        }
        if ("UFN".equalsIgnoreCase(text) || "PERM".equalsIgnoreCase(text)) {
            return fallback.plusYears(100);
        }
        Matcher matcher = Pattern.compile("(\\d{10})").matcher(text);
        if (!matcher.find()) {
            return fallback;
        }
        String digits = matcher.group(1);
        int year = 2000 + Integer.parseInt(digits.substring(0, 2));
        int month = Integer.parseInt(digits.substring(2, 4));
        int day = Integer.parseInt(digits.substring(4, 6));
        int hour = Integer.parseInt(digits.substring(6, 8));
        int minute = Integer.parseInt(digits.substring(8, 10));
        if (hour == 24) {
            hour = 23;
            minute = 59;
        }
        return ZonedDateTime.of(year, month, day, hour, minute, 0, 0, ZoneOffset.UTC);
    }

    private double altitudeFromFlightLevel(String value) {
        String text = emptyToNull(value);
        if (text == null) {
            return 0;
        }
        return Integer.parseInt(text.replaceAll("[^0-9]", "")) * 100.0;
    }

    private double altitudeFromText(String value, double fallback) {
        String text = emptyToNull(value);
        if (text == null) {
            return fallback;
        }
        String upper = text.toUpperCase();
        if (upper.contains("GND") || upper.contains("SFC")) {
            return 0;
        }
        if (upper.contains("UNL")) {
            return 100000;
        }
        Matcher fl = Pattern.compile("FL\\s*(\\d{2,3})").matcher(upper);
        if (fl.find()) {
            return Integer.parseInt(fl.group(1)) * 100.0;
        }
        Matcher feet = Pattern.compile("(\\d{1,6})\\s*(FT|MSL|AGL)?").matcher(upper);
        if (feet.find()) {
            return Double.parseDouble(feet.group(1));
        }
        return fallback;
    }

    private String extractNotamType(String text) {
        Matcher matcher = NOTAM_TYPE.matcher(text);
        return matcher.find() ? matcher.group(1).toUpperCase() : null;
    }

    private String emptyToNull(String value) {
        if (value == null) {
            return null;
        }
        String trimmed = value.trim();
        return trimmed.isEmpty() ? null : trimmed;
    }

    private static class CoordinateRadius {
        private final double latitude;
        private final double longitude;
        private final double radiusNauticalMiles;

        private CoordinateRadius(double latitude, double longitude, double radiusNauticalMiles) {
            this.latitude = latitude;
            this.longitude = longitude;
            this.radiusNauticalMiles = radiusNauticalMiles;
        }
    }
}
