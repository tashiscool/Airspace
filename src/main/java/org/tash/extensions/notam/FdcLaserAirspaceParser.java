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

public class FdcLaserAirspaceParser {
    private static final Pattern HEADER = Pattern.compile("^!FDC\\s+(\\d+/\\d+)\\s+([A-Z0-9]{3})\\b",
            Pattern.CASE_INSENSITIVE | Pattern.DOTALL);
    private static final Pattern EFFECTIVE_UNTIL =
            Pattern.compile("\\bEFFECTIVE\\s+(\\d{10,12})\\s+UTC\\s+UNTIL\\s+(\\d{10,12})\\s+UTC\\b",
                    Pattern.CASE_INSENSITIVE | Pattern.DOTALL);
    private static final Pattern DMS_COORDINATE =
            Pattern.compile("\\b(\\d{6})([NS])\\s*/\\s*(\\d{7})([EW])\\b", Pattern.CASE_INSENSITIVE);
    private static final Pattern FEET_AND_BELOW =
            Pattern.compile("\\b(\\d{3,6})\\s+FEET\\s+AND\\s+BELOW\\b", Pattern.CASE_INSENSITIVE);
    private static final Pattern ACTIVITY_SENTENCE =
            Pattern.compile("(AIRBORNE\\s+TO\\s+GROUND\\s+LASER\\s+ACTIVITY.*?)(?:AVOID|THIS\\s+LASER|$)",
                    Pattern.CASE_INSENSITIVE | Pattern.DOTALL);

    public NotamAirspaceRestriction parse(String rawText) {
        if (rawText == null || rawText.trim().isEmpty()) {
            throw new IllegalArgumentException("FDC laser NOTAM text is required");
        }
        String normalized = rawText.replace('\r', '\n').replaceAll("\\s+", " ").trim();
        if (!normalized.toUpperCase().contains("AIRBORNE TO GROUND LASER ACTIVITY")) {
            throw new IllegalArgumentException("FDC NOTAM is not an airborne laser activity notice");
        }

        Matcher header = HEADER.matcher(normalized);
        if (!header.find()) {
            throw new IllegalArgumentException("FDC laser NOTAM is missing header");
        }

        Matcher coordinate = DMS_COORDINATE.matcher(normalized);
        if (!coordinate.find()) {
            throw new IllegalArgumentException("FDC laser NOTAM does not contain a parseable coordinate");
        }
        GeoCoordinate centerCoordinate = GeoCoordinate.builder()
                .latitude(parseDms(coordinate.group(1), coordinate.group(2)))
                .longitude(parseDms(coordinate.group(3), coordinate.group(4)))
                .altitude(0)
                .build();

        ZonedDateTime start = null;
        ZonedDateTime end = null;
        Matcher effective = EFFECTIVE_UNTIL.matcher(normalized);
        if (effective.find()) {
            start = parseTime(effective.group(1));
            end = parseTime(effective.group(2));
        }

        double upper = 100000;
        Matcher altitude = FEET_AND_BELOW.matcher(normalized);
        if (altitude.find()) {
            upper = Double.parseDouble(altitude.group(1));
        }

        SpatialPoint center = SpatialPoint.builder()
                .id("FDC-LASER-CENTER-" + UUID.randomUUID())
                .coordinate(centerCoordinate)
                .build();
        SpatialCircle circle = SpatialCircle.builder()
                .id("FDC-LASER-CIRCLE-" + UUID.randomUUID())
                .center(center)
                .radius(0)
                .build();
        SpatialPolygon polygon = circle.toPolygon(36);
        SpatialVolume volume = SpatialVolume.builder()
                .id("FDC-LASER-VOLUME-" + UUID.randomUUID())
                .basePolygon(polygon)
                .lowerAltitude(0)
                .upperAltitude(upper)
                .startTime(start)
                .endTime(end)
                .build();

        return NotamAirspaceRestriction.builder()
                .id("FDC-LASER-" + UUID.randomUUID())
                .notamType("FDC")
                .accountability(header.group(2).toUpperCase())
                .affectedLocation(extractAffectedLocation(normalized, header.end()))
                .qCode("FDC-LASER")
                .effectiveStart(start)
                .effectiveEnd(end)
                .lowerAltitudeFeet(0)
                .upperAltitudeFeet(upper)
                .centerLatitude(centerCoordinate.getLatitude())
                .centerLongitude(centerCoordinate.getLongitude())
                .radiusNauticalMiles(0)
                .description(extractDescription(normalized))
                .volume(volume)
                .build();
    }

    private double parseDms(String digits, String hemisphere) {
        int degreeDigits = digits.length() == 6 ? 2 : 3;
        int degrees = Integer.parseInt(digits.substring(0, degreeDigits));
        int minutes = Integer.parseInt(digits.substring(degreeDigits, degreeDigits + 2));
        int seconds = Integer.parseInt(digits.substring(degreeDigits + 2));
        double decimal = degrees + minutes / 60.0 + seconds / 3600.0;
        return "S".equalsIgnoreCase(hemisphere) || "W".equalsIgnoreCase(hemisphere) ? -decimal : decimal;
    }

    private ZonedDateTime parseTime(String value) {
        int year = 2000 + Integer.parseInt(value.substring(0, 2));
        int month = Integer.parseInt(value.substring(2, 4));
        int day = Integer.parseInt(value.substring(4, 6));
        int hour = Integer.parseInt(value.substring(6, 8));
        int minute = Integer.parseInt(value.substring(8, 10));
        int second = value.length() >= 12 ? Integer.parseInt(value.substring(10, 12)) : 0;
        return ZonedDateTime.of(year, month, day, hour, minute, second, 0, ZoneOffset.UTC);
    }

    private String extractAffectedLocation(String text, int start) {
        int activity = text.toUpperCase().indexOf("AIRBORNE TO GROUND LASER ACTIVITY", start);
        if (activity <= start) {
            return null;
        }
        return text.substring(start, activity).replace(".", "").trim();
    }

    private String extractDescription(String text) {
        Matcher matcher = ACTIVITY_SENTENCE.matcher(text);
        return matcher.find() ? matcher.group(1).trim() : text;
    }
}
