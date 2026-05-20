package org.tash.extensions.reservation;

import org.tash.data.GeoCoordinate;

import java.time.ZonedDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class ReservationMessageParser {
    private static final DateTimeFormatter DATE_FORMAT =
            DateTimeFormatter.ofPattern("EEE MMM dd HH:mm:ss z yyyy", Locale.US);
    private static final Pattern RESERVATION_BLOCK =
            Pattern.compile("reservation\\[(\\d+)](.*?)(?=\\nreservation\\[|\\z)", Pattern.DOTALL);
    private static final Pattern POINTS =
            Pattern.compile("points\\[(\\d{7}[EW])\\s+(\\d{6}[NS])-(\\d{7}[EW])\\s+(\\d{6}[NS])]");

    public List<AirspaceReservation> parse(String messageName, String text) {
        List<AirspaceReservation> reservations = new ArrayList<>();
        Matcher matcher = RESERVATION_BLOCK.matcher(text.replace("\r", ""));
        while (matcher.find()) {
            String index = matcher.group(1);
            String block = matcher.group(2);
            Matcher points = POINTS.matcher(block);
            if (!points.find()) {
                throw new IllegalArgumentException("Reservation " + index + " is missing SOURCE points");
            }

            double[] deconflictionRange = parseRange(valueAfter(block, "VERT RANGE FOR DECONFLICTION"));
            double[] verticalRange = parseFlightLevelRange(valueAfter(block, "VERT RANGE"));

            reservations.add(AirspaceReservation.builder()
                    .id(messageName + "-R" + index)
                    .startTime(parseDate(valueAfter(block, "START DATE")))
                    .endTime(parseDate(valueAfter(block, "END DATE")))
                    .deconflictionEndTime(parseOptionalDate(optionalValueAfter(block, "END DATE W/AVANA")))
                    .lowerAltitudeFeet(verticalRange[0])
                    .upperAltitudeFeet(verticalRange[1])
                    .deconflictionLowerAltitudeFeet(deconflictionRange[0])
                    .deconflictionUpperAltitudeFeet(deconflictionRange[1])
                    .verticalSeparationFeet(parseDouble(valueAfter(block, "VERT SEPARATION")))
                    .lateralSeparationNauticalMiles(parseLateralSeparation(block))
                    .longitudinalSeparationNauticalMiles(optionalSourceMetric(block, "longitudinalSep", 0))
                    .routeStart(coordinate(points.group(2), points.group(1)))
                    .routeEnd(coordinate(points.group(4), points.group(3)))
                    .build());
        }
        return reservations;
    }

    private String valueAfter(String block, String label) {
        String value = optionalValueAfter(block, label);
        if (value == null) {
            throw new IllegalArgumentException("Missing field: " + label);
        }
        return value;
    }

    private String optionalValueAfter(String block, String label) {
        Pattern pattern = Pattern.compile("(?m)^" + Pattern.quote(label) + "\\s*\\n([^\\n]+)");
        Matcher matcher = pattern.matcher(block);
        if (!matcher.find()) {
            return null;
        }
        return matcher.group(1).trim();
    }

    private ZonedDateTime parseDate(String value) {
        return ZonedDateTime.parse(value, DATE_FORMAT);
    }

    private ZonedDateTime parseOptionalDate(String value) {
        return value == null ? null : parseDate(value);
    }

    private double[] parseFlightLevelRange(String value) {
        Matcher matcher = Pattern.compile("FL(\\d+)B(\\d+)").matcher(value);
        if (!matcher.find()) {
            throw new IllegalArgumentException("Invalid vertical range: " + value);
        }
        return new double[]{Double.parseDouble(matcher.group(1)) * 100,
                Double.parseDouble(matcher.group(2)) * 100};
    }

    private double[] parseRange(String value) {
        String[] parts = value.split("-");
        return new double[]{Double.parseDouble(parts[0]), Double.parseDouble(parts[1])};
    }

    private double parseSourceMetric(String block, String name) {
        Matcher matcher = Pattern.compile(name + "\\s+([0-9.]+)").matcher(block);
        if (!matcher.find()) {
            throw new IllegalArgumentException("Missing SOURCE metric: " + name);
        }
        return Double.parseDouble(matcher.group(1));
    }

    private double optionalSourceMetric(String block, String name, double fallback) {
        Matcher matcher = Pattern.compile(name + "\\s+([0-9.]+)").matcher(block);
        return matcher.find() ? Double.parseDouble(matcher.group(1)) : fallback;
    }

    private double parseLateralSeparation(String block) {
        Matcher lateral = Pattern.compile("lateralSep\\s+([0-9.]+)").matcher(block);
        if (lateral.find()) {
            return Double.parseDouble(lateral.group(1));
        }

        Matcher horizontalMeters = Pattern.compile("horSep\\s+([0-9.]+)").matcher(block);
        if (horizontalMeters.find()) {
            return Double.parseDouble(horizontalMeters.group(1)) / 1852.0;
        }

        throw new IllegalArgumentException("Missing SOURCE metric: lateralSep or horSep");
    }

    private double parseDouble(String value) {
        return Double.parseDouble(value);
    }

    private GeoCoordinate coordinate(String latitude, String longitude) {
        return GeoCoordinate.builder()
                .latitude(parseLatitude(latitude))
                .longitude(parseLongitude(longitude))
                .altitude(0)
                .build();
    }

    private double parseLatitude(String value) {
        int degrees = Integer.parseInt(value.substring(0, 2));
        int minutes = Integer.parseInt(value.substring(2, 4));
        int seconds = Integer.parseInt(value.substring(4, 6));
        minutes += seconds / 60;
        seconds = seconds % 60;
        double decimal = degrees + minutes / 60.0 + seconds / 3600.0;
        return value.charAt(6) == 'S' ? -decimal : decimal;
    }

    private double parseLongitude(String value) {
        int degrees = Integer.parseInt(value.substring(0, 3));
        int minutes = Integer.parseInt(value.substring(3, 5));
        int seconds = Integer.parseInt(value.substring(5, 7));
        minutes += seconds / 60;
        seconds = seconds % 60;
        double decimal = degrees + minutes / 60.0 + seconds / 3600.0;
        return value.charAt(7) == 'W' ? -decimal : decimal;
    }
}
