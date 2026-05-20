package org.tash.extensions.notam;

import org.tash.data.GeoCoordinate;
import org.tash.extensions.reservation.CarfWaypointResolver;
import org.tash.extensions.reservation.DefaultCarfWaypointResolver;
import org.tash.spatial.SpatialCircle;
import org.tash.spatial.SpatialPoint;
import org.tash.spatial.SpatialPolygon;
import org.tash.spatial.SpatialVolume;

import java.time.ZonedDateTime;
import java.util.Optional;
import java.util.UUID;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class DomesticAirspaceRestrictionParser {
    private static final Pattern RADIUS = Pattern.compile("(\\d+(?:\\.\\d+)?)\\s*NMR", Pattern.CASE_INSENSITIVE);
    private static final Pattern ALTITUDE_BELOW = Pattern.compile("(\\d{3,6})(?:MSL)?\\s*/\\s*B(?:LW|LO)", Pattern.CASE_INSENSITIVE);
    private static final Pattern ALTITUDE_RANGE = Pattern.compile("(\\d{3,6})\\s*[-/]\\s*(\\d{3,6})", Pattern.CASE_INSENSITIVE);
    private static final Pattern RADIAL_DME = Pattern.compile("\\b([A-Z0-9]{2,5})(\\d{3})(\\d{3}(?:\\.\\d+)?)\\b");
    private static final Pattern NAMED_FIX = Pattern.compile("\\b([A-Z0-9]{3,5})\\b");

    private final DomesticNotamParser domesticNotamParser;
    private final CarfWaypointResolver waypointResolver;

    public DomesticAirspaceRestrictionParser() {
        this(new DomesticNotamParser(), new DefaultCarfWaypointResolver());
    }

    public DomesticAirspaceRestrictionParser(DomesticNotamParser domesticNotamParser,
                                             CarfWaypointResolver waypointResolver) {
        this.domesticNotamParser = domesticNotamParser;
        this.waypointResolver = waypointResolver;
    }

    public NotamAirspaceRestriction parse(String rawText) {
        DomesticNotamRecord record = domesticNotamParser.parse(rawText);
        if (!"AIRSPACE".equals(record.getKeyword())) {
            throw new IllegalArgumentException("Domestic NOTAM is not an AIRSPACE restriction");
        }
        String text = record.getText() == null ? "" : record.getText().toUpperCase();
        double radius = radius(text);
        double[] altitudes = altitudes(text);
        GeoCoordinate center = center(record, text);

        SpatialPoint centerPoint = SpatialPoint.builder()
                .id("DOMESTIC-NOTAM-CENTER-" + UUID.randomUUID())
                .coordinate(GeoCoordinate.builder()
                        .latitude(center.getLatitude())
                        .longitude(center.getLongitude())
                        .altitude(altitudes[0])
                        .build())
                .build();
        SpatialCircle circle = SpatialCircle.builder()
                .id("DOMESTIC-NOTAM-CIRCLE-" + UUID.randomUUID())
                .center(centerPoint)
                .radius(radius)
                .build();
        SpatialPolygon polygon = circle.toPolygon(36);
        SpatialVolume volume = SpatialVolume.builder()
                .id("DOMESTIC-NOTAM-VOLUME-" + UUID.randomUUID())
                .basePolygon(polygon)
                .lowerAltitude(altitudes[0])
                .upperAltitude(altitudes[1])
                .startTime(record.getEffectiveStart())
                .endTime(record.getEffectiveEnd())
                .build();

        return NotamAirspaceRestriction.builder()
                .id("DOMESTIC-NOTAM-" + UUID.randomUUID())
                .notamType(record.getType().name())
                .accountability(record.getAccountability())
                .affectedLocation(record.getLocation())
                .qCode("DOMESTIC-AIRSPACE")
                .effectiveStart(record.getEffectiveStart())
                .effectiveEnd(record.getEffectiveEnd())
                .lowerAltitudeFeet(altitudes[0])
                .upperAltitudeFeet(altitudes[1])
                .centerLatitude(center.getLatitude())
                .centerLongitude(center.getLongitude())
                .radiusNauticalMiles(radius)
                .description(record.getText())
                .volume(volume)
                .build();
    }

    private double radius(String text) {
        Matcher matcher = RADIUS.matcher(text);
        if (!matcher.find()) {
            throw new IllegalArgumentException("Domestic AIRSPACE NOTAM is missing NMR radius");
        }
        return Double.parseDouble(matcher.group(1));
    }

    private double[] altitudes(String text) {
        Matcher below = ALTITUDE_BELOW.matcher(text);
        if (below.find()) {
            return new double[]{0, Double.parseDouble(below.group(1))};
        }
        Matcher range = ALTITUDE_RANGE.matcher(text);
        if (range.find()) {
            double first = Double.parseDouble(range.group(1));
            double second = Double.parseDouble(range.group(2));
            return new double[]{Math.min(first, second), Math.max(first, second)};
        }
        return new double[]{0, 100000};
    }

    private GeoCoordinate center(DomesticNotamRecord record, String text) {
        Matcher radial = RADIAL_DME.matcher(text);
        if (radial.find()) {
            Optional<GeoCoordinate> base = waypointResolver.resolve(radial.group(1));
            if (base.isPresent()) {
                return destination(base.get(), Double.parseDouble(radial.group(2)),
                        Double.parseDouble(radial.group(3)));
            }
        }
        Matcher named = NAMED_FIX.matcher(text);
        while (named.find()) {
            Optional<GeoCoordinate> point = waypointResolver.resolve(named.group(1));
            if (point.isPresent()) {
                return point.get();
            }
        }
        return waypointResolver.resolve(record.getLocation())
                .orElseGet(() -> fallback(record.getLocation()));
    }

    private GeoCoordinate destination(GeoCoordinate start, double bearingDegrees, double distanceNauticalMiles) {
        double angularDistance = (distanceNauticalMiles * 1852.0) / 6371000.0;
        double bearing = Math.toRadians(bearingDegrees);
        double lat1 = Math.toRadians(start.getLatitude());
        double lon1 = Math.toRadians(start.getLongitude());
        double lat2 = Math.asin(Math.sin(lat1) * Math.cos(angularDistance)
                + Math.cos(lat1) * Math.sin(angularDistance) * Math.cos(bearing));
        double lon2 = lon1 + Math.atan2(Math.sin(bearing) * Math.sin(angularDistance) * Math.cos(lat1),
                Math.cos(angularDistance) - Math.sin(lat1) * Math.sin(lat2));
        return GeoCoordinate.builder()
                .latitude(Math.toDegrees(lat2))
                .longitude(Math.toDegrees(lon2))
                .altitude(start.getAltitude())
                .build();
    }

    private GeoCoordinate fallback(String name) {
        int hash = Math.abs((name == null ? "DOMESTIC" : name).hashCode());
        return GeoCoordinate.builder()
                .latitude(20.0 + (hash % 3000) / 100.0)
                .longitude(-130.0 + ((hash / 3000) % 7000) / 100.0)
                .altitude(0)
                .build();
    }
}
