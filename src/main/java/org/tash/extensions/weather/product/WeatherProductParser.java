package org.tash.extensions.weather.product;

import org.tash.data.GeoCoordinate;
import org.tash.extensions.weather.HazardSeverity;
import org.tash.extensions.weather.WeatherElementType;
import org.tash.extensions.weather.avoid.CircularWeatherCell;
import org.tash.extensions.weather.avoid.PolygonalWeatherCell;
import org.tash.extensions.weather.pirep.PirepIntensity;
import org.tash.extensions.weather.pirep.PirepPhenomenon;
import org.tash.extensions.weather.pirep.PirepReport;
import org.tash.extensions.weather.decision.WeatherDecisionSeverity;

import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.EnumMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class WeatherProductParser {
    private static final Pattern ALTITUDE = Pattern.compile("\\b(?:FL)?(\\d{3})(?:-(?:FL)?(\\d{3}))?\\b");
    private static final Pattern COORD = Pattern.compile("([0-8]\\d)(\\d{2})?([NS])\\s*([0-1]\\d{2})(\\d{2})?([EW])");
    private static final Pattern COORD_PREFIXED = Pattern.compile("([NS])(\\d{2})(\\d{2})?\\s*([EW])(\\d{3})(\\d{2})?");
    private static final Pattern COORD_COMPACT = Pattern.compile("\\b([0-8]\\d)([NS])\\s*([0-1]\\d{2})([EW])\\b");
    private static final Pattern TIME = Pattern.compile("\\b(\\d{6}Z|\\d{4}Z)\\b");
    private static final Pattern TAF_CHANGE_GROUP = Pattern.compile("\\b(FM\\d{6}|TEMPO\\s+\\d{4}/\\d{4}|BECMG\\s+\\d{4}/\\d{4}|PROB\\d{2}(?:\\s+TEMPO)?\\s+\\d{4}/\\d{4})\\b");
    private final Map<WeatherProductType, WeatherProductDecoder> decoders = new EnumMap<>(WeatherProductType.class);

    public WeatherProductParser() {
        decoders.put(WeatherProductType.METAR, new MetarDecoder(this));
        decoders.put(WeatherProductType.TAF, new TafDecoder(this));
        decoders.put(WeatherProductType.SIGMET, new SigmetDecoder(this));
        decoders.put(WeatherProductType.AIRMET, new AirmetDecoder(this));
        decoders.put(WeatherProductType.NEXRAD_POLYGON, new CwapDecoder(this));
        decoders.put(WeatherProductType.PIREP_DERIVED, new PirepDecoder(this));
    }

    public WeatherProductParseResult parse(String raw, WeatherProductType hint) {
        String text = raw == null ? "" : raw.trim();
        String upper = text.toUpperCase(Locale.US);
        WeatherProductType type = hint == null ? classify(upper) : hint;
        WeatherProductDecoder decoder = decoders.get(type);
        if (decoder != null) {
            return decoder.decode(raw).getParseResult();
        }
        return parseResolved(raw, type);
    }

    WeatherProductParseResult parseResolved(String raw, WeatherProductType resolvedType) {
        List<String> warnings = new ArrayList<>();
        List<String> errors = new ArrayList<>();
        List<WeatherParseDiagnostic> diagnostics = new ArrayList<>();
        List<WeatherSourceSpan> sourceSpans = new ArrayList<>();
        String text = raw == null ? "" : raw.trim();
        String upper = text.toUpperCase(Locale.US);
        WeatherProductType type = resolvedType == null ? classify(upper) : resolvedType;
        if (text.isEmpty()) {
            errors.add("Weather text is empty");
            diagnostics.add(diagnostic(WeatherDiagnosticType.MISSING_GEOMETRY, WeatherDecisionSeverity.WARNING,
                    "Weather text is empty", null));
            return result(false, false, null, null, type, text, warnings, errors, diagnostics, sourceSpans);
        }
        if (type == null) {
            warnings.add("Weather text retained without a recognized weather product family");
            diagnostics.add(diagnostic(WeatherDiagnosticType.MISSING_PROVENANCE, WeatherDecisionSeverity.ADVISORY,
                    "Weather text retained without a recognized weather product family", span("raw", upper, 0, upper.length())));
            return result(false, true, null, null, WeatherProductType.GENERIC_FORECAST_HAZARD, text, warnings, errors,
                    diagnostics, sourceSpans);
        }
        if (type == WeatherProductType.PIREP_DERIVED) {
            PirepReport report = parsePirep(text, upper, diagnostics, sourceSpans);
            boolean accepted = report.getLocation() != null && report.getAltitudeFeet() != null
                    && report.getPhenomenon() != PirepPhenomenon.OTHER;
            if (!accepted) {
                errors.add("PIREP missing required weather/location/altitude fields");
            }
            return result(accepted, false, null, report, type, text, warnings, errors, diagnostics, sourceSpans);
        }
        WeatherProduct product = parseProduct(text, upper, type, warnings, diagnostics, sourceSpans);
        boolean requiresGeometry = type == WeatherProductType.SIGMET
                || type == WeatherProductType.AIRMET
                || type == WeatherProductType.NEXRAD_POLYGON;
        boolean structured = isStructured(type, product);
        boolean classifiedOnly = type == WeatherProductType.GENERIC_FORECAST_HAZARD && !structured;
        boolean accepted = product != null && !classifiedOnly && (!requiresGeometry || !product.getGeometry().isEmpty());
        if (!accepted && requiresGeometry) {
            errors.add(type + " missing coordinate geometry");
        }
        return result(accepted, classifiedOnly || (!accepted && !requiresGeometry), product, null, type, text, warnings, errors,
                diagnostics, sourceSpans);
    }

    private WeatherProduct parseProduct(String raw, String upper, WeatherProductType type, List<String> warnings,
                                        List<WeatherParseDiagnostic> diagnostics,
                                        List<WeatherSourceSpan> sourceSpans) {
        List<GeoCoordinate> points = coordinates(upper, sourceSpans);
        double[] altitude = altitudeBand(upper);
        ZonedDateTime now = ZonedDateTime.parse("2026-05-20T00:00:00Z");
        WeatherValidityWindow validity = validityWindow(type, upper, now, sourceSpans, diagnostics);
        HazardSeverity severity = severity(upper);
        WeatherElementType elementType = elementType(type, upper);
        Double lineWidth = lineWidthNauticalMiles(upper, sourceSpans);
        org.tash.extensions.weather.HazardousWeather hazard = null;
        if (points.size() >= 3) {
            hazard = PolygonalWeatherCell.builder()
                    .id(id(type, raw))
                    .type(elementType)
                    .severity(severity)
                    .startTime(validity.getValidStart())
                    .endTime(validity.getValidEnd())
                    .minAltitude(altitude[0])
                    .maxAltitude(altitude[1])
                    .vertices(points)
                    .build();
        } else if (points.size() == 2 && lineWidth != null) {
            hazard = PolygonalWeatherCell.builder()
                    .id(id(type, raw))
                    .type(elementType)
                    .severity(severity)
                    .startTime(validity.getValidStart())
                    .endTime(validity.getValidEnd())
                    .minAltitude(altitude[0])
                    .maxAltitude(altitude[1])
                    .vertices(lineCorridor(points.get(0), points.get(1), lineWidth))
                    .build();
        } else if (!points.isEmpty()) {
            hazard = CircularWeatherCell.builder()
                    .id(id(type, raw))
                    .type(elementType)
                    .severity(severity)
                    .startTime(validity.getValidStart())
                    .endTime(validity.getValidEnd())
                    .minAltitude(altitude[0])
                    .maxAltitude(altitude[1])
                    .center(points.get(0))
                    .radius(radius(upper))
                    .build();
        } else {
            warnings.add("Weather product has no coordinates; retained as non-geometric product");
            diagnostics.add(diagnostic(WeatherDiagnosticType.MISSING_GEOMETRY, WeatherDecisionSeverity.ADVISORY,
                    "Weather product has no coordinates", null));
        }
        if (altitude[0] == 0.0 && altitude[1] == 60000.0) {
            diagnostics.add(diagnostic(WeatherDiagnosticType.MISSING_ALTITUDE_BAND, WeatherDecisionSeverity.INFO,
                    "Weather product has no explicit altitude band", null));
        }
        return WeatherProduct.builder()
                .id(id(type, raw))
                .type(type)
                .source(source(type))
                .sourceProduct(type.name())
                .provider("USNS")
                .rawText(raw)
                .validity(validity)
                .issuedAt(issuedAt(upper, now, sourceSpans))
                .receivedAt(now)
                .forecastHour(forecastHour(upper))
                .confidence(WeatherConfidence.builder().value(confidence(upper)).basis("parsed text").build())
                .provenance("USNS weather parser")
                .hazard(hazard)
                .geometry(points)
                .lowerAltitudeFeet(altitude[0])
                .upperAltitudeFeet(altitude[1])
                .movement(movement(upper))
                .echoTopFeet(echoTopFeet(upper))
                .growthTrend(growthTrend(upper))
                .stormPhase(stormPhase(upper))
                .ceilingFeet(ceilingFeet(upper))
                .visibilityStatuteMiles(visibilityMiles(upper))
                .stationId(stationId(type, upper, sourceSpans))
                .windDirectionDegrees(windDirectionDegrees(upper, sourceSpans))
                .windSpeedKnots(windSpeedKnots(upper))
                .windGustKnots(windGustKnots(upper))
                .altimeterInchesHg(altimeterInchesHg(upper, sourceSpans))
                .temperatureCelsius(temperatureCelsius(upper, sourceSpans))
                .dewpointCelsius(dewpointCelsius(upper))
                .runwayVisualRangeFeet(runwayVisualRangeFeet(upper, sourceSpans))
                .lineWidthNauticalMiles(lineWidth)
                .amended(isAmended(upper))
                .corrected(isCorrected(upper))
                .variableWindFromDegrees(variableWindFromDegrees(upper, sourceSpans))
                .variableWindToDegrees(variableWindToDegrees(upper))
                .remarks(remarks(upper, sourceSpans))
                .cloudLayers(cloudLayers(upper, sourceSpans))
                .forecastChangeGroups(forecastChangeGroups(type, upper, sourceSpans))
                .weatherPhenomena(weatherPhenomena(upper, sourceSpans))
                .forecastSlices(forecastSlices(type, upper, validity, now, sourceSpans))
                .build();
    }

    private PirepReport parsePirep(String raw, String upper, List<WeatherParseDiagnostic> diagnostics,
                                   List<WeatherSourceSpan> sourceSpans) {
        GeoCoordinate location = coordinates(upper, sourceSpans).stream().findFirst().orElse(null);
        String locationText = pirepLocationText(upper, sourceSpans);
        double[] altitude = altitudeBand(upper);
        PirepIntensity intensity = upper.contains("SEV") ? PirepIntensity.SEVERE
                : upper.contains("MOD") ? PirepIntensity.MODERATE
                : upper.contains("LGT") ? PirepIntensity.LIGHT
                : PirepIntensity.UNKNOWN;
        PirepPhenomenon phenomenon = upper.contains("/IC") || upper.contains("ICE") || upper.contains("ICING") ? PirepPhenomenon.ICING
                : upper.contains("/TB") || upper.contains("TURB") ? PirepPhenomenon.TURBULENCE
                : upper.contains("/WX") || upper.contains("TS") || upper.contains("CB") ? PirepPhenomenon.CONVECTION
                : upper.contains("LLWS") || upper.contains("WIND SHEAR") ? PirepPhenomenon.WIND_SHEAR
                : upper.contains("VIS") ? PirepPhenomenon.VISIBILITY
                : upper.contains("RA") || upper.contains("SN") ? PirepPhenomenon.PRECIPITATION
                : PirepPhenomenon.OTHER;
        if (location == null) {
            diagnostics.add(diagnostic(WeatherDiagnosticType.MISSING_GEOMETRY, WeatherDecisionSeverity.WARNING,
                    "PIREP missing /OV location", fieldSpan(upper, "/OV")));
        }
        if (altitude[0] == 0.0) {
            diagnostics.add(diagnostic(WeatherDiagnosticType.MISSING_ALTITUDE_BAND, WeatherDecisionSeverity.WARNING,
                    "PIREP missing /FL altitude", fieldSpan(upper, "/FL")));
        }
        if (phenomenon == PirepPhenomenon.OTHER) {
            diagnostics.add(diagnostic(WeatherDiagnosticType.MISSING_PROVENANCE, WeatherDecisionSeverity.WARNING,
                    "PIREP missing recognized weather phenomenon", null));
        }
        addPirepFieldSpans(upper, sourceSpans);
        return PirepReport.builder()
                .id(id(WeatherProductType.PIREP_DERIVED, raw))
                .aircraftType(extractAircraftType(upper))
                .observationTime(pirepObservationTime(upper))
                .receivedTime(ZonedDateTime.parse("2026-05-20T00:00:00Z"))
                .location(location)
                .locationText(locationText)
                .altitudeFeet(altitude[0] == 0 ? null : altitude[0])
                .phenomenon(phenomenon)
                .intensity(intensity)
                .urgent(intensity == PirepIntensity.SEVERE || intensity == PirepIntensity.EXTREME)
                .rawText(raw)
                .normalizedText(upper)
                .remarks(extractRemarks(upper))
                .source("USNS")
                .locationQuality(location == null ? 0.0 : 1.0)
                .codingQuality(codingQuality(upper, phenomenon))
                .build();
    }

    private WeatherProductType classify(String upper) {
        if (upper.startsWith("PIREP") || upper.startsWith("UA ") || upper.startsWith("UUA ") || upper.contains(" PIREP ")) {
            return WeatherProductType.PIREP_DERIVED;
        }
        if (upper.contains("CONVECTIVE SIGMET") || upper.contains("SIGMET")) return WeatherProductType.SIGMET;
        if (upper.contains("AIRMET")) return WeatherProductType.AIRMET;
        if (upper.startsWith("METAR ") || upper.contains(" METAR ") || upper.startsWith("SPECI ") || upper.contains(" SPECI ")) return WeatherProductType.METAR;
        if (upper.startsWith("TAF ") || upper.contains(" TAF ")) return WeatherProductType.TAF;
        if (upper.contains("NEXRAD") || upper.contains("CWAP") || upper.contains("CWAF")) return WeatherProductType.NEXRAD_POLYGON;
        if (upper.contains(" CWA ") || upper.startsWith("CWA ") || upper.contains("CENTER WEATHER ADVISORY")
                || upper.contains("CONV") || upper.contains("TS") || upper.contains("WX ADVISORY")
                || upper.contains("WEATHER ADVISORY")) return WeatherProductType.GENERIC_FORECAST_HAZARD;
        return null;
    }

    private WeatherElementType elementType(WeatherProductType type, String upper) {
        if (type.getWeatherElementType() != null) return type.getWeatherElementType();
        if (upper.contains("ICE") || upper.contains("FZRA")) return WeatherElementType.ICING;
        if (upper.contains("TURB") || upper.contains("LLWS") || upper.contains(" WS ")) return WeatherElementType.TURBULENCE;
        if (upper.contains("CIG") || upper.contains("CEILING")) return WeatherElementType.CEILING;
        if (upper.contains("VIS") || upper.contains(" FG") || upper.contains(" BR") || upper.contains("HZ")) return WeatherElementType.VISIBILITY;
        if (upper.contains("RA") || upper.contains("SN") || upper.contains("DZ")) return WeatherElementType.PRECIPITATION;
        return WeatherElementType.CONVECTION;
    }

    private WeatherProductSource source(WeatherProductType type) {
        if (type == WeatherProductType.METAR || type == WeatherProductType.TAF || type == WeatherProductType.NEXRAD_POLYGON) {
            return WeatherProductSource.NOAA;
        }
        if (type == WeatherProductType.PIREP_DERIVED) {
            return WeatherProductSource.PILOT_REPORT;
        }
        return WeatherProductSource.FAA;
    }

    private List<GeoCoordinate> coordinates(String upper, List<WeatherSourceSpan> sourceSpans) {
        List<GeoCoordinate> coordinates = new ArrayList<>();
        Matcher matcher = COORD.matcher(upper);
        while (matcher.find()) {
            double lat = Integer.parseInt(matcher.group(1)) + minutes(matcher.group(2));
            double lon = Integer.parseInt(matcher.group(4)) + minutes(matcher.group(5));
            if ("S".equals(matcher.group(3))) lat = -lat;
            if ("W".equals(matcher.group(6))) lon = -lon;
            addCoordinate(coordinates, GeoCoordinate.builder().latitude(lat).longitude(lon).altitude(altitudeBand(upper)[0]).build());
            sourceSpans.add(span("coordinate", upper, matcher.start(), matcher.end()));
        }
        Matcher prefixed = COORD_PREFIXED.matcher(upper);
        while (prefixed.find()) {
            double lat = Integer.parseInt(prefixed.group(2)) + minutes(prefixed.group(3));
            double lon = Integer.parseInt(prefixed.group(5)) + minutes(prefixed.group(6));
            if ("S".equals(prefixed.group(1))) lat = -lat;
            if ("W".equals(prefixed.group(4))) lon = -lon;
            addCoordinate(coordinates, GeoCoordinate.builder().latitude(lat).longitude(lon).altitude(altitudeBand(upper)[0]).build());
            sourceSpans.add(span("coordinate", upper, prefixed.start(), prefixed.end()));
        }
        Matcher compact = COORD_COMPACT.matcher(upper);
        while (compact.find()) {
            double lat = Integer.parseInt(compact.group(1));
            double lon = Integer.parseInt(compact.group(3));
            if ("S".equals(compact.group(2))) lat = -lat;
            if ("W".equals(compact.group(4))) lon = -lon;
            addCoordinate(coordinates, GeoCoordinate.builder().latitude(lat).longitude(lon).altitude(altitudeBand(upper)[0]).build());
            sourceSpans.add(span("coordinate", upper, compact.start(), compact.end()));
        }
        return coordinates;
    }

    private void addCoordinate(List<GeoCoordinate> coordinates, GeoCoordinate coordinate) {
        for (GeoCoordinate existing : coordinates) {
            if (Math.abs(existing.getLatitude() - coordinate.getLatitude()) < 0.0001
                    && Math.abs(existing.getLongitude() - coordinate.getLongitude()) < 0.0001) {
                return;
            }
        }
        coordinates.add(coordinate);
    }

    private double minutes(String value) {
        return value == null ? 0.0 : Integer.parseInt(value) / 60.0;
    }

    private double[] altitudeBand(String upper) {
        Matcher sfcToFlightLevel = Pattern.compile("\\bSFC\\s*[-/]\\s*(?:FL)?(\\d{3})\\b").matcher(upper);
        if (sfcToFlightLevel.find()) {
            return new double[]{0.0, Integer.parseInt(sfcToFlightLevel.group(1)) * 100.0};
        }
        Matcher flightLevelToUnlimited = Pattern.compile("\\b(?:FL)?(\\d{3})\\s*[-/]\\s*(?:UNL|UNLIMITED|ABV)\\b").matcher(upper);
        if (flightLevelToUnlimited.find()) {
            return new double[]{Integer.parseInt(flightLevelToUnlimited.group(1)) * 100.0, 60000.0};
        }
        Matcher matcher = ALTITUDE.matcher(upper);
        if (matcher.find()) {
            double lower = Integer.parseInt(matcher.group(1)) * 100.0;
            double upperAlt = matcher.group(2) == null ? lower + 2000.0 : Integer.parseInt(matcher.group(2)) * 100.0;
            return new double[]{lower, upperAlt};
        }
        return new double[]{0.0, 60000.0};
    }

    private WeatherValidityWindow validityWindow(WeatherProductType type, String upper, ZonedDateTime now,
                                                 List<WeatherSourceSpan> sourceSpans,
                                                 List<WeatherParseDiagnostic> diagnostics) {
        Matcher range = Pattern.compile("\\b(?:VALID|VALID\\s+UNTIL|FROM)\\s*(\\d{6})/(\\d{6})\\b").matcher(upper);
        if (range.find()) {
            sourceSpans.add(span("validity", upper, range.start(), range.end()));
            return WeatherValidityWindow.builder()
                    .validStart(dayHourMinute(now, range.group(1)))
                    .validEnd(dayHourMinute(now, range.group(2)))
                    .build();
        }
        Matcher until = Pattern.compile("\\bVALID\\s+UNTIL\\s*(\\d{6})Z?\\b").matcher(upper);
        if (until.find()) {
            sourceSpans.add(span("validity", upper, until.start(), until.end()));
            return WeatherValidityWindow.builder()
                    .validStart(now)
                    .validEnd(dayHourMinute(now, until.group(1)))
                    .build();
        }
        Matcher tafRange = Pattern.compile("\\b(\\d{4})/(\\d{4})\\b").matcher(upper);
        if (type == WeatherProductType.TAF && tafRange.find()) {
            sourceSpans.add(span("validity", upper, tafRange.start(), tafRange.end()));
            return WeatherValidityWindow.builder()
                    .validStart(dayHour(now, tafRange.group(1)))
                    .validEnd(dayHour(now, tafRange.group(2)))
                    .build();
        }
        if (type == WeatherProductType.SIGMET || type == WeatherProductType.AIRMET || type == WeatherProductType.TAF) {
            diagnostics.add(diagnostic(WeatherDiagnosticType.MISSING_VALIDITY, WeatherDecisionSeverity.ADVISORY,
                    type + " has no explicit validity range", null));
        }
        return WeatherValidityWindow.builder()
                .validStart(now)
                .validEnd(now.plusHours(type == WeatherProductType.TAF ? 6 : 2))
                .build();
    }

    private ZonedDateTime issuedAt(String upper, ZonedDateTime now, List<WeatherSourceSpan> sourceSpans) {
        Matcher matcher = TIME.matcher(upper);
        if (!matcher.find()) {
            return now.minusMinutes(5);
        }
        String value = matcher.group(1).replace("Z", "");
        sourceSpans.add(span("issuedAt", upper, matcher.start(), matcher.end()));
        if (value.length() == 6) {
            return dayHourMinute(now, value);
        }
        return now.withHour(Integer.parseInt(value.substring(0, 2)))
                .withMinute(Integer.parseInt(value.substring(2, 4)))
                .withSecond(0)
                .withNano(0);
    }

    private ZonedDateTime dayHourMinute(ZonedDateTime base, String value) {
        int day = Integer.parseInt(value.substring(0, 2));
        int hour = Integer.parseInt(value.substring(2, 4));
        int minute = Integer.parseInt(value.substring(4, 6));
        ZonedDateTime candidate = base.withDayOfMonth(Math.min(day, base.toLocalDate().lengthOfMonth()))
                .withHour(hour).withMinute(minute).withSecond(0).withNano(0);
        if (candidate.isBefore(base.minusDays(15))) {
            candidate = candidate.plusMonths(1);
        }
        return candidate;
    }

    private ZonedDateTime dayHour(ZonedDateTime base, String value) {
        int day = Integer.parseInt(value.substring(0, 2));
        int hour = Integer.parseInt(value.substring(2, 4));
        ZonedDateTime candidate = base.withDayOfMonth(Math.min(day, base.toLocalDate().lengthOfMonth()))
                .withHour(hour).withMinute(0).withSecond(0).withNano(0);
        if (candidate.isBefore(base.minusDays(15))) {
            candidate = candidate.plusMonths(1);
        }
        return candidate;
    }

    private HazardSeverity severity(String upper) {
        if (upper.contains("EXTREME") || upper.contains("ROUTE BLOCKED")) return HazardSeverity.EXTREME;
        if (upper.contains("SEV") || upper.contains("+TS") || upper.contains("SIGMET")) return HazardSeverity.SEVERE;
        if (upper.contains("MOD") || upper.contains("AIRMET")) return HazardSeverity.MODERATE;
        return HazardSeverity.LIGHT;
    }

    private double radius(String upper) {
        Matcher matcher = Pattern.compile("\\b(?:RADIUS|RAD)\\s*(\\d+)").matcher(upper);
        return matcher.find() ? Double.parseDouble(matcher.group(1)) : 25.0;
    }

    private int forecastHour(String upper) {
        Matcher matcher = Pattern.compile("\\b(?:FH|FORECAST HOUR)\\s*(\\d+)").matcher(upper);
        return matcher.find() ? Integer.parseInt(matcher.group(1)) : 0;
    }

    private double confidence(String upper) {
        Matcher matcher = Pattern.compile("\\bCONF(?:IDENCE)?\\s*(\\d+(?:\\.\\d+)?)").matcher(upper);
        if (!matcher.find()) return 0.8;
        double value = Double.parseDouble(matcher.group(1));
        return value > 1.0 ? value / 100.0 : value;
    }

    private WeatherMovementVector movement(String upper) {
        Matcher matcher = Pattern.compile("\\bMOV(?:ING)?\\s*(\\d{1,3})\\s*(?:AT|/)?\\s*(\\d{1,3})").matcher(upper);
        if (!matcher.find()) {
            Matcher compass = Pattern.compile("\\bMOV(?:ING)?\\s*(N|NE|E|SE|S|SW|W|NW)\\s*(\\d{1,3})(?:KT|KTS)?\\b").matcher(upper);
            if (!compass.find()) return null;
            return WeatherMovementVector.builder()
                    .bearingDegrees(compassBearing(compass.group(1)))
                    .speedNauticalMilesPerHour(Double.parseDouble(compass.group(2)))
                    .build();
        }
        return WeatherMovementVector.builder()
                .bearingDegrees(Double.parseDouble(matcher.group(1)))
                .speedNauticalMilesPerHour(Double.parseDouble(matcher.group(2)))
                .build();
    }

    private double compassBearing(String direction) {
        if ("N".equals(direction)) return 0.0;
        if ("NE".equals(direction)) return 45.0;
        if ("E".equals(direction)) return 90.0;
        if ("SE".equals(direction)) return 135.0;
        if ("S".equals(direction)) return 180.0;
        if ("SW".equals(direction)) return 225.0;
        if ("W".equals(direction)) return 270.0;
        if ("NW".equals(direction)) return 315.0;
        return 0.0;
    }

    private Double echoTopFeet(String upper) {
        Matcher matcher = Pattern.compile("\\b(?:ECHO TOPS?|TOPS?|TOP)\\s*(?:TO\\s*)?(?:FL)?(\\d{3})").matcher(upper);
        return matcher.find() ? Double.parseDouble(matcher.group(1)) * 100.0 : null;
    }

    private Double growthTrend(String upper) {
        if (upper.contains("GROWTH") || upper.contains("GROWING")) return 1.0;
        if (upper.contains("DECAY") || upper.contains("DECAYING")) return -1.0;
        return 0.0;
    }

    private String stormPhase(String upper) {
        if (upper.contains("INIT") || upper.contains("DEVELOP")) return "DEVELOPING";
        if (upper.contains("MATURE")) return "MATURE";
        if (upper.contains("DECAY") || upper.contains("WEAKEN")) return "DECAYING";
        if (upper.contains("GROWTH") || upper.contains("GROWING")) return "GROWING";
        return null;
    }

    private Double ceilingFeet(String upper) {
        Matcher explicit = Pattern.compile("\\b(?:CIG|CEILING)\\s*(\\d{3,5})\\b").matcher(upper);
        if (explicit.find()) {
            double value = Double.parseDouble(explicit.group(1));
            return value < 1000.0 ? value * 100.0 : value;
        }
        Matcher cloud = Pattern.compile("\\b(?:BKN|OVC|VV)(\\d{3})\\b").matcher(upper);
        return cloud.find() ? Double.parseDouble(cloud.group(1)) * 100.0 : null;
    }

    private Double visibilityMiles(String upper) {
        if (upper.contains("P6SM")) {
            return 6.0;
        }
        Matcher lessThan = Pattern.compile("\\bM(\\d)/(\\d)SM\\b").matcher(upper);
        if (lessThan.find()) {
            return Double.parseDouble(lessThan.group(1)) / Double.parseDouble(lessThan.group(2));
        }
        Matcher fraction = Pattern.compile("\\b(\\d)/(\\d)SM\\b").matcher(upper);
        if (fraction.find()) {
            return Double.parseDouble(fraction.group(1)) / Double.parseDouble(fraction.group(2));
        }
        Matcher sm = Pattern.compile("\\b(\\d+)(?:\\s+)?SM\\b").matcher(upper);
        if (sm.find()) {
            return Double.parseDouble(sm.group(1));
        }
        Matcher vis = Pattern.compile("\\bVIS\\s+(\\d+)(?:/(\\d+))?\\b").matcher(upper);
        if (vis.find()) {
            return vis.group(2) == null
                    ? Double.parseDouble(vis.group(1))
                    : Double.parseDouble(vis.group(1)) / Double.parseDouble(vis.group(2));
        }
        return null;
    }

    private Integer windDirectionDegrees(String upper, List<WeatherSourceSpan> sourceSpans) {
        Matcher matcher = windMatcher(upper);
        if (!matcher.find()) {
            return null;
        }
        sourceSpans.add(span("wind", upper, matcher.start(), matcher.end()));
        return "VRB".equals(matcher.group(1)) ? null : Integer.parseInt(matcher.group(1));
    }

    private Double windSpeedKnots(String upper) {
        Matcher matcher = windMatcher(upper);
        return matcher.find() ? Double.parseDouble(matcher.group(2)) : null;
    }

    private Double windGustKnots(String upper) {
        Matcher matcher = windMatcher(upper);
        return matcher.find() && matcher.group(3) != null ? Double.parseDouble(matcher.group(3)) : null;
    }

    private Matcher windMatcher(String upper) {
        return Pattern.compile("\\b(\\d{3}|VRB)(\\d{2,3})(?:G(\\d{2,3}))?KT\\b").matcher(upper);
    }

    private Double altimeterInchesHg(String upper, List<WeatherSourceSpan> sourceSpans) {
        Matcher inches = Pattern.compile("\\bA(\\d{4})\\b").matcher(upper);
        if (inches.find()) {
            sourceSpans.add(span("altimeter", upper, inches.start(), inches.end()));
            return Double.parseDouble(inches.group(1)) / 100.0;
        }
        Matcher qnh = Pattern.compile("\\bQ(\\d{4})\\b").matcher(upper);
        if (qnh.find()) {
            sourceSpans.add(span("altimeter", upper, qnh.start(), qnh.end()));
            return Double.parseDouble(qnh.group(1)) * 0.0295299830714;
        }
        return null;
    }

    private Double temperatureCelsius(String upper, List<WeatherSourceSpan> sourceSpans) {
        Matcher matcher = temperatureMatcher(upper);
        if (!matcher.find()) {
            return null;
        }
        sourceSpans.add(span("temperatureDewpoint", upper, matcher.start(), matcher.end()));
        return signedMetarTemperature(matcher.group(1));
    }

    private Double dewpointCelsius(String upper) {
        Matcher matcher = temperatureMatcher(upper);
        return matcher.find() ? signedMetarTemperature(matcher.group(2)) : null;
    }

    private Matcher temperatureMatcher(String upper) {
        return Pattern.compile("\\b(M?\\d{2})/(M?\\d{2})\\b").matcher(upper);
    }

    private double signedMetarTemperature(String token) {
        return token.startsWith("M")
                ? -Double.parseDouble(token.substring(1))
                : Double.parseDouble(token);
    }

    private Double runwayVisualRangeFeet(String upper, List<WeatherSourceSpan> sourceSpans) {
        Matcher matcher = Pattern.compile("\\bR\\d{2}[LCR]?/(?:M|P)?(\\d{4})FT\\b").matcher(upper);
        if (!matcher.find()) {
            return null;
        }
        sourceSpans.add(span("runwayVisualRange", upper, matcher.start(), matcher.end()));
        return Double.parseDouble(matcher.group(1));
    }

    private Integer variableWindFromDegrees(String upper, List<WeatherSourceSpan> sourceSpans) {
        Matcher matcher = Pattern.compile("\\b(\\d{3})V(\\d{3})\\b").matcher(upper);
        if (!matcher.find()) {
            return null;
        }
        sourceSpans.add(span("variableWind", upper, matcher.start(), matcher.end()));
        return Integer.parseInt(matcher.group(1));
    }

    private Integer variableWindToDegrees(String upper) {
        Matcher matcher = Pattern.compile("\\b(\\d{3})V(\\d{3})\\b").matcher(upper);
        return matcher.find() ? Integer.parseInt(matcher.group(2)) : null;
    }

    private String remarks(String upper, List<WeatherSourceSpan> sourceSpans) {
        Matcher matcher = Pattern.compile("\\bRMK\\s+(.+)$").matcher(upper);
        if (!matcher.find()) {
            return null;
        }
        sourceSpans.add(span("remarks", upper, matcher.start(1), matcher.end(1)));
        return matcher.group(1).trim();
    }

    private boolean isAmended(String upper) {
        return Pattern.compile("^(?:TAF\\s+)?AMD\\b|\\bTAF\\s+AMD\\b").matcher(upper).find();
    }

    private boolean isCorrected(String upper) {
        return Pattern.compile("^(?:METAR|SPECI|TAF)?\\s*COR\\b|\\b(?:METAR|SPECI|TAF)\\s+COR\\b|\\b[A-Z0-9]{4}\\s+COR\\b").matcher(upper).find();
    }

    private List<String> cloudLayers(String upper, List<WeatherSourceSpan> sourceSpans) {
        List<String> layers = new ArrayList<>();
        Matcher matcher = Pattern.compile("\\b(FEW|SCT|BKN|OVC|VV)(\\d{3})(CB|TCU)?\\b").matcher(upper);
        while (matcher.find()) {
            layers.add(matcher.group());
            sourceSpans.add(span("cloudLayer", upper, matcher.start(), matcher.end()));
        }
        return layers;
    }

    private Double lineWidthNauticalMiles(String upper, List<WeatherSourceSpan> sourceSpans) {
        Matcher eitherSide = Pattern.compile("\\bWI(?:THIN)?\\s+(\\d+(?:\\.\\d+)?)\\s*NM\\s+EITHER\\s+SIDE\\b").matcher(upper);
        if (eitherSide.find()) {
            sourceSpans.add(span("lineWidth", upper, eitherSide.start(), eitherSide.end()));
            return Double.parseDouble(eitherSide.group(1)) * 2.0;
        }
        Matcher wide = Pattern.compile("\\b(\\d+(?:\\.\\d+)?)\\s*NM\\s+WIDE\\b").matcher(upper);
        if (wide.find()) {
            sourceSpans.add(span("lineWidth", upper, wide.start(), wide.end()));
            return Double.parseDouble(wide.group(1));
        }
        return null;
    }

    private List<GeoCoordinate> lineCorridor(GeoCoordinate start, GeoCoordinate end, double widthNauticalMiles) {
        if (start == null || end == null) {
            return Collections.emptyList();
        }
        double halfWidthDegrees = Math.max(0.1, widthNauticalMiles / 2.0 / 60.0);
        double dLat = end.getLatitude() - start.getLatitude();
        double dLon = end.getLongitude() - start.getLongitude();
        double length = Math.sqrt(dLat * dLat + dLon * dLon);
        if (length == 0.0) {
            return Collections.singletonList(start);
        }
        double offLat = -dLon / length * halfWidthDegrees;
        double offLon = dLat / length * halfWidthDegrees;
        List<GeoCoordinate> points = new ArrayList<>();
        points.add(GeoCoordinate.builder().latitude(start.getLatitude() + offLat).longitude(start.getLongitude() + offLon).altitude(start.getAltitude()).build());
        points.add(GeoCoordinate.builder().latitude(end.getLatitude() + offLat).longitude(end.getLongitude() + offLon).altitude(end.getAltitude()).build());
        points.add(GeoCoordinate.builder().latitude(end.getLatitude() - offLat).longitude(end.getLongitude() - offLon).altitude(end.getAltitude()).build());
        points.add(GeoCoordinate.builder().latitude(start.getLatitude() - offLat).longitude(start.getLongitude() - offLon).altitude(start.getAltitude()).build());
        return points;
    }

    private boolean isStructured(WeatherProductType type, WeatherProduct product) {
        if (product == null) return false;
        return !product.getGeometry().isEmpty()
                || product.getHazard() != null
                || product.getCeilingFeet() != null
                || product.getVisibilityStatuteMiles() != null
                || product.getWindSpeedKnots() != null
                || product.getAltimeterInchesHg() != null
                || !product.getForecastSlices().isEmpty()
                || product.getMovement() != null
                || product.getEchoTopFeet() != null
                || type == WeatherProductType.METAR
                || type == WeatherProductType.TAF;
    }

    private String extractAircraftType(String upper) {
        Matcher matcher = Pattern.compile("(?:\\bACFT\\b|\\bAIRCRAFT\\b|/TP)\\s*([A-Z0-9]{2,6})").matcher(upper);
        return matcher.find() ? matcher.group(1) : "UNKNOWN";
    }

    private double codingQuality(String upper, PirepPhenomenon phenomenon) {
        double quality = phenomenon == PirepPhenomenon.OTHER ? 0.5 : 1.0;
        if (upper.contains("UNKN") || upper.contains("UNKNOWN")) {
            quality = Math.min(quality, 0.6);
        }
        if (!upper.contains("/OV") || !upper.contains("/FL")) {
            quality = Math.min(quality, 0.5);
        }
        return quality;
    }

    private String stationId(WeatherProductType type, String upper, List<WeatherSourceSpan> sourceSpans) {
        if (type != WeatherProductType.METAR && type != WeatherProductType.TAF) {
            return null;
        }
        Matcher matcher = Pattern.compile("^(?:METAR|SPECI|TAF)(?:\\s+(?:AMD|COR))?\\s+([A-Z0-9]{4})\\b").matcher(upper);
        if (!matcher.find()) {
            return null;
        }
        sourceSpans.add(span("station", upper, matcher.start(1), matcher.end(1)));
        return matcher.group(1);
    }

    private List<String> forecastChangeGroups(WeatherProductType type, String upper, List<WeatherSourceSpan> sourceSpans) {
        List<String> groups = new ArrayList<>();
        if (type != WeatherProductType.TAF) {
            return groups;
        }
        Matcher matcher = TAF_CHANGE_GROUP.matcher(upper);
        while (matcher.find()) {
            groups.add(matcher.group(1));
            sourceSpans.add(span("tafChangeGroup", upper, matcher.start(), matcher.end()));
        }
        return groups;
    }

    private List<WeatherForecastSlice> forecastSlices(WeatherProductType type, String upper, WeatherValidityWindow validity,
                                                      ZonedDateTime now, List<WeatherSourceSpan> sourceSpans) {
        if (type == WeatherProductType.TAF) {
            return tafSlices(upper, validity, now, sourceSpans);
        }
        if (type == WeatherProductType.NEXRAD_POLYGON || upper.contains("CWAP") || upper.contains("CWAF")) {
            return cwapSlices(type, upper, validity, now, sourceSpans);
        }
        return Collections.emptyList();
    }

    private List<WeatherForecastSlice> cwapSlices(WeatherProductType type, String upper, WeatherValidityWindow validity,
                                                  ZonedDateTime now, List<WeatherSourceSpan> sourceSpans) {
        List<WeatherForecastSlice> slices = new ArrayList<>();
        List<MatcherHit> hits = new ArrayList<>();
        Matcher matcher = Pattern.compile("\\b(?:FH|FORECAST HOUR)\\s*(\\d+)\\b").matcher(upper);
        while (matcher.find()) {
            hits.add(new MatcherHit(matcher.start(), matcher.end(), matcher.group(1)));
        }
        if (hits.isEmpty()) {
            hits.add(new MatcherHit(0, 0, String.valueOf(forecastHour(upper))));
        }
        for (int i = 0; i < hits.size(); i++) {
            MatcherHit hit = hits.get(i);
            int end = i + 1 < hits.size() ? hits.get(i + 1).start : upper.length();
            int startOffset = hit.start == 0 && hit.end == 0 ? 0 : hit.start;
            int forecastHour = Integer.parseInt(hit.text);
            ZonedDateTime start = validity == null || validity.getValidStart() == null
                    ? now.plusHours(forecastHour)
                    : validity.getValidStart().plusHours(forecastHour);
            String rawSlice = upper.substring(startOffset, end).trim();
            WeatherSourceSpan sourceSpan = span("forecastSlice", upper, startOffset, end);
            sourceSpans.add(sourceSpan);
            List<GeoCoordinate> geometry = coordinates(upper, new ArrayList<>());
            slices.add(WeatherForecastSlice.builder()
                    .sequence(i)
                    .groupType("FORECAST_HOUR")
                    .rawText(rawSlice.isEmpty() ? upper : rawSlice)
                    .validStart(start)
                    .validEnd(start.plusHours(1))
                    .forecastHour(forecastHour)
                    .ceilingFeet(ceilingFeet(rawSlice))
                    .visibilityStatuteMiles(visibilityMiles(rawSlice))
                    .weatherPhenomena(weatherPhenomena(rawSlice.isEmpty() ? upper : rawSlice, new ArrayList<>()))
                    .confidence(confidence(rawSlice.isEmpty() ? upper : rawSlice))
                    .sourceProductId(id(type, upper))
                    .geometry(geometry)
                    .movementAdjustedGeometry(geometry)
                    .ruleIds(Collections.singletonList("WX-BLOCKED-PROBABILITY-GTE-0.80"))
                    .sourceSpan(sourceSpan)
                    .build());
        }
        return slices;
    }

    private List<WeatherForecastSlice> tafSlices(String upper, WeatherValidityWindow validity,
                                                 ZonedDateTime now, List<WeatherSourceSpan> sourceSpans) {
        List<WeatherForecastSlice> slices = new ArrayList<>();
        List<MatcherHit> hits = new ArrayList<>();
        Matcher matcher = TAF_CHANGE_GROUP.matcher(upper);
        while (matcher.find()) {
            hits.add(new MatcherHit(matcher.start(), matcher.end(), matcher.group(1)));
        }
        int bodyStart = tafBodyStart(upper);
        if (hits.isEmpty()) {
            slices.add(slice(0, "BASE", upper.substring(bodyStart).trim(),
                    validity == null ? now : validity.getValidStart(),
                    validity == null ? now.plusHours(6) : validity.getValidEnd(),
                    now, upper, bodyStart, upper.length(), sourceSpans));
            return slices;
        }
        slices.add(slice(0, "BASE", upper.substring(bodyStart, hits.get(0).start).trim(),
                validity == null ? now : validity.getValidStart(),
                startForGroup(hits.get(0).text, now, validity),
                now, upper, bodyStart, hits.get(0).start, sourceSpans));
        for (int i = 0; i < hits.size(); i++) {
            MatcherHit hit = hits.get(i);
            int end = i + 1 < hits.size() ? hits.get(i + 1).start : upper.length();
            ZonedDateTime start = startForGroup(hit.text, now, validity);
            ZonedDateTime sliceEnd = endForGroup(hit.text, now, validity,
                    i + 1 < hits.size() ? startForGroup(hits.get(i + 1).text, now, validity) : null);
            slices.add(slice(i + 1, groupType(hit.text), upper.substring(hit.start, end).trim(),
                    start, sliceEnd, now, upper, hit.start, end, sourceSpans));
        }
        return slices;
    }

    private WeatherForecastSlice slice(int sequence, String groupType, String rawSlice,
                                       ZonedDateTime validStart, ZonedDateTime validEnd, ZonedDateTime now,
                                       String source, int startOffset, int endOffset,
                                       List<WeatherSourceSpan> sourceSpans) {
        WeatherSourceSpan sourceSpan = span("forecastSlice", source, startOffset, endOffset);
        sourceSpans.add(sourceSpan);
        return WeatherForecastSlice.builder()
                .sequence(sequence)
                .groupType(groupType)
                .rawText(rawSlice)
                .validStart(validStart)
                .validEnd(validEnd)
                .forecastHour(validStart == null ? null : (int) java.time.Duration.between(now, validStart).toHours())
                .ceilingFeet(ceilingFeet(rawSlice))
                .visibilityStatuteMiles(visibilityMiles(rawSlice))
                .weatherPhenomena(weatherPhenomena(rawSlice, new ArrayList<>()))
                .confidence(sliceConfidence(rawSlice))
                .sourceProductId(id(WeatherProductType.TAF, source))
                .ruleIds(Collections.singletonList("WX-STALE-DATA-REVIEW"))
                .sourceSpan(sourceSpan)
                .build();
    }

    private int tafBodyStart(String upper) {
        Matcher validity = Pattern.compile("\\b\\d{4}/\\d{4}\\b").matcher(upper);
        return validity.find() ? validity.end() : 0;
    }

    private ZonedDateTime startForGroup(String group, ZonedDateTime now, WeatherValidityWindow validity) {
        if (group.startsWith("FM")) {
            return dayHourMinute(now, group.substring(2, 8));
        }
        Matcher range = Pattern.compile("(\\d{4})/(\\d{4})").matcher(group);
        if (range.find()) {
            return dayHour(now, range.group(1));
        }
        return validity == null ? now : validity.getValidStart();
    }

    private ZonedDateTime endForGroup(String group, ZonedDateTime now, WeatherValidityWindow validity, ZonedDateTime nextStart) {
        Matcher range = Pattern.compile("(\\d{4})/(\\d{4})").matcher(group);
        if (range.find()) {
            return dayHour(now, range.group(2));
        }
        if (nextStart != null) {
            return nextStart;
        }
        return validity == null ? now.plusHours(6) : validity.getValidEnd();
    }

    private String groupType(String group) {
        if (group.startsWith("FM")) return "FM";
        if (group.startsWith("TEMPO")) return "TEMPO";
        if (group.startsWith("BECMG")) return "BECMG";
        if (group.startsWith("PROB")) return group.startsWith("PROB") && group.contains("TEMPO") ? "PROB_TEMPO" : "PROB";
        return "CHANGE";
    }

    private double sliceConfidence(String rawSlice) {
        Matcher prob = Pattern.compile("\\bPROB(\\d{2})\\b").matcher(rawSlice);
        if (prob.find()) {
            return Double.parseDouble(prob.group(1)) / 100.0;
        }
        return confidence(rawSlice);
    }

    private List<String> weatherPhenomena(String upper, List<WeatherSourceSpan> sourceSpans) {
        List<String> phenomena = new ArrayList<>();
        String[] tokens = {"TSRA", "+TSRA", "-TSRA", "TSGR", "FZRA", "+RA", "-RA", "RA", "SN", "FG", "BR", "HZ", "LLWS", "WS", "CB"};
        for (String token : tokens) {
            Matcher matcher = Pattern.compile("(?<![A-Z0-9])" + Pattern.quote(token) + "(?![A-Z0-9])").matcher(upper);
            while (matcher.find()) {
                if (!phenomena.contains(token)) {
                    phenomena.add(token);
                }
                sourceSpans.add(span("phenomenon", upper, matcher.start(), matcher.end()));
            }
        }
        return phenomena;
    }

    private ZonedDateTime pirepObservationTime(String upper) {
        Matcher matcher = Pattern.compile("(?:^|\\s)/TM\\s*(\\d{4})\\b").matcher(upper);
        if (!matcher.find()) {
            return ZonedDateTime.parse("2026-05-20T00:00:00Z");
        }
        String value = matcher.group(1);
        return ZonedDateTime.parse("2026-05-20T00:00:00Z")
                .withHour(Integer.parseInt(value.substring(0, 2)))
                .withMinute(Integer.parseInt(value.substring(2, 4)));
    }

    private String pirepLocationText(String upper, List<WeatherSourceSpan> sourceSpans) {
        Matcher matcher = Pattern.compile("(?:^|\\s)/OV\\s*([^/]+)").matcher(upper);
        if (!matcher.find()) {
            return null;
        }
        String value = matcher.group(1).trim();
        int endOfField = value.indexOf(' ');
        if (endOfField > 0) {
            value = value.substring(0, endOfField);
        }
        sourceSpans.add(span("pirepLocation", upper, matcher.start(1), matcher.start(1) + value.length()));
        return value;
    }

    private void addPirepFieldSpans(String upper, List<WeatherSourceSpan> sourceSpans) {
        String[] fields = {"/TP", "/OV", "/TM", "/FL", "/TB", "/IC", "/RM"};
        for (String field : fields) {
            WeatherSourceSpan span = fieldSpan(upper, field);
            if (span != null) {
                sourceSpans.add(span);
            }
        }
    }

    private String extractRemarks(String upper) {
        Matcher matcher = Pattern.compile("(?:/RM|RMK)\\s+(.+)$").matcher(upper);
        return matcher.find() ? matcher.group(1).trim() : "";
    }

    private String id(WeatherProductType type, String raw) {
        return type.name() + "-" + Math.abs((raw == null ? "" : raw).hashCode());
    }

    private WeatherSourceSpan fieldSpan(String upper, String field) {
        int start = upper.indexOf(field);
        return start < 0 ? null : span(field, upper, start, Math.min(upper.length(), start + field.length()));
    }

    private WeatherSourceSpan span(String field, String text, int start, int end) {
        return WeatherSourceSpan.builder()
                .field(field)
                .startOffset(start)
                .endOffset(end)
                .text(text.substring(Math.max(0, start), Math.min(text.length(), end)))
                .build();
    }

    private WeatherParseDiagnostic diagnostic(WeatherDiagnosticType type, WeatherDecisionSeverity severity,
                                              String message, WeatherSourceSpan span) {
        return WeatherParseDiagnostic.builder()
                .type(type)
                .severity(severity)
                .message(message)
                .sourceSpan(span)
                .build();
    }

    private WeatherProductParseResult result(boolean accepted, boolean classifiedOnly, WeatherProduct product, PirepReport pirep,
                                             WeatherProductType type, String raw, List<String> warnings,
                                             List<String> errors, List<WeatherParseDiagnostic> diagnostics,
                                             List<WeatherSourceSpan> sourceSpans) {
        return WeatherProductParseResult.builder()
                .accepted(accepted)
                .classifiedOnly(classifiedOnly)
                .product(product)
                .pirepReport(pirep)
                .classifiedType(type)
                .rawText(raw)
                .warnings(warnings)
                .errors(errors)
                .diagnostics(diagnostics)
                .sourceSpans(sourceSpans)
                .build();
    }

    private static class MatcherHit {
        private final int start;
        private final int end;
        private final String text;

        private MatcherHit(int start, int end, String text) {
            this.start = start;
            this.end = end;
            this.text = text;
        }
    }
}
