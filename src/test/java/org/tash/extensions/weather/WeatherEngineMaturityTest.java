package org.tash.extensions.weather;

import org.junit.jupiter.api.Test;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.engine.DecisionRuleCatalog;
import org.tash.extensions.engine.OperationalGeometryService;
import org.tash.extensions.engine.OperationalDecisionResult;
import org.tash.extensions.weather.product.DecodedMetar;
import org.tash.extensions.weather.product.DecodedSigmetAirmet;
import org.tash.extensions.weather.product.DecodedTaf;
import org.tash.extensions.weather.product.MetarDecoder;
import org.tash.extensions.weather.product.SigmetDecoder;
import org.tash.extensions.weather.product.TafDecoder;
import org.tash.extensions.weather.product.WeatherDecoderResult;
import org.tash.extensions.weather.product.WeatherProduct;
import org.tash.extensions.weather.product.WeatherProductParseResult;
import org.tash.extensions.weather.product.WeatherProductParser;
import org.tash.extensions.weather.product.WeatherProductStalenessPolicy;
import org.tash.extensions.weather.product.WeatherProductType;

import java.io.IOException;
import java.time.ZonedDateTime;
import java.util.Arrays;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class WeatherEngineMaturityTest {
    private static final ZonedDateTime T0 = ZonedDateTime.parse("2026-05-20T00:00:00Z");

    @Test
    void productSpecificDecodersExposeTypedDecodedModels() {
        WeatherProductParser parser = new WeatherProductParser();
        WeatherDecoderResult metar = new MetarDecoder(parser).decode("METAR KJFK COR 200300Z 18012G22KT 4SM RA BKN008 17/15 A2987 RMK COR");
        WeatherDecoderResult taf = new TafDecoder(parser).decode("TAF AMD KJFK 200000Z 2000/2106 2SM RA BKN010 FM200300 1/2SM TSRA BKN004");
        WeatherDecoderResult sigmet = new SigmetDecoder(parser).decode("SIGMET CONV VALID 200000/200600 FROM 3000N15000W TO 3000N14900W TO 3100N14900W FL240-260 TOP FL450");

        assertTrue(metar.isAccepted());
        assertTrue(metar.getDecoded() instanceof DecodedMetar);
        assertEquals("KJFK", ((DecodedMetar) metar.getDecoded()).getStationId());
        assertTrue(((DecodedMetar) metar.getDecoded()).isCorrected());
        assertTrue(((DecodedMetar) metar.getDecoded()).getPresentWeather().contains("RA"));
        assertTrue(taf.getDecoded() instanceof DecodedTaf);
        assertTrue(((DecodedTaf) taf.getDecoded()).isAmended());
        assertFalse(((DecodedTaf) taf.getDecoded()).getGroups().isEmpty());
        assertTrue(sigmet.getDecoded() instanceof DecodedSigmetAirmet);
        assertEquals(3, ((DecodedSigmetAirmet) sigmet.getDecoded()).getGeometry().size());
    }

    @Test
    void forecastSlicesExpandIntoSliceSpecificProducts() {
        WeatherProductParseResult result = new WeatherProductParser().parse(
                "TAF KJFK 200000Z 2000/2106 2SM RA BKN010 FM200300 1/2SM TSRA BKN004 TEMPO 2004/2008 1/4SM FG VV002",
                null);

        List<WeatherProduct> expanded = result.getProduct().expandForecastSlices();

        assertTrue(result.isAccepted(), result.getErrors().toString());
        assertEquals(result.getProduct().getForecastSlices().size(), expanded.size());
        assertTrue(expanded.stream().anyMatch(product -> product.getId().contains("#slice-")));
        assertTrue(expanded.stream().anyMatch(product -> product.getVisibilityStatuteMiles() != null
                && product.getVisibilityStatuteMiles() <= 0.25));
    }

    @Test
    void terminalMetarAndTafWithoutCoordinatesRemainNonGeometricGuidanceArtifacts() {
        WeatherProductParseResult metar = new WeatherProductParser().parse(
                "SPECI KJFK 281655Z 04005KT 1/8SM R04L/1000FT FG VV002 08/08 A2992 RMK AO2 RVRNO",
                null);
        WeatherProductParseResult taf = new WeatherProductParser().parse(
                "TAF AMD KJFK 280000Z 2800/2906 1SM BR OVC004 TEMPO 2804/2808 1/2SM FG VV002",
                null);

        assertTrue(metar.isAccepted(), metar.getErrors().toString());
        assertTrue(taf.isAccepted(), taf.getErrors().toString());
        assertEquals(1000.0, metar.getProduct().getRunwayVisualRangeFeet());
        assertTrue(metar.getProduct().getGeometry().isEmpty());
        assertTrue(taf.getProduct().getGeometry().isEmpty());
        assertTrue(metar.getWarnings().stream().anyMatch(warning -> warning.contains("non-geometric")));
        assertFalse(taf.getProduct().getForecastSlices().isEmpty());
    }

    @Test
    void documentedPirepAndAirepEdgeCasesRetainTypedDiagnostics() {
        WeatherProductParser parser = new WeatherProductParser();
        WeatherProductParseResult urgentOceanic = parser.parse(
                "UUA /OV 3000N15000W/TM 2015/FL240/TP B789/TB SEV/RM OCEANIC POSITION",
                null);
        WeatherProductParseResult airepSmooth = parser.parse(
                "AIREP /OV 3100N14900W/TM 2020/FL300/TP A359/TB SMOOTH/RM NEGATIVE TURBULENCE",
                null);
        WeatherProductParseResult missingAltitude = parser.parse(
                "UA /OV 3200N14800W/TM 2030/TP B738/IC MOD/RM ALTITUDE FIELD OMITTED",
                null);

        assertTrue(urgentOceanic.isAccepted(), urgentOceanic.getErrors().toString());
        assertTrue(urgentOceanic.getPirepReport().isUrgent());
        assertEquals(24000.0, urgentOceanic.getPirepReport().getAltitudeFeet());
        assertNotNull(urgentOceanic.getPirepReport().getLocation());

        assertTrue(airepSmooth.isAccepted(), airepSmooth.getErrors().toString());
        assertEquals(WeatherProductType.PIREP_DERIVED, airepSmooth.getClassifiedType());
        assertTrue(airepSmooth.getPirepReport().getRemarks().contains("NEGATIVE TURBULENCE"));

        assertFalse(missingAltitude.isAccepted());
        assertTrue(missingAltitude.getDiagnostics().stream()
                .anyMatch(diagnostic -> diagnostic.getMessage().contains("missing /FL altitude")));
    }

    @Test
    void documentedSigmetAirmetCwaMissingGeometryCasesStayRetainedWithDiagnostics() {
        WeatherProductParser parser = new WeatherProductParser();
        WeatherProductParseResult gAirmet = parser.parse(
                "G-AIRMET VALID 200000/200600 MOD ICE BLW FL180 MOV NE 20KT",
                null);
        WeatherProductParseResult cwsu = parser.parse(
                "CWSU CWA 101 VALID 200000/200100 AREA TS TOP FL420 MOV E 30KT INTSF",
                null);

        assertTrue(gAirmet.isAccepted(), gAirmet.getErrors().toString());
        assertEquals(WeatherProductType.AIRMET, gAirmet.getClassifiedType());
        assertTrue(gAirmet.getProduct().getGeometry().isEmpty());
        assertTrue(gAirmet.getDiagnostics().stream()
                .anyMatch(diagnostic -> diagnostic.getMessage().contains("no coordinates")));

        assertTrue(cwsu.isAccepted(), cwsu.getErrors().toString());
        assertTrue(cwsu.getProduct().getGeometry().isEmpty());
        assertTrue(cwsu.getProduct().getRawText().contains("CWSU"));
        assertTrue(cwsu.getDiagnostics().stream()
                .anyMatch(diagnostic -> diagnostic.getMessage().contains("no coordinates")));
    }

    @Test
    void productStalenessPolicyDistinguishesTacticalCwaFromTafAndAirmetHorizons() {
        ZonedDateTime received = ZonedDateTime.parse("2026-05-20T00:00:00Z");
        WeatherProduct cwa = WeatherProduct.builder()
                .id("cwa-1")
                .type(WeatherProductType.NEXRAD_POLYGON)
                .rawText("CWA 201 VALID 200000/200100 CONVECTIVE LINE TOP FL430")
                .issuedAt(received)
                .receivedAt(received)
                .build();
        WeatherProduct taf = WeatherProduct.builder()
                .id("taf-1")
                .type(WeatherProductType.TAF)
                .rawText("TAF KJFK 200000Z 2000/2106 3SM BR BKN010")
                .issuedAt(received)
                .receivedAt(received)
                .build();
        WeatherProduct airmet = WeatherProduct.builder()
                .id("airmet-1")
                .type(WeatherProductType.AIRMET)
                .rawText("G-AIRMET VALID 200000/200600 MOD ICE")
                .issuedAt(received)
                .receivedAt(received)
                .build();
        WeatherProductStalenessPolicy policy = new WeatherProductStalenessPolicy();

        assertTrue(policy.isStale(cwa, received.plusMinutes(50)));
        assertFalse(policy.isStale(taf, received.plusMinutes(50)));
        assertFalse(policy.isStale(airmet, received.plusHours(3)));
        assertTrue(policy.isStale(airmet, received.plusHours(5)));
    }

    @Test
    void operationalGeometryServiceHandlesCorridorsAndPreciseOverlap() {
        OperationalGeometryService geometry = new OperationalGeometryService();
        List<GeoCoordinate> route = Arrays.asList(point(30, -150), point(30, -149));
        List<GeoCoordinate> corridor = geometry.toCorridor(route.get(0), route.get(1), 20);
        List<GeoCoordinate> crossing = Arrays.asList(point(29.9, -149.5), point(30.1, -149.5));
        List<GeoCoordinate> separated = Arrays.asList(point(30.6, -149.5), point(30.8, -149.5));

        assertTrue(geometry.overlaps(corridor, crossing));
        assertFalse(geometry.overlaps(corridor, separated));
        assertEquals(Arrays.asList(0), geometry.routeSegmentsImpacted(route, crossing, 20));
        assertTrue(geometry.distanceToRoute(route, point(30, -149.5)) < 1.0);
    }

    @Test
    void expandedScenarioCorpusParsesByFamily() throws IOException {
        ScenarioCorpusRunner runner = new ScenarioCorpusRunner();

        assertEquals(20, runner.parseLineCorpus("/scenarios/weather-engine/corpus-metar-speci.txt", null).size());
        assertEquals(20, runner.parseLineCorpus("/scenarios/weather-engine/corpus-taf.txt", null).size());
        assertEquals(15, runner.parseLineCorpus("/scenarios/weather-engine/corpus-sigmet-airmet.txt", null).size());
        assertEquals(10, runner.parseLineCorpus("/scenarios/weather-engine/corpus-cwap-cwaf.txt", null).size());
        assertEquals(10, runner.parseLineCorpus("/scenarios/weather-engine/corpus-pirep.txt", null).size());
    }

    @Test
    void mixedUsnsCorpusRunsThroughEngineAndTraceRulesAreCatalogued() throws IOException {
        OperationalDecisionResult result = new ScenarioCorpusRunner()
                .runEngineCorpus("/scenarios/weather-engine/corpus-mixed-usns.txt", T0);

        assertTrue(result.getWeatherProducts().size() >= 5);
        assertFalse(result.getPirepResults().isEmpty());
        assertTrue(result.getTrace().getSteps().stream()
                .filter(step -> step.getRule() != null)
                .allMatch(step -> DecisionRuleCatalog.byId(step.getRule().getId()) != null));
    }

    private GeoCoordinate point(double lat, double lon) {
        return GeoCoordinate.builder().latitude(lat).longitude(lon).altitude(25000).build();
    }
}
