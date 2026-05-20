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
