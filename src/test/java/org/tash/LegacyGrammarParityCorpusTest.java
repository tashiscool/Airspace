package org.tash;

import org.junit.jupiter.api.Test;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.carf.altrv.AltrvParseResult;
import org.tash.extensions.carf.altrv.AltrvParser;
import org.tash.extensions.carf.altrv.AltrvRouteKind;
import org.tash.extensions.carf.altrv.AltrvSpatialMapper;
import org.tash.extensions.carf.refdata.InMemoryCarfReferenceDataProvider;
import org.tash.extensions.messaging.UsnsMessageEnvelope;
import org.tash.extensions.messaging.transaction.UsnsTransactionSplitter;
import org.tash.extensions.messaging.transaction.UsnsTransactionType;
import org.tash.extensions.notam.DomesticNotamParseResult;
import org.tash.extensions.notam.DomesticNotamParser;
import org.tash.extensions.notam.NotamAirspaceParser;
import org.tash.extensions.notam.NotamFieldParseResult;
import org.tash.extensions.reservation.CarfReservationEvent;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.stream.Collectors;

import static org.junit.jupiter.api.Assertions.*;

class LegacyGrammarParityCorpusTest {
    @Test
    void altrvRouteFamilyCorpusPreservesLegacyRouteKindsAndEventMetadata() {
        AltrvParseResult result = new AltrvParser().parse(resource("altrv-route-families.txt"));
        Set<AltrvRouteKind> routeKinds = result.getMessage().getRouteGroups().stream()
                .flatMap(group -> group.getRoutes().stream())
                .map(route -> route.getKind())
                .collect(Collectors.toSet());

        assertTrue(result.isAccepted(), result.getDiagnostics().toString());
        assertTrue(routeKinds.containsAll(Arrays.asList(
                AltrvRouteKind.COMMON,
                AltrvRouteKind.BRANCH,
                AltrvRouteKind.PARTIAL,
                AltrvRouteKind.ALTERNATE_DEPARTURE,
                AltrvRouteKind.REVERSE)));
        assertTrue(result.getMessage().getEvents().stream()
                .anyMatch(event -> "ALTRV.g".equals(event.getMetadata().get("grammarSource"))
                        && event.getMetadata().containsKey("flightLevel")));
        assertTrue(result.getMessage().getEvents().stream()
                .anyMatch(event -> "LOCAL_EVENT_WINDOW".equals(event.getMetadata().get("scope"))
                        && event.getMetadata().containsKey("fix")
                        && event.getMetadata().containsKey("elapsedTime")));
    }

    @Test
    void altrvStationaryLineCorpusMapsWidthAltitudeAndGeometryIntent() {
        AltrvParseResult result = new AltrvParser().parse(resource("altrv-stationary-line-corridor.txt"));
        Map<String, GeoCoordinate> fixes = new HashMap<>();
        fixes.put("FIXA", point(30, -150));
        fixes.put("FIXB", point(31, -149));
        List<CarfReservationEvent> events = new AltrvSpatialMapper()
                .toReservationEvents(result.getMessage(), new InMemoryCarfReferenceDataProvider(fixes));

        assertTrue(result.isAccepted(), result.getDiagnostics().toString());
        assertTrue(events.stream().anyMatch(event -> event.getRouteWidthNauticalMiles() == 40.0
                && event.getLowerAltitudeFeet() == 0.0
                && event.getUpperAltitudeFeet() == 18000.0
                && "LINE_CORRIDOR".equals(event.getShapeIntent())
                && event.getSourceText().contains("geometryIntent=LINE_CORRIDOR")
                && event.getSourceText().contains("association=STATIONARY_RESERVATION")));
        assertTrue(result.getMessage().getAreas().stream().anyMatch(area ->
                "LINE_CORRIDOR".equals(area.getGeometryIntent())
                        && "STATIONARY_RESERVATION".equals(area.getEnterExitAssociation())
                        && "SFC".equals(area.getLowerFlightLevel())
                        && "180".equals(area.getUpperFlightLevel())));
    }

    @Test
    void dom1CorpusKeepsAcceptedRecordsAndTypedMalformedDiagnostics() {
        DomesticNotamParser parser = new DomesticNotamParser();
        List<DomesticNotamParseResult> rows = lines("dom1-record-shapes.txt").stream()
                .map(parser::parseDetailed)
                .collect(Collectors.toList());

        assertEquals(6, rows.size());
        assertTrue(rows.subList(0, 5).stream().allMatch(DomesticNotamParseResult::isAccepted));
        assertFalse(rows.get(5).isAccepted());
        assertNotNull(rows.get(5).getRejectionReason());
        assertTrue(rows.stream().anyMatch(result -> "DOM2.UNMATCHED".equals(result.getReducerRuleId())));
    }

    @Test
    void dom2CorpusReducesImplementedSemanticFamilies() {
        DomesticNotamParser parser = new DomesticNotamParser();
        for (String row : lines("dom2-semantic-cases.txt")) {
            String[] parts = row.split("\\|");
            DomesticNotamParseResult result = parser.parseDetailed(domestic(parts[0]));

            assertTrue(result.isAccepted(), row + " -> " + result.getRejectionReason());
            assertEquals(parts[1], result.getReducerRuleId(), row);
            assertEquals(parts[2], result.getSemanticFacilityFamily(), row);
            assertNotNull(result.getSemanticClassification(), row);
        }
    }

    @Test
    void usnsGrammarCorpusClassifiesNotamRequestTableAndGenotFamiliesDistinctly() {
        String body = String.join("\n", lines("usns-grammar-families.txt"));
        UsnsMessageEnvelope envelope = UsnsMessageEnvelope.builder()
                .originAddress("CZYZ")
                .body(body)
                .build();
        List<org.tash.extensions.messaging.transaction.UsnsTransaction> transactions =
                new UsnsTransactionSplitter().split(envelope).getTransactions();
        Set<UsnsTransactionType> types = transactions.stream()
                .map(transaction -> transaction.getType())
                .collect(Collectors.toSet());

        assertTrue(types.contains(UsnsTransactionType.DOMESTIC));
        assertTrue(types.contains(UsnsTransactionType.ICAO_NOTAMN));
        assertTrue(types.contains(UsnsTransactionType.ICAO_NOTAMC));
        assertTrue(types.contains(UsnsTransactionType.CANADIAN_DOMESTIC));
        assertTrue(types.contains(UsnsTransactionType.SERVICE_REQUEST));
        assertTrue(types.contains(UsnsTransactionType.SERVICE_TABLE));
        assertTrue(types.contains(UsnsTransactionType.GENOT));

        NotamAirspaceParser notamParser = new NotamAirspaceParser();
        NotamFieldParseResult icao = transactions.stream()
                .filter(transaction -> transaction.getType() == UsnsTransactionType.ICAO_NOTAMN)
                .findFirst()
                .map(transaction -> notamParser.parseFields(transaction.getRawText()))
                .orElseThrow(AssertionError::new);
        NotamFieldParseResult canadian = transactions.stream()
                .filter(transaction -> transaction.getType() == UsnsTransactionType.CANADIAN_DOMESTIC)
                .findFirst()
                .map(transaction -> notamParser.parseFields(transaction.getRawText()))
                .orElseThrow(AssertionError::new);

        assertEquals("KZNY", icao.getAccountability());
        assertEquals("QWMLW", icao.getQCode());
        assertTrue(icao.isHasGeometry());
        assertEquals("NOTAMJ", canadian.getNotamType());
        assertEquals("CYYZ", canadian.getAField());
        assertTrue(canadian.isPermanentEnd());
    }

    private String domestic(String body) {
        return "!DCA LDN " + body + " 1012211200-1012211300";
    }

    private GeoCoordinate point(double latitude, double longitude) {
        return GeoCoordinate.builder().latitude(latitude).longitude(longitude).altitude(0).build();
    }

    private List<String> lines(String name) {
        return Arrays.stream(resource(name).split("\\R"))
                .map(String::trim)
                .filter(line -> !line.isEmpty())
                .collect(Collectors.toList());
    }

    private String resource(String name) {
        try {
            return new String(getClass().getResourceAsStream("/legacy/grammar-parity/" + name).readAllBytes(),
                    StandardCharsets.UTF_8);
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }
}
