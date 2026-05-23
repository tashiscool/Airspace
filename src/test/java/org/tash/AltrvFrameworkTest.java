package org.tash;

import org.junit.jupiter.api.Test;
import org.tash.data.GeoCoordinate;
import org.tash.extensions.carf.altrv.AltrvParseResult;
import org.tash.extensions.carf.altrv.AltrvParser;
import org.tash.extensions.carf.altrv.AltrvReservationMapper;
import org.tash.extensions.carf.altrv.AltrvRouteGraphVertexType;
import org.tash.extensions.carf.altrv.AltrvSpatialMapper;
import org.tash.extensions.carf.altrv.AltrvRouteEventType;
import org.tash.extensions.carf.altrv.AltrvRouteGraph;
import org.tash.extensions.carf.altrv.AltrvRouteGraphBuilder;
import org.tash.extensions.carf.altrv.AltrvRouteGraphValidation;
import org.tash.extensions.carf.altrv.AltrvRouteGraphValidator;
import org.tash.extensions.carf.altrv.AltrvTokenType;
import org.tash.extensions.carf.altrv.AltrvTokenizer;
import org.tash.extensions.carf.api.CarfAnalysisResult;
import org.tash.extensions.carf.api.CarfAnalysisService;
import org.tash.extensions.carf.lifecycle.ReservationLifecycleInput;
import org.tash.extensions.carf.lifecycle.ReservationLifecycleResult;
import org.tash.extensions.carf.lifecycle.ReservationLifecycleService;
import org.tash.extensions.carf.lifecycle.ReservationLifecycleState;
import org.tash.extensions.carf.refdata.CarfSchemaCategory;
import org.tash.extensions.carf.refdata.CarfSchemaCatalog;
import org.tash.extensions.carf.refdata.CarfSchemaTable;
import org.tash.extensions.carf.refdata.CarfSchemaUse;
import org.tash.extensions.carf.refdata.InMemoryCarfReferenceDataProvider;
import org.tash.extensions.messaging.MessageControlCharacters;
import org.tash.extensions.messaging.UsnsIngestResult;
import org.tash.extensions.messaging.UsnsIngestService;
import org.tash.extensions.reservation.AirspaceReservation;
import org.tash.extensions.reservation.CarfReservationEvent;

import java.time.Duration;
import java.time.ZonedDateTime;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static org.junit.jupiter.api.Assertions.*;

class AltrvFrameworkTest {
    @Test
    void tokenizerClassifiesAltrvSectionsCoordinatesTimesAndFlightLevels() {
        List<org.tash.extensions.carf.altrv.AltrvToken> tokens = new AltrvTokenizer().tokenize(altrvMessage());

        assertTrue(tokens.stream().anyMatch(t -> t.getType() == AltrvTokenType.SECTION_LABEL && "A".equals(t.getText())));
        assertTrue(tokens.stream().anyMatch(t -> t.getType() == AltrvTokenType.COORDINATE_LATITUDE));
        assertTrue(tokens.stream().anyMatch(t -> t.getType() == AltrvTokenType.COORDINATE_LONGITUDE));
        assertTrue(tokens.stream().anyMatch(t -> t.getType() == AltrvTokenType.FLIGHT_LEVEL_RANGE));
        assertTrue(tokens.stream().anyMatch(t -> t.getType() == AltrvTokenType.TIME && "0100".equals(t.getText())));
        assertTrue(tokens.stream().allMatch(t -> t.getSourceSpan() != null && t.getEndOffset() >= t.getOffset()));
    }

    @Test
    void parserDetectsModernCoverageSurfaceFromLegacyAltrvGrammar() {
        AltrvParseResult result = new AltrvParser().parse(altrvMessage());

        assertTrue(result.isAccepted(), result.getDiagnostics().toString());
        assertEquals("TEST01", result.getMessage().getActivityName());
        assertEquals(2, result.getMessage().getRouteGroups().get(0).getRoutes().get(0).getPoints().size());
        assertEvent(result, AltrvRouteEventType.ADMIS_SECONDS);
        assertEvent(result, AltrvRouteEventType.AVANA);
        assertEvent(result, AltrvRouteEventType.LVLOF_BY);
        assertEvent(result, AltrvRouteEventType.AIRFL);
        assertEvent(result, AltrvRouteEventType.IFPFP);
        assertEvent(result, AltrvRouteEventType.JOIN);
        assertEvent(result, AltrvRouteEventType.LEAVE);
        assertEvent(result, AltrvRouteEventType.LAND);
        assertEvent(result, AltrvRouteEventType.ALTRV_ENDS);
        assertFalse(result.getSectionResults().isEmpty());
        assertFalse(result.getSourceSpans().isEmpty());
        assertTrue(result.getErrors().isEmpty(), result.getErrors().toString());
        assertEquals(org.tash.extensions.carf.altrv.AltrvRouteKind.IMPLICIT,
                result.getMessage().getRouteGroups().get(0).getRoutes().get(0).getKind());
    }

    @Test
    void parserModelsAdvancedRouteEventsAreasAndStationarySections() {
        String message = "1. STATIONARY AREA BNDD BY 3000N 15000W 0000 3100N 15000W 0010 3100N 15100W 0020 WITHIN 25 RADIUS\n"
                + "2. COMMENTS ONLY BROAD FRONT ACCELERATE TO SUPERSONIC END SUPERSONIC CROSS ABOVE ARCP ARIP AIRFL DRCT ENCAN EXCAN RAVEC";

        AltrvParseResult result = new AltrvParser().parse(message);

        assertTrue(result.isAccepted(), result.getDiagnostics().toString());
        assertNotNull(result.getMessage().getStationaryReservation());
        assertTrue(result.getMessage().getAreas().stream()
                .anyMatch(area -> area.getType() == org.tash.extensions.carf.altrv.AltrvAreaType.POLYGON));
        assertTrue(result.getMessage().getAreas().stream()
                .anyMatch(area -> area.getType() == org.tash.extensions.carf.altrv.AltrvAreaType.CIRCLE));
        assertEvent(result, AltrvRouteEventType.BROAD_FRONT);
        assertEvent(result, AltrvRouteEventType.SUPERSONIC);
        assertEvent(result, AltrvRouteEventType.CROSS_ABOVE);
        assertEvent(result, AltrvRouteEventType.AIR_REFUELING_CONTROL_POINT);
        assertEvent(result, AltrvRouteEventType.AIR_REFUELING_INITIAL_POINT);
        assertEvent(result, AltrvRouteEventType.AIR_REFUELING_DIRECT);
        assertEvent(result, AltrvRouteEventType.CANADA_ENTER);
        assertEvent(result, AltrvRouteEventType.CANADA_EXIT);
        assertEvent(result, AltrvRouteEventType.RAVEC);
    }

    @Test
    void parserPreservesLegacyRouteFamiliesAndEventMetadata() {
        String message = "A. TEST01\n"
                + "B. 2/F16\n"
                + "C. LOCATION\n"
                + "D. BEGIN CMN RTE TEST01 TEST02 FL240B280 FIXA 0000 JOIN CMN RTE TO FIXB 0010 ALTRV ENDS "
                + "BEGIN BRANCH RTE TEST02 FL260 FIXB 0010 LEAVE TEST01 FIXC 0020 CLOSE BRANCH MERGE "
                + "BEGIN PARTIAL RTE TEST03 FIXC 0020 FIXD 0030 ALTRV ENDS "
                + "BEGIN ALT DPRT RTE TEST04 CLMB FL300 FIXE 0040 ALTRV ENDS "
                + "REVERSE COURSE ALTRV TO FIXF FL240 TO CELL-1\n"
                + "E. TEST01 TEST02\n"
                + "F. TEST01 ETD 021300 MAR 2010 AVANA 021400 ADMIS 30 SEC\n"
                + "G. TAS: 480 KTAS";

        AltrvParseResult result = new AltrvParser().parse(message);

        assertTrue(result.isAccepted(), result.getDiagnostics().toString());
        assertTrue(result.getMessage().getRouteGroups().get(0).getRoutes().stream()
                .anyMatch(route -> route.getKind() == org.tash.extensions.carf.altrv.AltrvRouteKind.COMMON));
        assertTrue(result.getMessage().getRouteGroups().get(0).getRoutes().stream()
                .anyMatch(route -> route.getKind() == org.tash.extensions.carf.altrv.AltrvRouteKind.BRANCH));
        assertTrue(result.getMessage().getRouteGroups().get(0).getRoutes().stream()
                .anyMatch(route -> route.getKind() == org.tash.extensions.carf.altrv.AltrvRouteKind.PARTIAL));
        assertTrue(result.getMessage().getRouteGroups().get(0).getRoutes().stream()
                .anyMatch(route -> route.getKind() == org.tash.extensions.carf.altrv.AltrvRouteKind.ALTERNATE_DEPARTURE));
        assertTrue(result.getMessage().getRouteGroups().get(0).getRoutes().stream()
                .anyMatch(route -> route.getKind() == org.tash.extensions.carf.altrv.AltrvRouteKind.REVERSE));
        assertTrue(result.getMessage().getEvents().stream()
                .anyMatch(event -> "ALTRV.g".equals(event.getMetadata().get("grammarSource"))
                        && event.getMetadata().containsKey("flightLevel")));
        assertTrue(result.getMessage().getEvents().stream()
                .anyMatch(event -> event.getType() == AltrvRouteEventType.CLIMB
                        && "FIXE".equals(event.getMetadata().get("fix"))
                        && "0040".equals(event.getMetadata().get("elapsedTime"))
                        && "FL300".equals(event.getMetadata().get("flightLevel"))
                        && "LOCAL_EVENT_WINDOW".equals(event.getMetadata().get("scope"))));
        assertTrue(result.getMessage().getEvents().stream()
                .anyMatch(event -> event.getType() == AltrvRouteEventType.JOIN_COMMON_ROUTE
                        && "JOIN".equals(event.getMetadata().get("enterExitAssociation"))));
        assertTrue(result.getMessage().getEvents().stream()
                .anyMatch(event -> event.getType() == AltrvRouteEventType.BRANCH_CLOSE
                        && "CLOSE_BRANCH".equals(event.getMetadata().get("enterExitAssociation"))));
        assertTrue(result.getMessage().getEvents().stream()
                .anyMatch(event -> event.getType() == AltrvRouteEventType.ADMIS_SECONDS
                        && "30".equals(event.getMetadata().get("intervalSeconds"))));
    }

    @Test
    void parserPreservesLineCorridorEitherSideAreaMetadata() {
        String message = "1. STATIONARY RESERVATION 20 NM EITHER SIDE FIXA 0000 FIXB 0010 FROM 021200 MAR 2010 TO 021400 MAR 2010 SFC TO FL180\n"
                + "2. COMMENTS ONLY";
        Map<String, GeoCoordinate> fixes = new HashMap<>();
        fixes.put("FIXA", point(30, -150));
        fixes.put("FIXB", point(31, -149));

        AltrvParseResult result = new AltrvParser().parse(message);

        assertTrue(result.isAccepted(), result.getDiagnostics().toString());
        assertTrue(result.getMessage().getAreas().stream()
                .anyMatch(area -> area.getType() == org.tash.extensions.carf.altrv.AltrvAreaType.LINE
                        && area.getWidthNauticalMiles() == 40.0
                        && "LINE_CORRIDOR".equals(area.getGeometryIntent())
                        && "STATIONARY_RESERVATION".equals(area.getEnterExitAssociation())
                        && "SFC".equals(area.getLowerFlightLevel())
                        && "180".equals(area.getUpperFlightLevel())
                        && area.getTimingText() != null));
        List<CarfReservationEvent> events = new AltrvSpatialMapper()
                .toReservationEvents(result.getMessage(), new InMemoryCarfReferenceDataProvider(fixes));
        assertTrue(events.stream().anyMatch(event -> event.getRouteWidthNauticalMiles() == 40.0
                && event.getLowerAltitudeFeet() == 0
                && event.getUpperAltitudeFeet() == 18000
                && "LINE_CORRIDOR".equals(event.getShapeIntent())
                && event.getSourceText().contains("corridorWidthNm=40.0")));
    }

    @Test
    void spatialMapperPreservesLegacyAltitudeForms() {
        Map<String, GeoCoordinate> fixes = new HashMap<>();
        fixes.put("FIXA", point(30, -150));
        fixes.put("FIXB", point(31, -149));

        assertMappedAltitude("D. FIXA 0000 FIXB 0010 SURFACE TO UNLIMITED", fixes, 0, 100000);
        assertMappedAltitude("D. FIXA 0000 FIXB 0010 ABV FL240", fixes, 24000, 100000);
        assertMappedAltitude("D. FIXA 0000 FIXB 0010 BLW FL180", fixes, 0, 18000);
        assertMappedAltitude("D. FIXA 0000 FIXB 0010 FL260", fixes, 26000, 26000);
    }

    @Test
    void messageValidatorPortsCoreLegacyValidationRules() {
        String invalid = "A. TOOLONG01 TEST02 TEST02\n"
                + "B. 1/F16\n"
                + "C. LOCATION\n"
                + "D. FL240B260 3000N 15000W 0000 3000N 15100W 0100\n"
                + "F. TEST01 ETD 021300 MAR 2010 AVANA 021302\n"
                + "G. COMMENTS";

        AltrvParseResult result = new AltrvParser().parse(invalid);

        assertTrue(result.getDiagnostics().stream().anyMatch(d -> d.contains("exceeds seven")));
        assertTrue(result.getDiagnostics().stream().anyMatch(d -> d.contains("Duplicate callsign")));
        assertTrue(result.getDiagnostics().stream().anyMatch(d -> d.contains("different than number of aircraft types")));
        assertTrue(result.getDiagnostics().stream().anyMatch(d -> d.contains("do not match callsigns in F section")));
        assertTrue(result.getDiagnostics().stream().anyMatch(d -> d.contains("Multi-aircraft departure groups require ADMIS")));
        assertTrue(result.getDiagnostics().stream().anyMatch(d -> d.contains("AVANA must be 5 minutes")));
        assertFalse(result.getErrors().isEmpty());
        assertTrue(result.getTypedDiagnostics().stream()
                .anyMatch(d -> d.getSeverity() == org.tash.extensions.carf.altrv.AltrvDiagnosticSeverity.ERROR));
    }

    @Test
    void routeGraphValidationUsesReferenceDataForNamedFixes() {
        String namedFixMessage = "A. NAMED01\n"
                + "B. MISSION\n"
                + "C. LOCATION\n"
                + "D. FL240B260 FIXA 0000 FIXB 0100 JOIN BLUE1 LEAVE RED1\n"
                + "F. ETD 021300 MAR 2010 AVANA 021400\n"
                + "G. TAS: 480 KTAS";
        AltrvParseResult result = new AltrvParser().parse(namedFixMessage);
        AltrvRouteGraph graph = new AltrvRouteGraphBuilder().build(result.getMessage());
        Map<String, GeoCoordinate> fixes = new HashMap<>();
        fixes.put("FIXA", point(30, -150));
        fixes.put("FIXB", point(30, -151));

        AltrvRouteGraphValidation valid = new AltrvRouteGraphValidator()
                .validate(result.getMessage(), graph, new InMemoryCarfReferenceDataProvider(fixes));
        AltrvRouteGraphValidation unresolved = new AltrvRouteGraphValidator()
                .validate(result.getMessage(), graph, new InMemoryCarfReferenceDataProvider(Collections.emptyMap()));

        assertTrue(valid.isValid(), valid.getDiagnostics().toString());
        assertTrue(unresolved.isValid(), unresolved.getDiagnostics().toString());
        assertTrue(unresolved.getDiagnostics().stream().anyMatch(d -> d.contains("Unresolved fix/navaid FIXA")));
        assertTrue(graph.getVertices().values().stream().anyMatch(v -> v.getType() == AltrvRouteGraphVertexType.DEPARTURE));
        assertTrue(graph.getVertices().values().stream().anyMatch(v -> v.getType() == AltrvRouteGraphVertexType.ROUTE_START));
        assertTrue(graph.getVertices().values().stream().anyMatch(v -> v.getType() == AltrvRouteGraphVertexType.ROUTE_GROUP));
        assertTrue(graph.getEdges().stream().anyMatch(e -> "departure-to-route".equals(e.getReason())));
    }

    @Test
    void routeGraphValidationFlagsNonMonotonicElapsedTimes() {
        String message = "A. TIME01\n"
                + "B. MISSION\n"
                + "C. LOCATION\n"
                + "D. FL240B260 3000N 15000W 0100 3000N 15100W 0030\n"
                + "F. ETD 021300 MAR 2010 AVANA 021400\n"
                + "G. TAS: 480 KTAS";
        AltrvParseResult result = new AltrvParser().parse(message);
        AltrvRouteGraph graph = new AltrvRouteGraphBuilder().build(result.getMessage());

        AltrvRouteGraphValidation validation = new AltrvRouteGraphValidator()
                .validate(result.getMessage(), graph, null);

        assertTrue(validation.getDiagnostics().stream().anyMatch(d -> d.contains("smaller than preceding route time")));
        assertFalse(validation.isValid());
        assertTrue(validation.getTypedDiagnostics().stream()
                .anyMatch(d -> d.getSeverity() == org.tash.extensions.carf.altrv.AltrvDiagnosticSeverity.ERROR));
    }

    @Test
    void spatialMapperConvertsAltrvAreasIntoReservationEvents() {
        String message = "1. STATIONARY AREA BNDD BY 3000N 15000W 0000 3100N 15000W 0010 3100N 15100W 0020 WITHIN 25 RADIUS\n"
                + "2. COMMENTS ONLY";
        AltrvParseResult result = new AltrvParser().parse(message);

        List<CarfReservationEvent> events = new AltrvSpatialMapper()
                .toReservationEvents(result.getMessage(), new InMemoryCarfReferenceDataProvider(Collections.emptyMap()));

        assertFalse(events.isEmpty());
        assertTrue(events.stream().anyMatch(e -> e.getType() == org.tash.extensions.reservation.CarfReservationEventType.STATIONARY_AREA));
        assertTrue(events.stream().anyMatch(e -> e.getType() == org.tash.extensions.reservation.CarfReservationEventType.ORBIT));
        assertTrue(events.stream().anyMatch(e -> "POLYGON".equals(e.getShapeIntent())
                && e.getSourceText().contains("geometryIntent=POLYGON")));
        assertTrue(events.stream().anyMatch(e -> "POINT_RADIUS".equals(e.getShapeIntent())
                && e.getSourceText().contains("radiusNm=25.0")));
        assertTrue(result.getMessage().getAreas().stream()
                .anyMatch(area -> "POLYGON".equals(area.getGeometryIntent())
                        && "ALTRV.g".equals(area.getMetadata().get("grammarSource"))));
        assertTrue(result.getMessage().getAreas().stream()
                .anyMatch(area -> "POINT_RADIUS".equals(area.getGeometryIntent())
                        && area.getRadiusNauticalMiles() == 25.0));
    }

    @Test
    void reservationMapperPreservesGraphAndFeatureMetadata() {
        AltrvParseResult result = new AltrvParser().parse(altrvMessage());

        List<AirspaceReservation> reservations = new AltrvReservationMapper()
                .toReservations(result.getMessage(), null);

        assertEquals(1, reservations.size());
        assertNotNull(reservations.get(0).getProtectedVolume());
        assertFalse(reservations.get(0).getRouteGraphNodeIds().isEmpty());
        assertTrue(reservations.get(0).getSourceText().contains("ADMIS_SECONDS"));
    }

    @Test
    void carfAnalysisFacadeParsesMapsAndAnalyzesConflicts() {
        CarfAnalysisService service = new CarfAnalysisService();
        CarfAnalysisResult first = service.parseAndMap(altrvMessage());
        CarfAnalysisResult second = service.parseAndMap(altrvMessage().replace("TEST01", "TEST02"));

        assertTrue(first.isAccepted(), first.getDiagnostics().toString());
        assertEquals("TEST01", first.getSourceMetadata().get("activityName"));
        assertEquals(1, first.getReservations().size());

        java.util.ArrayList<AirspaceReservation> all = new java.util.ArrayList<>();
        all.addAll(first.getReservations());
        all.addAll(second.getReservations());
        CarfAnalysisResult conflictResult = service.analyzeConflicts(all);

        assertTrue(conflictResult.isAccepted());
        assertFalse(conflictResult.getConflicts().isEmpty());
        assertTrue(conflictResult.getConflicts().get(0).getExplanation().contains("lateralDistance"));
    }

    @Test
    void lifecycleServiceCapturesScarfCompletionAndStaleLockRules() {
        ReservationLifecycleResult result = new ReservationLifecycleService().evaluate(ReservationLifecycleInput.builder()
                .currentState(ReservationLifecycleState.APPROVED)
                .reservationEndTime(ZonedDateTime.parse("2010-03-02T15:00:00Z"))
                .lockedAt(ZonedDateTime.parse("2010-03-02T14:00:00Z"))
                .evaluationTime(ZonedDateTime.parse("2010-03-02T16:00:00Z"))
                .completionBuffer(Duration.ofMinutes(30))
                .staleLockThreshold(Duration.ofMinutes(20))
                .build());

        assertEquals(ReservationLifecycleState.COMPLETED, result.getState());
        assertTrue(result.getActions().contains("mark-approved-reservation-complete-after-flown"));
        assertTrue(result.getActions().contains("release-stale-lock"));
    }

    @Test
    void schemaCatalogDocumentsTrainingBackupTables() {
        CarfSchemaCatalog catalog = new CarfSchemaCatalog();

        assertTrue(catalog.coreTrainingBackupTables().contains("t_ALTRVMessage"));
        assertTrue(catalog.coreTrainingBackupTables().contains("t_Navaids"));
        assertTrue(catalog.coreTrainingBackupTables().size() >= 52);
        assertEquals("AltrvRoutePoint", catalog.domainMappingFor("t_FixTime"));
        assertEquals("CarfReferenceDataProvider", catalog.domainMappingFor("t_Navaids"));
        assertEquals(CarfSchemaCategory.ROUTE_GRAPH, catalog.categoryFor("t_Route"));
        assertEquals(CarfSchemaUse.IMPLEMENTED, catalog.useFor("t_Reservation"));
        assertTrue(catalog.tablesByCategory(CarfSchemaCategory.SPATIAL_AREA).stream()
                .map(CarfSchemaTable::getTableName)
                .anyMatch("t_AreaFix"::equals));
        assertTrue(catalog.tablesByUse(CarfSchemaUse.ADAPTER_TARGET).stream()
                .anyMatch(table -> table.getTableName().equals("t_IncomingQueue") && table.isQueueTable()));
        assertTrue(catalog.table("t_ALTRVMessage")
                .map(CarfSchemaTable::getChildTables)
                .orElse(Collections.emptyList())
                .contains("t_RouteGroup"));
    }

    @Test
    void usnsCarfIngestExposesModernAnalysisResult() {
        UsnsIngestResult result = new UsnsIngestService().parse(envelope(altrvMessage()));

        assertTrue(result.getErrors().isEmpty(), result.getErrors().toString());
        assertEquals(1, result.getCarfMessages().size());
        assertEquals(1, result.getCarfAnalysisResults().size());
        assertTrue(result.getCarfAnalysisResults().get(0).isAccepted(),
                result.getCarfAnalysisResults().get(0).getDiagnostics().toString());
    }

    @Test
    void usnsClassifiedFamiliesReturnTypedClassifiedOnlyResults() {
        UsnsIngestResult result = new UsnsIngestService().parse(envelope("SNOWTAM 0123\nGENOT RWA ABC"));

        assertTrue(result.getErrors().isEmpty(), result.getErrors().toString());
        assertFalse(result.getFamilyParseResults().isEmpty());
        assertTrue(result.getFamilyParseResults().stream().allMatch(f ->
                f.getStatus() == org.tash.extensions.messaging.CarfMessageFamilyStatus.SUPPORTED));
    }

    private void assertEvent(AltrvParseResult result, AltrvRouteEventType type) {
        assertTrue(result.getMessage().getEvents().stream().anyMatch(e -> e.getType() == type),
                "Missing event " + type + " in " + result.getMessage().getEvents());
    }

    private String altrvMessage() {
        return "A. TEST01\n"
                + "B. MISSION\n"
                + "C. LOCATION\n"
                + "D. FL240B260 3000N 15000W 0000 3000N 15100W 0100 LVLOF BY 0030 AIRFL IFPFP JOIN BLUE1 LEAVE RED1 LAND ALTRV ENDS\n"
                + "F. ETD 021300 MAR 2010 ADMIS 20 SEC AVANA 021400\n"
                + "G. TAS: 480 KTAS";
    }

    private String envelope(String body) {
        return "01GGNC07GP\n"
                + "CNS000 300334\n"
                + "GG KDZZNAXX\n"
                + "300334 KGPS\n"
                + MessageControlCharacters.STX + body
                + MessageControlCharacters.VT + MessageControlCharacters.ETX;
    }

    private GeoCoordinate point(double latitude, double longitude) {
        return GeoCoordinate.builder().latitude(latitude).longitude(longitude).altitude(0).build();
    }

    private void assertMappedAltitude(String routeLine,
                                      Map<String, GeoCoordinate> fixes,
                                      double expectedLower,
                                      double expectedUpper) {
        String message = "A. ALT01\n"
                + "B. MISSION\n"
                + "C. LOCATION\n"
                + routeLine + "\n"
                + "F. ETD 021300 MAR 2010 AVANA 021400\n"
                + "G. COMMENTS";
        AltrvParseResult result = new AltrvParser().parse(message);
        List<CarfReservationEvent> events = new AltrvSpatialMapper()
                .toReservationEvents(result.getMessage(), new InMemoryCarfReferenceDataProvider(fixes));
        assertFalse(events.isEmpty(), result.getDiagnostics().toString());
        assertEquals(expectedLower, events.get(0).getLowerAltitudeFeet());
        assertEquals(expectedUpper, events.get(0).getUpperAltitudeFeet());
    }
}
