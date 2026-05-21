package org.tash.extensions.product;

import org.junit.jupiter.api.Test;
import org.tash.extensions.engine.OperationalDecisionEngine;
import org.tash.extensions.engine.OperationalDecisionRequest;
import org.tash.extensions.engine.OperationalDecisionResult;
import org.tash.extensions.engine.ReplayVerificationResult;
import org.tash.extensions.product.api.DecisionResource;
import org.tash.extensions.product.api.ProductAuthFilter;
import org.tash.extensions.product.application.AirspaceProductService;
import org.tash.extensions.product.application.ProductSearchService;
import org.tash.extensions.product.auth.AuthService;
import org.tash.extensions.product.dto.AuthDtos;
import org.tash.extensions.product.dto.ProductDtos;
import org.tash.extensions.feed.LocalReferenceDataSyncAdapter;
import org.tash.extensions.feed.LocalUsnsTrafficAdapter;
import org.tash.extensions.feed.LocalWeatherFeedAdapter;
import org.tash.extensions.feed.OperationalFeedIngestService;
import org.tash.extensions.feed.ReferenceDataSyncResult;
import org.tash.extensions.product.persistence.entity.AltrvAreaEntity;
import org.tash.extensions.product.persistence.entity.AltrvMessageEntity;
import org.tash.extensions.product.persistence.entity.AltrvRouteEntity;
import org.tash.extensions.product.persistence.entity.AltrvRouteEventEntity;
import org.tash.extensions.product.persistence.entity.AltrvRouteGroupEntity;
import org.tash.extensions.product.persistence.entity.ApreqEntity;
import org.tash.extensions.product.persistence.entity.ApprovalEntity;
import org.tash.extensions.product.persistence.entity.FeedArtifactEntity;
import org.tash.extensions.product.persistence.entity.HistoryEventEntity;
import org.tash.extensions.product.persistence.entity.MessageEntity;
import org.tash.extensions.product.persistence.entity.MissionEntity;
import org.tash.extensions.product.persistence.entity.NotamEntity;
import org.tash.extensions.product.persistence.entity.OperationalDecisionEntity;
import org.tash.extensions.product.persistence.entity.ReferencePointEntity;
import org.tash.extensions.product.persistence.entity.ReservationEntity;
import org.tash.extensions.product.persistence.mapper.ProductEntityMapper;
import org.tash.extensions.weather.product.WeatherProduct;
import org.tash.extensions.weather.product.WeatherProductParseResult;
import org.tash.extensions.weather.product.WeatherProductParser;
import org.tash.extensions.weather.product.WeatherProductType;
import org.tash.extensions.weather.decision.CwapCalibrationDataset;
import org.tash.extensions.weather.decision.HistoricalWeatherOutcome;
import org.tash.extensions.weather.decision.RouteImpactCalibrationModel;
import org.tash.extensions.weather.decision.StormCellLifecycle;
import org.tash.extensions.weather.decision.StormCellLifecycleTracker;
import org.tash.extensions.weather.decision.StormCellObservation;
import org.tash.extensions.workflow.InMemoryReservationWorkflowRepository;
import org.tash.extensions.workflow.ReservationWorkflowService;

import java.io.IOException;
import java.lang.reflect.Method;
import java.nio.charset.StandardCharsets;
import java.time.ZonedDateTime;
import java.util.Collections;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class ProductPersistenceAndServiceTest {
    @Test
    void entityMapperCapturesWeatherAndDecisionJsonForPostgresPersistence() {
        WeatherProductParseResult parsed = new WeatherProductParser().parse(
                "METAR KJFK 200000Z 18012G22KT 1/2SM +TSRA BKN004 OVC010 18/16 A2992",
                WeatherProductType.METAR);
        WeatherProduct product = parsed.getProduct();
        OperationalDecisionResult decision = new OperationalDecisionEngine().evaluate(OperationalDecisionRequest.builder()
                .decisionTime(ZonedDateTime.parse("2026-05-20T00:00:00Z"))
                .weatherProducts(Collections.singletonList(product))
                .build());

        ProductEntityMapper mapper = new ProductEntityMapper();
        org.tash.extensions.product.persistence.entity.WeatherProductEntity weatherEntity = mapper.toEntity(product);
        OperationalDecisionEntity decisionEntity = mapper.toEntity(decision);

        assertEquals(product.getId(), weatherEntity.getId());
        assertEquals("METAR", weatherEntity.getProductType());
        assertTrue(weatherEntity.getProductJson().contains("KJFK"));
        assertEquals(decision.getAction().name(), decisionEntity.getAction());
        assertTrue(decisionEntity.getResultJson().contains("confidence"));
        assertNotNull(decisionEntity.getAuditJson());
        assertNotNull(decisionEntity.getReplayJson());
    }

    @Test
    void decisionResourceVerifiesPersistedReplayJsonWhenTypedResultIsUnavailable() throws Exception {
        OperationalDecisionResult decision = new OperationalDecisionEngine().evaluate(OperationalDecisionRequest.builder()
                .decisionTime(ZonedDateTime.parse("2026-05-20T00:00:00Z"))
                .build());
        ProductDtos.DecisionSummary persisted = ProductDtos.DecisionSummary.builder()
                .id("decision-1")
                .replayJson(org.tash.extensions.engine.CanonicalJson.write(decision.getReplayBundle()))
                .auditJson(org.tash.extensions.engine.CanonicalJson.write(decision.getAuditEnvelope()))
                .build();

        Method verifier = DecisionResource.class.getDeclaredMethod("verifyPersistedReplay", ProductDtos.DecisionSummary.class);
        verifier.setAccessible(true);
        ReplayVerificationResult accepted = (ReplayVerificationResult) verifier.invoke(new DecisionResource(), persisted);
        assertTrue(accepted.isAccepted(), accepted.getErrors().toString());
        assertFalse(accepted.getWarnings().isEmpty());

        ProductDtos.DecisionSummary missing = ProductDtos.DecisionSummary.builder().id("decision-2").build();
        ReplayVerificationResult rejectedMissing = (ReplayVerificationResult) verifier.invoke(new DecisionResource(), missing);
        assertFalse(rejectedMissing.isAccepted());
        assertTrue(rejectedMissing.getErrors().get(0).contains("persisted replay JSON is missing"));

        ProductDtos.DecisionSummary tampered = ProductDtos.DecisionSummary.builder()
                .id("decision-3")
                .replayJson(persisted.getReplayJson())
                .auditJson(persisted.getAuditJson().replace(decision.getAuditEnvelope().getResultHash(), "tampered"))
                .build();
        ReplayVerificationResult rejectedTampered = (ReplayVerificationResult) verifier.invoke(new DecisionResource(), tampered);
        assertFalse(rejectedTampered.isAccepted());
        assertTrue(rejectedTampered.getErrors().stream().anyMatch(error -> error.contains("result hash")));
    }

    @Test
    void localAuthServiceExposesPlannerSupervisorAndAdminRoleChecks() {
        AuthService auth = new AuthService();
        AuthDtos.SessionResponse planner = auth.login(login("planner", "planner"));
        AuthDtos.SessionResponse supervisor = auth.login(login("supervisor", "supervisor"));
        AuthDtos.SessionResponse admin = auth.login(login("admin", "admin"));

        assertTrue(auth.hasAnyRole("Bearer " + planner.getToken(), "PLANNER"));
        assertFalse(auth.hasAnyRole("Bearer " + planner.getToken(), "SUPERVISOR"));
        assertTrue(auth.hasAnyRole("Bearer " + supervisor.getToken(), "SUPERVISOR"));
        assertTrue(auth.hasAnyRole("Bearer " + admin.getToken(), "ADMIN"));
        assertTrue(auth.hasAnyRole("Bearer " + admin.getToken(), "PLANNER", "SUPERVISOR"));
    }

    @Test
    void productAuthFilterMapsWorkflowPathsToConcreteRoles() throws Exception {
        ProductAuthFilter filter = new ProductAuthFilter();
        Method publicPath = ProductAuthFilter.class.getDeclaredMethod("isPublicPath", String.class);
        Method requiredRoles = ProductAuthFilter.class.getDeclaredMethod("requiredRoles", String.class, String.class);
        publicPath.setAccessible(true);
        requiredRoles.setAccessible(true);

        assertTrue((Boolean) publicPath.invoke(filter, "api/auth/login"));
        assertArrayEquals(new String[]{"SUPERVISOR", "ADMIN"},
                (String[]) requiredRoles.invoke(filter, "POST", "api/reservations/abc/approve"));
        assertArrayEquals(new String[]{"ADMIN"},
                (String[]) requiredRoles.invoke(filter, "POST", "api/reference/points"));
        assertArrayEquals(new String[]{"PLANNER", "SUPERVISOR", "ADMIN"},
                (String[]) requiredRoles.invoke(filter, "POST", "api/feed/ingest"));
        assertArrayEquals(new String[]{"OPERATOR", "PLANNER", "SUPERVISOR", "ADMIN"},
                (String[]) requiredRoles.invoke(filter, "GET", "api/missions"));
    }

    @Test
    void productServiceKeepsOperatorWorkspaceStateInMemoryForLocalMode() {
        AirspaceProductService service = new AirspaceProductService(new ReservationWorkflowService(new InMemoryReservationWorkflowRepository()));
        ProductDtos.MissionRequest missionRequest = new ProductDtos.MissionRequest();
        missionRequest.setMissionNumber("M-LOCAL");
        missionRequest.setTitle("Local Ops");
        missionRequest.setActor("planner");

        ProductDtos.MissionSummary mission = service.createMission(missionRequest);
        ProductDtos.MissionRequest secondMissionRequest = new ProductDtos.MissionRequest();
        secondMissionRequest.setMissionNumber("M-SECOND");
        secondMissionRequest.setTitle("Second Ops");
        ProductDtos.MissionSummary secondMission = service.createMission(secondMissionRequest);
        service.lockMission(mission.getId(), "planner");
        service.unlockMission(mission.getId(), "planner");

        ProductDtos.ReservationRequest reservationRequest = new ProductDtos.ReservationRequest();
        reservationRequest.setRawText("A. LOCAL\nB. 1F22/I");
        reservationRequest.setActor("planner");
        String localReservationId = service.createReservation(mission.getId(), reservationRequest).getRecord().getId();
        service.createReservation(secondMission.getId(), reservationRequest);
        reservationRequest.setRawText("A. LOCAL UPDATED\nB. 1F22/I");
        assertTrue(service.updateReservation(localReservationId, reservationRequest).isAccepted());

        ProductDtos.MessageRequest messageRequest = new ProductDtos.MessageRequest();
        messageRequest.setMissionId(mission.getId());
        messageRequest.setFamily("USNS");
        messageRequest.setDirection("OUTBOUND");
        messageRequest.setSubject("Coordination");
        messageRequest.setRawText("SVC RQ");
        ProductDtos.MessageSummary message = service.sendMessage(messageRequest);
        ProductDtos.MessageSummary reply = service.replyMessage(message.getId(), null);
        ProductDtos.MessageSummary forward = service.forwardMessage(message.getId(), null);
        assertEquals("RE: Coordination", reply.getSubject());
        assertEquals("FWD: Coordination", forward.getSubject());

        ProductDtos.FeedIngestRequest feed = new ProductDtos.FeedIngestRequest();
        feed.setSourceId("unit");
        feed.setType("WEATHER");
        feed.setRawPayload("METAR KJFK 200000Z 18012KT 2SM RA BKN010");
        service.ingestFeed(feed);
        assertFalse(service.feedTransactions(service.feedArtifacts().get(0).getId()).isEmpty());

        ProductDtos.DecisionEvaluateRequest decision = new ProductDtos.DecisionEvaluateRequest();
        decision.setDecisionTime("2026-05-20T00:00:00Z");
        ProductDtos.DecisionSummary summary = service.evaluateDecision(decision);
        assertEquals(summary.getAction(), service.decisionResult(summary.getId()).getAction().name());
        assertEquals(summary.getAction(), service.decisionReplayBundle(summary.getId()).getExpectedAction());
        assertEquals(service.decisionReplayBundle(summary.getId()).getExpectedResultHash(),
                service.decisionAuditEnvelope(summary.getId()).getResultHash());

        assertEquals("M-LOCAL", service.mission(mission.getId()).getMission().getMissionNumber());
        assertEquals(1, service.mission(mission.getId()).getReservations().size());
        assertEquals(localReservationId, service.mission(mission.getId()).getReservations().get(0).getId());
        assertEquals(message.getId(), service.message(message.getId()).getId());
        assertFalse(service.feedArtifacts().isEmpty());
        assertEquals(summary.getId(), service.decision(summary.getId()).getId());
        assertFalse(service.history().isEmpty());
        assertTrue(service.search("Local Ops").stream().anyMatch(result -> "mission".equals(result.getType())));
        assertTrue(service.search("Coordination").stream().anyMatch(result -> "message".equals(result.getType())));
        assertTrue(service.search("METAR").stream().anyMatch(result -> "feed".equals(result.getType())));
        assertTrue(service.search("MISSION_LOCKED").stream().anyMatch(result -> "history".equals(result.getType())));
        assertTrue(service.search("").size() >= 4);
        assertTrue(service.referenceMap("NAVAID").containsKey("JFK"));
        assertTrue(service.referencePoints("FIX").stream().anyMatch(point -> "3000N15000W".equals(point.getIdentifier())));
        ProductDtos.ReferencePointRequest pointRequest = new ProductDtos.ReferencePointRequest();
        pointRequest.setIdentifier("LOCALPOINT");
        pointRequest.setPointType("FIX");
        pointRequest.setLatitude(38.0);
        pointRequest.setLongitude(-77.0);
        ProductDtos.ReferencePointSummary point = service.createReferencePoint(pointRequest);
        assertEquals("LOCALPOINT", point.getIdentifier());
        assertTrue(service.referencePoints("").stream().anyMatch(saved -> "LOCALPOINT".equals(saved.getIdentifier())));
        assertThrows(IllegalArgumentException.class, () -> service.createReferencePoint(new ProductDtos.ReferencePointRequest()));
        ProductDtos.ReservationSupplementRequest supplementRequest = new ProductDtos.ReservationSupplementRequest();
        supplementRequest.setKind("APREQ");
        supplementRequest.setStatus("DRAFT");
        supplementRequest.setTitle("APREQ");
        supplementRequest.setText("Coordination request");
        supplementRequest.setActor("planner");
        ProductDtos.ReservationSupplementSummary supplement = service.createSupplement("reservation-local", supplementRequest);
        assertEquals("APREQ", supplement.getKind());
        ProductDtos.SupplementTransitionRequest transition = new ProductDtos.SupplementTransitionRequest();
        transition.setStatus("APPROVED");
        transition.setActor("supervisor");
        assertEquals("APPROVED", service.transitionSupplement(supplement.getId(), transition).getStatus());
        assertTrue(service.supplements("reservation-local").stream().anyMatch(saved -> supplement.getId().equals(saved.getId())));
        ProductDtos.ReservationSupplementSummary coordination = service.createSupplement("reservation-local", null);
        assertEquals("COORDINATION", coordination.getKind());
        assertEquals("OPEN", coordination.getStatus());
        ProductDtos.ReferenceDataImportRequest importRequest = new ProductDtos.ReferenceDataImportRequest();
        importRequest.setPayload("type,identifier,latitude,longitude,altitudeFeet,source,version\nFIX,LOCALSYNC,39.0,-76.0,0,unit,v1");
        ProductDtos.ReferenceDataImportResult preview = service.previewReferenceData(importRequest);
        assertTrue(preview.isAccepted());
        assertEquals(1, preview.getParsedCount());
        ProductDtos.ReferenceDataImportResult applied = service.applyReferenceData(importRequest);
        assertEquals(1, applied.getAppliedCount());
        assertTrue(service.referencePoints("FIX").stream().anyMatch(saved -> "LOCALSYNC".equals(saved.getIdentifier())));
        ProductSearchService searchService = new ProductSearchService(service);
        assertFalse(searchService.searchMissions("LOCAL").isEmpty());
        assertFalse(searchService.searchMessages("Coordination").isEmpty());
        assertFalse(searchService.searchDecisions(summary.getAction()).isEmpty());
        assertFalse(searchService.searchHistory("MISSION_LOCKED").isEmpty());
        assertNotNull(service.decisionRequest(summary.getId()));
        assertNotNull(service.rehydrateDecision(summary.getId()));
        assertEquals(localReservationId, service.rehydrateReservation(localReservationId).getId());
        java.util.Map<String, Double> metrics = service.metrics();
        assertTrue(metrics.get("product.records") >= 4.0);
        assertTrue(metrics.get("product.missions") >= 1.0);
        assertTrue(metrics.get("product.messages") >= 1.0);
        assertTrue(metrics.get("product.feedArtifacts") >= 1.0);
        assertTrue(metrics.get("product.referencePoints") >= 4.0);
        assertTrue(metrics.get("product.supplements") >= 1.0);
        assertTrue(metrics.containsKey("product.weather.affectedMissions"));
        assertTrue(metrics.containsKey("product.weather.averageGuidanceLatencySeconds"));
        assertTrue(metrics.containsKey("product.weather.guidanceTargetRate"));
    }

    @Test
    void localFeedAdaptersReferenceSyncAndCalibrationSeamsAreReadyForProductionAdapters() {
        OperationalFeedIngestService ingest = new OperationalFeedIngestService();
        assertEquals(1, ingest.ingest(new LocalUsnsTrafficAdapter("nadinsim",
                Collections.singletonList("!ABC ABC RWY CLSD 200000-200300")).poll()).getResults().size());
        assertEquals(1, ingest.ingest(new LocalWeatherFeedAdapter("wxs",
                Collections.singletonList("METAR KJFK 200000Z 18012KT 2SM RA BKN010")).poll()).getResults().size());

        ReferenceDataSyncResult sync = new LocalReferenceDataSyncAdapter().preview(
                "type,identifier,latitude,longitude,altitudeFeet,source,version\n"
                        + "NAVAID,JFK,40.6398,-73.7789,13,local,v1\n"
                        + "NAVAID,JFK,40.6398,-73.7789,13,local,v1");
        assertTrue(sync.isAccepted());
        assertEquals(2, sync.getRecords().size());
        assertFalse(sync.getWarnings().isEmpty());

        CwapCalibrationDataset dataset = CwapCalibrationDataset.builder()
                .id("cwaf-fixture")
                .source("historical-fixture")
                .outcomes(Collections.singletonList(HistoricalWeatherOutcome.builder()
                        .productId("wx-1")
                        .routeId("route-1")
                        .forecastTime(ZonedDateTime.parse("2026-05-20T00:00:00Z"))
                        .routeBlocked(true)
                        .observedDeviationRate(0.8)
                        .observedCapacityImpact(0.6)
                        .build()))
                .build();
        RouteImpactCalibrationModel loaded = RouteImpactCalibrationModel.load(dataset);
        assertEquals("loaded-cwaf-fixture", loaded.getVersion());
        assertTrue(loaded.getBlockedThreshold() > 0.5);

        List<StormCellLifecycle> lifecycles = new StormCellLifecycleTracker().update(Collections.emptyList(),
                Collections.singletonList(StormCellObservation.builder()
                        .observationId("storm-ob-1")
                        .productType(WeatherProductType.NEXRAD_POLYGON)
                        .observedAt(ZonedDateTime.parse("2026-05-20T00:00:00Z"))
                        .growthTrend(0.2)
                        .geometry(Collections.singletonList(org.tash.data.GeoCoordinate.builder()
                                .latitude(40).longitude(-75).altitude(0).build()))
                        .build()));
        assertEquals(1, lifecycles.size());
        assertFalse(lifecycles.get(0).getProducts().isEmpty());
    }

    @Test
    void persistenceEntitiesExposeOptimisticLockAndOperationalFields() {
        MissionEntity mission = new MissionEntity();
        mission.setMissionNumber("M-1");
        mission.setTitle("Mission");
        mission.setStatus("SUBMITTED");
        mission.setRawText("A. TEST");
        mission.setLockedBy("planner");
        mission.setLockedAt(ZonedDateTime.parse("2026-05-20T00:00:00Z"));
        mission.setVersion(3);

        ReservationEntity reservation = new ReservationEntity();
        reservation.setMission(mission);
        reservation.setReservationKey("R-1");
        reservation.setState("VALIDATED");
        reservation.setLowerAltitudeFeet(24000.0);
        reservation.setUpperAltitudeFeet(26000.0);
        reservation.setLastAnalysisJson("{}");

        ReferencePointEntity point = new ReferencePointEntity();
        point.setIdentifier("JFK");
        point.setPointType("NAVAID");
        point.setLatitude(40.0);
        point.setLongitude(-73.0);
        point.setAltitudeFeet(13.0);
        point.setSource("seed");

        assertEquals("M-1", mission.getMissionNumber());
        assertEquals(3, mission.getVersion());
        assertEquals("R-1", reservation.getReservationKey());
        assertEquals(24000.0, reservation.getLowerAltitudeFeet());
        assertEquals("JFK", point.getIdentifier());
        assertEquals("seed", point.getSource());

        MessageEntity message = new MessageEntity();
        java.util.UUID missionId = java.util.UUID.randomUUID();
        java.util.UUID reservationId = java.util.UUID.randomUUID();
        message.setMissionId(missionId);
        message.setReservationId(reservationId);
        message.setFamily("USNS");
        message.setDirection("INBOUND");
        message.setStatus("ACCEPTED");
        message.setSubject("Coordination");
        message.setRawText("SVC RQ");
        message.setParsedSummaryJson("{}");

        FeedArtifactEntity feed = new FeedArtifactEntity();
        feed.setId(java.util.UUID.randomUUID());
        feed.setSourceId("nadinsim");
        feed.setFeedType("WEATHER");
        feed.setPayloadHash("hash");
        feed.setRawPayload("METAR KJFK");
        feed.setDiagnosticsJson("{}");
        feed.setAccepted(true);
        feed.setReceivedAt(ZonedDateTime.parse("2026-05-20T00:00:00Z"));

        HistoryEventEntity history = new HistoryEventEntity();
        history.setId(java.util.UUID.randomUUID());
        history.setAggregateType("mission");
        history.setAggregateId(missionId.toString());
        history.setEventType("MISSION_CREATED");
        history.setActor("planner");
        history.setNote("created");
        history.setEventJson("{}");
        history.setCreatedAt(ZonedDateTime.parse("2026-05-20T00:00:00Z"));

        assertEquals(missionId, message.getMissionId());
        assertEquals(reservationId, message.getReservationId());
        assertEquals("USNS", message.getFamily());
        assertEquals("INBOUND", message.getDirection());
        assertEquals("ACCEPTED", message.getStatus());
        assertEquals("Coordination", message.getSubject());
        assertEquals("SVC RQ", message.getRawText());
        assertEquals("{}", message.getParsedSummaryJson());
        assertEquals("nadinsim", feed.getSourceId());
        assertEquals("WEATHER", feed.getFeedType());
        assertEquals("hash", feed.getPayloadHash());
        assertEquals("METAR KJFK", feed.getRawPayload());
        assertTrue(feed.isAccepted());
        assertEquals("mission", history.getAggregateType());
        assertEquals("MISSION_CREATED", history.getEventType());
        assertEquals("planner", history.getActor());

        NotamEntity notam = new NotamEntity();
        notam.setReservationId(reservationId);
        notam.setNotamType("DOMESTIC");
        notam.setQCode("QWALW");
        notam.setAffectedLocation("KZNY");
        notam.setLowerAltitudeFeet(24000.0);
        notam.setUpperAltitudeFeet(26000.0);
        notam.setEffectiveStart(ZonedDateTime.parse("2026-05-20T00:00:00Z"));
        notam.setEffectiveEnd(ZonedDateTime.parse("2026-05-20T02:00:00Z"));
        notam.setRawText("!DCA DCA RWY CLSD");
        notam.setParsedJson("{}");

        ApreqEntity apreq = new ApreqEntity();
        apreq.setReservationId(reservationId);
        apreq.setStatus("REQUESTED");
        apreq.setRequestText("request");
        apreq.setResponseText("response");

        ApprovalEntity approval = new ApprovalEntity();
        approval.setReservationId(reservationId);
        approval.setApprovalType("SUPERVISOR");
        approval.setStatus("APPROVED");
        approval.setActor("supervisor");
        approval.setNote("ok");

        assertEquals("DOMESTIC", notam.getNotamType());
        assertEquals("QWALW", notam.getQCode());
        assertEquals("KZNY", notam.getAffectedLocation());
        assertEquals(24000.0, notam.getLowerAltitudeFeet());
        assertEquals(26000.0, notam.getUpperAltitudeFeet());
        assertEquals("!DCA DCA RWY CLSD", notam.getRawText());
        assertEquals("{}", notam.getParsedJson());
        assertEquals("REQUESTED", apreq.getStatus());
        assertEquals("request", apreq.getRequestText());
        assertEquals("response", apreq.getResponseText());
        assertEquals("SUPERVISOR", approval.getApprovalType());
        assertEquals("APPROVED", approval.getStatus());
        assertEquals("supervisor", approval.getActor());
        assertEquals("ok", approval.getNote());
    }

    @Test
    void altrvPersistenceTargetsRepresentLegacyMissionModelMappingWithoutLegacyPackages() {
        AltrvMessageEntity message = new AltrvMessageEntity();
        message.setActivityName("OCEANIC ALTRV");
        message.setMessageType("A_G");
        message.setParserStatus("PARSED");
        message.setTas("300 KTAS");
        message.setProjectOfficer("CARF");

        AltrvRouteGroupEntity group = new AltrvRouteGroupEntity();
        group.setAltrvMessage(message);
        group.setGroupName("D1");
        group.setRouteFamily("COMMON");
        group.setSequenceIndex(1);

        AltrvRouteEntity route = new AltrvRouteEntity();
        route.setRouteGroup(group);
        route.setRouteFamily("BRANCH");
        route.setRouteName("RTE1");
        route.setLowerAltitudeFeet(24000.0);
        route.setUpperAltitudeFeet(26000.0);

        AltrvRouteEventEntity event = new AltrvRouteEventEntity();
        event.setRoute(route);
        event.setEventType("JOIN_CMN_RTE");
        event.setCallsign("TEST01");
        event.setMetadataJson("{\"source\":\"hbm-parity\"}");

        AltrvAreaEntity area = new AltrvAreaEntity();
        area.setAltrvMessage(message);
        area.setAreaType("BNDD_BY");
        area.setWidthNm(20.0);
        area.setGeometryJson("{\"type\":\"Polygon\"}");

        message.getRouteGroups().add(group);
        group.getRoutes().add(route);
        route.getEvents().add(event);

        assertEquals("PARSED", message.getParserStatus());
        assertEquals("COMMON", message.getRouteGroups().get(0).getRouteFamily());
        assertEquals("BRANCH", group.getRoutes().get(0).getRouteFamily());
        assertEquals("JOIN_CMN_RTE", route.getEvents().get(0).getEventType());
        assertEquals("BNDD_BY", area.getAreaType());
    }

    @Test
    void productMigrationsIncludeLegacyCarfWorkflowAndAltrvGraphTargets() throws IOException {
        String v2 = new String(getClass().getResourceAsStream("/db/migration/V002__carf_legacy_parity_schema_targets.sql")
                .readAllBytes(), StandardCharsets.UTF_8);

        assertTrue(v2.contains("ops_altrv_message"));
        assertTrue(v2.contains("ops_altrv_route_group"));
        assertTrue(v2.contains("ops_altrv_route_event"));
        assertTrue(v2.contains("ops_altrv_area"));
        assertTrue(v2.contains("ops_reservation_note"));
        assertTrue(v2.contains("ops_reservation_image"));
        assertTrue(v2.contains("ops_message_recipient"));
        assertTrue(v2.contains("ops_group_separation"));
        assertTrue(v2.contains("ops_preferred_navaid"));
    }

    @Test
    void productGuidanceApisTurnWeatherAndPirepsIntoMissionBriefingArtifacts() {
        AirspaceProductService service = new AirspaceProductService(new ReservationWorkflowService(new InMemoryReservationWorkflowRepository()));
        ProductDtos.MissionRequest missionRequest = new ProductDtos.MissionRequest();
        missionRequest.setMissionNumber("WX-MISSION");
        missionRequest.setTitle("Weather mission");
        ProductDtos.MissionSummary mission = service.createMission(missionRequest);

        ProductDtos.ReservationRequest reservationRequest = new ProductDtos.ReservationRequest();
        reservationRequest.setActor("planner");
        reservationRequest.setRawText("A. WX01\nB. 1F22/I\nC. KZNY\nD. FL240B260 3000N15000W 0000 3000N14900W 0100\nE. WX01\nF. ETD WX01 200000 MAY 2026\nG. TAS 300");
        String reservationId = service.createReservation(mission.getId(), reservationRequest).getRecord().getId();

        ProductDtos.MessageRequest sigmet = new ProductDtos.MessageRequest();
        sigmet.setMissionId(mission.getId());
        sigmet.setReservationId(reservationId);
        sigmet.setFamily("SIGMET");
        sigmet.setDirection("INBOUND");
        sigmet.setSubject("SIGMET ECHO");
        sigmet.setRawText("SIGMET CONV SEV 3000N15000W 3000N14900W 3100N14900W FL240-260 CONF 90");
        service.sendMessage(sigmet);

        ProductDtos.MessageRequest pirep = new ProductDtos.MessageRequest();
        pirep.setMissionId(mission.getId());
        pirep.setReservationId(reservationId);
        pirep.setFamily("PIREP");
        pirep.setDirection("INBOUND");
        pirep.setSubject("Urgent turbulence");
        pirep.setRawText("UA /OV 3000N15000W/TM 0000/FL240/TP C17/TB SEV/RM TEST");
        service.sendMessage(pirep);

        ProductDtos.MessageRequest notam = new ProductDtos.MessageRequest();
        notam.setMissionId(mission.getId());
        notam.setReservationId(reservationId);
        notam.setFamily("FDC");
        notam.setDirection("INBOUND");
        notam.setSubject("FDC NOTAM route restriction");
        notam.setRawText("!FDC TEST NOTAM AIRSPACE CLSD WI AN AREA 3000N15000W-3000N14900W SFC-FL260");
        service.sendMessage(notam);

        ProductDtos.MissionWeatherVerdictSummary verdict = service.missionWeatherVerdict(mission.getId());
        List<ProductDtos.WeatherSourceSummary> changes = service.weatherChanges(mission.getId(), null, 10);
        List<ProductDtos.AffectedMissionSummary> affected = service.affectedMissions(null, 10);
        ProductDtos.RouteImpactSummary impact = service.routeImpact(mission.getId(), reservationId);
        ProductDtos.PirepRelevanceRequest relevanceRequest = new ProductDtos.PirepRelevanceRequest();
        relevanceRequest.setReservationId(reservationId);
        relevanceRequest.setLowerAltitudeFeet(22000.0);
        relevanceRequest.setUpperAltitudeFeet(28000.0);
        relevanceRequest.setAltitudeToleranceFeet(1500.0);
        relevanceRequest.setRecencyMinutes(60);
        relevanceRequest.setCorridorNauticalMiles(40.0);
        ProductDtos.PirepRelevanceResult relevant = service.relevantPireps(mission.getId(), relevanceRequest);
        ProductDtos.CoordinationDraftSummary draft = service.coordinationDraft(mission.getId(), null);
        ProductDtos.PilotBriefSummary brief = service.pilotBrief(mission.getId(), null);

        assertEquals(mission.getId(), verdict.getMissionId());
        assertTrue(verdict.getSourceCount() >= 2);
        assertTrue(verdict.getConfidence() > 0.0);
        assertTrue(changes.stream().anyMatch(change -> "SIGMET".equals(change.getFamily())));
        assertTrue(changes.stream().anyMatch(change -> "FDC".equals(change.getFamily())));
        assertTrue(verdict.getSources().stream().anyMatch(source -> "FDC".equals(source.getFamily())
                && source.getRationale().contains("NOTAM constraint")));
        assertTrue(affected.stream().anyMatch(item -> mission.getId().equals(item.getMissionId())));
        assertTrue(affected.stream().filter(item -> mission.getId().equals(item.getMissionId())).findFirst().orElseThrow().getAgeSeconds() >= 0);
        assertTrue(affected.stream().filter(item -> mission.getId().equals(item.getMissionId())).findFirst().orElseThrow().getGuidanceLatencySeconds() >= 0);
        java.util.Map<String, Double> metrics = service.metrics();
        assertTrue(metrics.get("product.weather.affectedMissions") >= 1.0);
        assertTrue(metrics.get("product.weather.guidanceTargetMet") >= 1.0);
        assertTrue(metrics.get("product.weather.averageGuidanceLatencySeconds") >= 0.0);
        assertTrue(metrics.get("product.weather.guidanceTargetRate") > 0.0);
        assertEquals(mission.getId(), impact.getMissionId());
        assertNotNull(impact.getAction());
        assertTrue(impact.getSourceRefs().stream().anyMatch(ref -> ref.startsWith("FDC:")));
        assertTrue(impact.getImpactedSegments().stream().anyMatch(segment -> segment.contains("NOTAM constraint")));
        assertTrue(impact.getRationale().contains("NOTAM constraint source"));
        assertTrue(relevant.getTotalPireps() >= 1);
        assertEquals(1500.0, relevant.getAltitudeToleranceFeet());
        assertEquals(60, relevant.getRecencyMinutes());
        assertEquals(40.0, relevant.getCorridorNauticalMiles());
        assertTrue(relevant.getAverageRelevanceScore() > 0.0);
        List<ProductDtos.WeatherSourceSummary> allPireps = new java.util.ArrayList<>(relevant.getRelevant());
        allPireps.addAll(relevant.getExcluded());
        assertTrue(allPireps.stream().anyMatch(source -> source.getRelevanceScore() > 0.0));
        assertTrue(allPireps.stream().anyMatch(source -> source.getAgeMinutes() >= 0));
        assertTrue(allPireps.stream().anyMatch(source -> source.getAgingCategory() != null));
        assertEquals("USNS", draft.getFamily());
        assertTrue(draft.getRawText().contains("WEATHER COORDINATION"));
        assertTrue(brief.getPrintableText().contains("AIRSPACE PILOT BRIEF"));
        assertTrue(brief.getPrintableText().contains("SOURCE DRIVERS"));
        assertTrue(brief.getSourceSummaryLines().stream().anyMatch(line -> line.contains("NOTAM")));
        assertNotNull(brief.getDecisionTraceSummary());
    }

    private AuthDtos.LoginRequest login(String username, String password) {
        AuthDtos.LoginRequest request = new AuthDtos.LoginRequest();
        request.setUsername(username);
        request.setPassword(password);
        return request;
    }
}
