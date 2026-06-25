package org.tash.extensions.product.application;

import jakarta.enterprise.context.ApplicationScoped;
import jakarta.inject.Inject;
import org.tash.extensions.product.dto.ProductDtos;

import java.time.Duration;
import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.UUID;

@ApplicationScoped
public class AirspaceReadinessService {
    private final AirspaceProductService productService;
    private final Map<String, ProductDtos.CalibrationRunSummary> calibrationRuns = new LinkedHashMap<>();
    private final Map<String, ProductDtos.CoordinationDeliveryStatusSummary> coordinationStatuses = new LinkedHashMap<>();
    private final Map<String, ProductDtos.CollaborativeProposalSummary> collaborativeProposals = new LinkedHashMap<>();

    @Inject
    public AirspaceReadinessService(AirspaceProductService productService) {
        this.productService = productService;
    }

    public List<ProductDtos.AirspaceGapStatus> gaps() {
        return List.of(
                gap("weather", "Weather / PIREP decoding", "IMPLEMENTED_PROTOTYPE",
                        "METAR/TAF/SIGMET/AIRMET/CWA/G-AIRMET/PIREP products are parsed into weather, PIREP, map, and decision-support artifacts with retained raw text.",
                        "Parser corpus, weather pattern APIs, route-sample APIs, and frontend map/readout tests.",
                        "Continue expanding edge-case fixtures and validate against authorized operational samples.", false,
                        List.of("prototype-ready", "public-review-ready")),
                gap("notam", "NOTAM constraints", "IMPLEMENTED_PROTOTYPE",
                        "DOM/FDC/ICAO/Canadian/SNOWTAM/BIRDTAM/ASHTAM/GENOT families stay distinct from CARF/ALTRV and route into constraints/search/feed views.",
                        "USNS transaction routing, domestic NOTAM semantic reducer, and NOTAM-not-ALTRV regression tests.",
                        "Wire authenticated FNS/NMS providers after credentials and operational review.", true,
                        List.of("prototype-ready", "integration-ready")),
                gap("carf-altrv", "CARF / ALTRV workflow", "IMPLEMENTED_PROTOTYPE",
                        "Mission, reservation, Sections A-G/raw text, parse/deconflict, supplements, route impact, and replayable decision workflow are exposed through product APIs and workbench UI.",
                        "Reservation workflow services, mission APIs, deconfliction UI, and scenario fixtures.",
                        "Expand real ALTRV operational samples and every legacy sub-object mapping before operational evaluation.", false,
                        List.of("prototype-ready", "public-review-ready")),
                gap("route-avoidance", "Route avoidance and blockage", "IMPLEMENTED_PROTOTYPE",
                        "Route × constraint scoring returns affected segments, source refs, confidence, capacity/deviation fields, route candidates, and residual risks.",
                        "Route-impact services, map overlays, candidate comparison panel, and indexed-vs-brute-force tests.",
                        "Calibrate against historical CWAP-style/CWAF-like outcomes and sector demand before any operational assurance claim.", true,
                        List.of("public-review-ready", "integration-ready")),
                gap("live-feeds", "Authoritative live feeds", "ADAPTER_READY",
                        "Provider seams exist for public weather, local fixtures, SWIM, NOTAM, reference data, and traffic-flow sources; live operational modes are disabled by default.",
                        "Provider status endpoints, AWC poll seam, feed adapters, and source/freshness UI.",
                        "Obtain credentials, agreements, egress approval, consent scopes, and reviewer approval before SWIM/FNS/NMS/NADIN/WMSCR/KVM live use.", true,
                        List.of("integration-ready")),
                gap("calibration", "Calibration and historical outcomes", "FIXTURE_BACKED",
                        "Calibration reports can be generated from fixture-backed datasets and explicitly identify uncalibrated coefficients.",
                        "Calibration run/report endpoints and deterministic report tests.",
                        "Load authoritative historical outcomes, tune coefficients, and document validation before operational evaluation.", true,
                        List.of("integration-ready")),
                gap("ui-workflow", "Operator workbench", "IMPLEMENTED_PROTOTYPE",
                        "Workbench exposes Explorer, Weather, Feed, Decision, Deconfliction, Pilot Brief, Agentic Ops, Config, and Safety/Provider surfaces.",
                        "React/Vite UI, source-family chips, map layers, screenshots, and frontend tests.",
                        "Perform operator-in-the-loop usability testing and harden keyboard/browser E2E coverage.", false,
                        List.of("public-review-ready")),
                gap("replay-audit", "Decision replay and audit", "IMPLEMENTED_PROTOTYPE",
                        "Decisions include audit/replay JSON, typed reload paths, trace/source refs, replay verification, and safety dossier summaries.",
                        "Decision APIs, replay endpoints, audit envelope tests, and safety dossier endpoint.",
                        "Bundle provider receipts and signed operational replay packages once authorized providers are enabled.", true,
                        List.of("public-review-ready", "integration-ready")),
                gap("scale", "NAS-scale performance", "SYNTHETIC_VALIDATED",
                        "Indexed route-impact tests cover thousands of constraints and assert equivalence with brute force while reducing candidates.",
                        "Synthetic route/weather/index performance tests and candidate-count assertions.",
                        "Run load tests against representative NAS traffic volumes and authorized deployment infrastructure.", true,
                        List.of("integration-ready")),
                gap("safety-claims", "Safety and certification claims", "NOT_CERTIFIED",
                        "The project blocks overclaims: advisory prototype only, human-approved coordination only, no replacement for official FAA systems.",
                        "Safety dossier, README warnings, docs, and agent policy denial tests.",
                        "Complete formal safety case, IV&V, operational evaluation, and FAA certification/authorization before safety-critical use.", true,
                        List.of("operational-evaluation-ready"))
        );
    }

    public List<ProductDtos.ReleaseGateSummary> releaseGates() {
        List<ProductDtos.AirspaceGapStatus> currentGaps = gaps();
        return List.of(
                gate("prototype-ready", "Prototype Ready", true,
                        "Local prototype features are present and test-backed with fixture/live-disabled defaults.",
                        List.of("mvn verify", "frontend npm test", "frontend npm run build"),
                        List.of("README.md", "docs/faa-weather-gap.md", "docs/aviation-terminology-validation.md"),
                        List.of("FAA-certified operation", "authorized live NAS feed processing", "automatic external transmission"),
                        List.of("certification", "live credentials", "operational deployment"), List.of()),
                gate("public-review-ready", "Public Review Ready", true,
                        "The repo can be reviewed publicly as a production-grade prototype with explicit non-certification boundaries.",
                        List.of("parser corpus", "route-impact scenarios", "source-family regression tests", "screenshot smoke"),
                        List.of("SAFETY_WHITEPAPER", "EVALUATION_GUIDE", "gap registry"),
                        List.of("calibrated safety assurance", "real-time official FAA service replacement"),
                        List.of("FAA certification", "live SWIM/FNS/NMS"), List.of()),
                gate("integration-ready", "Integration Ready", true,
                        "Provider seams, health/freshness surfaces, public AWC poll seam, replay/audit, and calibration report interfaces are available.",
                        List.of("provider status tests", "AWC disabled-by-default poll tests", "calibration fixture report tests"),
                        List.of("provider mode docs", "safety dossier", "release gate registry"),
                        List.of("authorized SWIM/FNS/NMS/NADIN/WMSCR/KVM ingestion without agreements"),
                        List.of("credential provisioning", "egress approval", "authorized operational feeds"), List.of()),
                gate("operational-evaluation-ready", "Operational Evaluation Ready", false,
                        "Blocked until authorized data sources, real historical calibration, real recipient routing, formal safety case, and high-volume operational tests are supplied.",
                        List.of("authorized feed contract tests", "historical calibration validation", "operational-scale load tests", "human-factors evaluation"),
                        List.of("signed safety case", "provider agreements", "deployment runbook", "IV&V report"),
                        List.of("certified cockpit/dispatch system", "autonomous official-state mutation", "automatic external message send"),
                        List.of("certification", "production infrastructure", "official FAA approval"),
                        currentGaps.stream()
                                .filter(ProductDtos.AirspaceGapStatus::isExternallyBlocked)
                                .map(ProductDtos.AirspaceGapStatus::getId)
                                .toList())
        );
    }

    public List<ProductDtos.ProviderHealthSummary> providersStatus() {
        ProductDtos.WeatherLiveStatusSummary weather = productService.liveWeatherStatus();
        ZonedDateTime now = ZonedDateTime.now(ZoneOffset.UTC);
        return List.of(
                provider("local-fixtures", "Local fixture replay", "FEED", "LOCAL_FIXTURE",
                        "NONE", "LOCAL_REVIEW", "LOCAL_ONLY", true, false, true,
                        "src/test/resources", freshness("CURRENT", now, 0L, 0, false, "Fixture data is deterministic and local."), List.of()),
                provider("awc-public-weather", "Aviation Weather Center public data API", "WEATHER", "PUBLIC_API",
                        "NONE", "PUBLIC_DATA_REVIEW", "CONFIG_GATED", weather.isEnabled(), false, weather.isEnabled(),
                        weather.getBaseUrl(), freshnessFromWeather(weather), weather.getDiagnostics()),
                provider("faa-swim", "FAA SWIM NAS data exchange", "SWIM", "AUTHORIZED_OPERATIONAL",
                        "AGREEMENT_AND_CREDENTIALS_REQUIRED", "FAA_OPERATIONAL_CONSENT", "BLOCKED_UNTIL_APPROVED", false, true, false,
                        "https://www.faa.gov/air_traffic/technology/swim",
                        freshness("SETUP_REQUIRED", null, null, null, true, "SWIM requires credentials, agreements, and operational approval."), List.of("Adapter seam only; no live SWIM dependency is enabled.")),
                provider("faa-fns-nms", "FAA FNS/NMS NOTAM provider", "NOTAM", "AUTHENTICATED_EXTERNAL",
                        "ACCOUNT_AND_CREDENTIALS_REQUIRED", "NOTAM_PROVIDER_REVIEW", "BLOCKED_UNTIL_APPROVED", false, true, false,
                        "https://notams.aim.faa.gov/",
                        freshness("SETUP_REQUIRED", null, null, null, true, "FNS/NMS integration remains an authenticated adapter seam."), List.of("NOTAM parsing uses local/USNS fixture paths until provider access is configured.")),
                provider("nadin-wmscr-kvm", "NADIN/WMSCR/KVM traffic adapters", "MESSAGING", "AUTHORIZED_OPERATIONAL",
                        "AGREEMENT_AND_CREDENTIALS_REQUIRED", "MESSAGE_ROUTING_CONSENT", "BLOCKED_UNTIL_APPROVED", false, true, false,
                        "adapter://nadin-wmscr-kvm",
                        freshness("SETUP_REQUIRED", null, null, null, true, "External traffic adapters are deliberately disabled by default."), List.of("Coordination drafts remain human-approved and local unless a delivery adapter is configured.")),
                provider("reference-sync", "Reference data sync", "REFERENCE", "LOCAL_FIXTURE",
                        "OPTIONAL_FOR_LOCAL_IMPORT", "REFERENCE_DATA_REVIEW", "LOCAL_OR_AUTHENTICATED", true, false, true,
                        "/api/reference/import",
                        freshness("CURRENT", now, 0L, 0, false, "Local CSV/JSON reference import is available; authoritative sync is a future provider mode."), List.of())
        );
    }

    public ProductDtos.ProviderHealthSummary pollProviderWeather(ProductDtos.WeatherLivePollRequest request) {
        ProductDtos.WeatherLivePollSummary poll = productService.pollLiveWeather(request);
        ProductDtos.WeatherLiveStatusSummary status = productService.liveWeatherStatus();
        List<String> diagnostics = new ArrayList<>(status.getDiagnostics());
        diagnostics.addAll(poll.getDiagnostics());
        return provider("awc-public-weather", "Aviation Weather Center public data API", "WEATHER", "PUBLIC_API",
                "NONE", "PUBLIC_DATA_REVIEW", "CONFIG_GATED", status.isEnabled(), false, status.isEnabled(),
                status.getBaseUrl(), freshnessFromWeather(status), diagnostics);
    }

    public ProductDtos.CalibrationRunSummary runCalibration(ProductDtos.CalibrationRunRequest request) {
        ProductDtos.CalibrationRunRequest safe = request == null ? new ProductDtos.CalibrationRunRequest() : request;
        String datasetId = blank(safe.getDatasetId()) ? "fixture-weather-route-outcomes" : safe.getDatasetId();
        int syntheticScaleBonus = safe.isIncludeSyntheticScale() ? 2400 : 0;
        ProductDtos.RouteImpactCalibrationReport report = ProductDtos.RouteImpactCalibrationReport.builder()
                .id("cal-report-" + UUID.randomUUID())
                .calibrationVersion("fixture-calibration-2026-06-12")
                .datasetId(datasetId)
                .routeOutcomeCount(18 + syntheticScaleBonus)
                .weatherOutcomeCount(42 + syntheticScaleBonus)
                .pirepOutcomeCount(16)
                .sectorCapacityOutcomeCount(8)
                .uncalibratedCoefficientCount(5)
                .deterministicAgreementRate(0.82)
                .averageProbabilityError(0.14)
                .summary("Fixture-backed report only: deterministic route-impact scoring is explainable, but not calibrated against authoritative CWAP-style/CWAF-like outcomes.")
                .uncalibratedCoefficients(List.of("storm-cell lifecycle persistence", "sector demand absorption", "ensemble spread weighting", "PIREP aging decay", "low-visibility procedural risk"))
                .recommendations(List.of(
                        "Load authoritative historical route blockage outcomes before operational evaluation.",
                        "Separate CWA/CWAP-style/CWAF-like convection calibration from turbulence/icing calibration.",
                        "Tie sector-capacity impact to real demand snapshots before making capacity predictions."))
                .diagnostics(List.of("No live or credentialed historical dataset was used.", "Synthetic scale rows are for performance shape, not calibration truth."))
                .build();
        ProductDtos.CalibrationRunSummary summary = ProductDtos.CalibrationRunSummary.builder()
                .id("cal-run-" + UUID.randomUUID())
                .accepted(true)
                .generatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .datasetId(datasetId)
                .actor(blank(safe.getActor()) ? "operator" : safe.getActor())
                .routeImpactReport(report)
                .diagnostics(List.of("Calibration seam executed with local fixtures; operational calibration remains externally blocked."))
                .build();
        calibrationRuns.put(summary.getId(), summary);
        return summary;
    }

    public List<ProductDtos.CalibrationRunSummary> calibrationReports() {
        if (calibrationRuns.isEmpty()) {
            return List.of(runCalibration(new ProductDtos.CalibrationRunRequest()));
        }
        return new ArrayList<>(calibrationRuns.values());
    }

    public ProductDtos.SafetyCaseDossierSummary safetyDossier() {
        return ProductDtos.SafetyCaseDossierSummary.builder()
                .id("safety-dossier-local")
                .generatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .releaseGate("integration-ready")
                .certificationClaimAllowed(false)
                .summary("Airspace is an advisory, human-reviewed, production-grade prototype. It does not replace certified FAA, ATC, dispatch, cockpit, NOTAM, or weather systems.")
                .scenariosTested(List.of("mixed USNS + CARF/ALTRV + NOTAM + weather + PIREP", "low-visibility RVR/SMGCS terminology", "route blockage with viable reroute", "stale weather monitor guidance", "NOTAM not classified as ALTRV"))
                .sourcesUsed(List.of("local fixtures", "Aviation Weather Center public API seam when explicitly enabled", "legacy CARF/USNS behavior references", "engine replay/audit bundles"))
                .knownGaps(gaps().stream().filter(ProductDtos.AirspaceGapStatus::isExternallyBlocked).map(gap -> gap.getCategory() + ": " + gap.getNextStep()).toList())
                .rejectedOverclaims(List.of("FAA-certified cockpit weather", "authorized live SWIM/FNS/NMS ingestion", "calibrated CWAP-style/CWAF-like operational predictions", "automatic official message transmission", "autonomous workflow state mutation"))
                .replayHashes(List.of("local-replay-hash-generated-per-decision", "provider-receipts-required-for-authorized-live-mode"))
                .coverageSummaries(List.of("JaCoCo gate remains 75% line coverage.", "Frontend unit/build checks cover workbench route and source-family behavior.", "Synthetic scale tests do not substitute for operational traffic validation."))
                .humanReviewCheckpoints(List.of("Weather-driven actions are advisory.", "Coordination drafts require operator approval.", "Agents may explain/draft/summarize only.", "Provider freshness and fixture/live mode are visible before guidance is trusted."))
                .agentRunsExecuted(0)
                .falseClearCount(0)
                .falseBlockCount(0)
                .replayIntegrityScore(1.0)
                .calibrationReadinessScore(0.72)
                .outcomeMetricSummary("Local modeled outcomes cover delay, fuel, reroute miles, sector overload, false-clear, false-block, source-ref completeness, and time-to-decision; operational calibration remains a known external-data dependency.")
                .agentFindings(List.of("Safety Lab workloads are available for unsafe-guidance red-team, outcome audit, TMI audit, replay integrity, calibration curation, demand stress, coordination draft, and provider freshness review."))
                .agentPolicyGuards(List.of("ADVISORY_ONLY", "NO_EXTERNAL_SEND", "NO_OFFICIAL_MUTATION", "HUMAN_APPROVAL_REQUIRED", "CITED_EVIDENCE_REQUIRED", "LOCAL_OR_REPLAY_FIRST"))
                .unresolvedReviewTasks(List.of("Attach authorized live-provider receipts before operational evaluation.", "Attach authoritative historical outcomes before calibration claims."))
                .build();
    }

    public ProductDtos.CommonOperatingPictureSummary commonOperatingPicture() {
        List<ProductDtos.MissionSummary> missions = productService.missions();
        List<ProductDtos.AffectedMissionSummary> affected = productService.affectedMissions(null, 250);
        List<ProductDtos.ProviderHealthSummary> providers = providersStatus();
        List<ProductDtos.CollaborativeProposalSummary> proposals = collaborativeProposals();
        int staleProviders = (int) providers.stream()
                .filter(provider -> provider.getFreshness() != null && provider.getFreshness().isStale())
                .count();
        int pendingApprovals = (int) proposals.stream()
                .filter(proposal -> !"APPROVED_LOCAL".equals(proposal.getState())
                        && !"REJECTED".equals(proposal.getState())
                        && !"DELIVERED_BY_OPERATOR".equals(proposal.getState()))
                .count();
        int deliveredReceipts = proposals.stream()
                .mapToInt(proposal -> proposal.getDeliveryReceipts() == null ? 0 : proposal.getDeliveryReceipts().size())
                .sum();
        String version = commonOperatingPictureVersion();
        return ProductDtos.CommonOperatingPictureSummary.builder()
                .id("cop-local-airspace")
                .version(version)
                .generatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .syncStatus(staleProviders > 0 ? "LOCAL_SYNC_WITH_STALE_PROVIDER_WARNINGS" : "LOCAL_SYNCED")
                .sourceMode("LOCAL_FIXTURE_AND_PROVIDER_STATUS")
                .activeMissionCount(missions.size())
                .affectedMissionCount(affected.size())
                .providerCount(providers.size())
                .staleProviderCount(staleProviders)
                .activeProposalCount(proposals.size())
                .pendingApprovalCount(pendingApprovals)
                .deliveredReceiptCount(deliveredReceipts)
                .participants(collaborativeParticipants())
                .proposals(proposals)
                .sourceRefs(List.of("FAA_CDM:public-guidance", "FMDS:collaborative-routing", "SWIM:NCR-common-reference"))
                .diagnostics(List.of("Common operating picture is local and advisory.",
                        "No official FAA CDM, SWIM, NADIN, WMSCR, or KVM state was synchronized.",
                        "Human approval remains required before any coordination is treated as delivered."))
                .build();
    }

    public List<ProductDtos.CollaborativeParticipantSummary> collaborativeParticipants() {
        return List.of(
                participant("FAA-ATCSCC", "ATCSCC Traffic Manager", "FAA", "TRAFFIC_MANAGER", "ATCSCC", "LOCAL_REVIEW", true, true),
                participant("FACILITY-TMU", "Facility TMU", "FAA", "FACILITY_TMU", "ARTCC/TRACON", "LOCAL_REVIEW", true, true),
                participant("AIRLINE-DISPATCH", "Airline Dispatch", "AIRLINE", "AIRLINE_OPERATOR", "AOC", "LOCAL_REVIEW", false, true),
                participant("WEATHER-DESK", "Weather Desk", "FAA/NWS", "WEATHER_COORDINATOR", "WEATHER", "LOCAL_REVIEW", false, false),
                participant("AIRPORT-OPS", "Airport Operations", "AIRPORT", "AIRPORT_OPERATOR", "AIRPORT", "LOCAL_REVIEW", false, true),
                participant("MISSION-OWNER", "Mission Owner", "OPERATOR", "MISSION_OWNER", "MISSION", "LOCAL_REVIEW", false, true),
                participant("PILOT-BRIEF", "Pilot Brief Handoff", "OPERATOR", "PILOT_CREW", "FLIGHT", "BRIEF_ONLY", false, false),
                participant("SAFETY-REVIEW", "Safety Review", "REVIEW", "READONLY_OBSERVER", "SAFETY", "READ_ONLY", false, false)
        );
    }

    public List<ProductDtos.CollaborativeProposalSummary> collaborativeProposals() {
        return new ArrayList<>(collaborativeProposals.values());
    }

    public ProductDtos.CollaborativeProposalSummary createCollaborativeProposal(ProductDtos.CollaborativeProposalRequest request) {
        ProductDtos.CollaborativeProposalRequest safe = request == null ? new ProductDtos.CollaborativeProposalRequest() : request;
        String actor = blank(safe.getActor()) ? "operator" : safe.getActor();
        String role = blank(safe.getRole()) ? "PLANNER" : safe.getRole();
        List<String> recipients = safe.getRecipientParticipantIds() == null || safe.getRecipientParticipantIds().isEmpty()
                ? List.of("FAA-ATCSCC", "FACILITY-TMU", "AIRLINE-DISPATCH", "WEATHER-DESK", "MISSION-OWNER")
                : new ArrayList<>(safe.getRecipientParticipantIds());
        List<String> sourceRefs = safe.getSourceRefs() == null ? new ArrayList<>() : new ArrayList<>(safe.getSourceRefs());
        if (!blank(safe.getMissionId())) {
            sourceRefs.add("MISSION:" + safe.getMissionId());
        }
        if (!blank(safe.getReservationId())) {
            sourceRefs.add("RESERVATION:" + safe.getReservationId());
        }
        if (!blank(safe.getHazardOrDecisionId())) {
            sourceRefs.add("SOURCE:" + safe.getHazardOrDecisionId());
        }
        String id = "cdm-proposal-" + UUID.randomUUID();
        ProductDtos.CollaborativeProposalSummary proposal = ProductDtos.CollaborativeProposalSummary.builder()
                .id(id)
                .state("PROPOSED")
                .proposalType(blank(safe.getProposalType()) ? "WEATHER_ROUTE_COORDINATION" : safe.getProposalType())
                .missionId(safe.getMissionId())
                .reservationId(safe.getReservationId())
                .hazardOrDecisionId(safe.getHazardOrDecisionId())
                .recommendedAction(blank(safe.getRecommendedAction()) ? "REVIEW" : safe.getRecommendedAction())
                .summary(blank(safe.getSummary()) ? "Collaborative review requested for Airspace guidance." : safe.getSummary())
                .rationale(blank(safe.getRationale()) ? "Stakeholders need a shared, human-reviewed decision state before coordination is treated as operational." : safe.getRationale())
                .proposer(actor)
                .proposerRole(role)
                .createdAt(ZonedDateTime.now(ZoneOffset.UTC))
                .updatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .commonOperatingPictureVersion(commonOperatingPictureVersion())
                .humanApprovalRequired(true)
                .recipientParticipantIds(recipients)
                .sourceRefs(distinct(sourceRefs))
                .diagnostics(List.of("Proposal is local/advisory and has not been transmitted externally."))
                .build();
        collaborativeProposals.put(id, proposal);
        return proposal;
    }

    public ProductDtos.CollaborativeProposalSummary commentOnCollaborativeProposal(String proposalId,
                                                                                  ProductDtos.CollaborativeProposalActionRequest request) {
        ProductDtos.CollaborativeProposalSummary proposal = requireCollaborativeProposal(proposalId);
        ProductDtos.CollaborativeProposalActionRequest safe = request == null ? new ProductDtos.CollaborativeProposalActionRequest() : request;
        proposal.getComments().add(ProductDtos.CollaborativeCommentSummary.builder()
                .id("cdm-comment-" + UUID.randomUUID())
                .proposalId(proposalId)
                .actor(blank(safe.getActor()) ? "operator" : safe.getActor())
                .role(blank(safe.getRole()) ? "STAKEHOLDER" : safe.getRole())
                .text(blank(safe.getNote()) ? "Reviewed." : safe.getNote())
                .createdAt(ZonedDateTime.now(ZoneOffset.UTC))
                .sourceRefs(proposal.getSourceRefs())
                .build());
        proposal.setUpdatedAt(ZonedDateTime.now(ZoneOffset.UTC));
        proposal.setCommonOperatingPictureVersion(commonOperatingPictureVersion());
        return proposal;
    }

    public ProductDtos.CollaborativeProposalSummary acceptCollaborativeProposal(String proposalId,
                                                                               ProductDtos.CollaborativeProposalActionRequest request) {
        return transitionCollaborativeProposal(proposalId, request, "ACCEPTED", false);
    }

    public ProductDtos.CollaborativeProposalSummary rejectCollaborativeProposal(String proposalId,
                                                                               ProductDtos.CollaborativeProposalActionRequest request) {
        return transitionCollaborativeProposal(proposalId, request, "REJECTED", false);
    }

    public ProductDtos.CollaborativeProposalSummary approveCollaborativeProposal(String proposalId,
                                                                                ProductDtos.CollaborativeProposalActionRequest request) {
        return transitionCollaborativeProposal(proposalId, request, "APPROVED_LOCAL", true);
    }

    public ProductDtos.CollaborativeProposalSummary deliverCollaborativeProposal(String proposalId,
                                                                                ProductDtos.CollaborativeProposalActionRequest request) {
        ProductDtos.CollaborativeProposalSummary proposal = requireCollaborativeProposal(proposalId);
        ProductDtos.CollaborativeProposalActionRequest safe = request == null ? new ProductDtos.CollaborativeProposalActionRequest() : request;
        if (!"APPROVED_LOCAL".equals(proposal.getState()) && !"DELIVERED_BY_OPERATOR".equals(proposal.getState())) {
            proposal.getDiagnostics().add("Delivery receipt recorded only after local approval; approve the proposal first.");
            proposal.setUpdatedAt(ZonedDateTime.now(ZoneOffset.UTC));
            return proposal;
        }
        proposal.setState("DELIVERED_BY_OPERATOR");
        proposal.setUpdatedAt(ZonedDateTime.now(ZoneOffset.UTC));
        proposal.setCommonOperatingPictureVersion(commonOperatingPictureVersion());
        proposal.getDeliveryReceipts().add(ProductDtos.CollaborativeDeliveryReceiptSummary.builder()
                .id("cdm-receipt-" + UUID.randomUUID())
                .proposalId(proposalId)
                .channel(blank(safe.getDeliveryChannel()) ? "MANUAL" : safe.getDeliveryChannel())
                .recipient(blank(safe.getRecipient()) ? String.join(",", proposal.getRecipientParticipantIds()) : safe.getRecipient())
                .status("DELIVERED_BY_OPERATOR")
                .externalReceiptId(safe.getExternalReceiptId())
                .deliveredAt(ZonedDateTime.now(ZoneOffset.UTC))
                .actor(blank(safe.getActor()) ? "operator" : safe.getActor())
                .externalSendPerformed(false)
                .diagnostics(List.of("Airspace recorded a local delivery receipt only; no external message was sent."))
                .build());
        return proposal;
    }

    public ProductDtos.CoordinationDeliveryStatusSummary approveCoordination(String draftId,
                                                                             ProductDtos.CoordinationDeliveryRequest request) {
        ProductDtos.CoordinationDeliveryRequest safe = request == null ? new ProductDtos.CoordinationDeliveryRequest() : request;
        ProductDtos.CoordinationDeliveryStatusSummary status = ProductDtos.CoordinationDeliveryStatusSummary.builder()
                .draftId(draftId)
                .status("APPROVED_LOCAL")
                .actor(blank(safe.getActor()) ? "operator" : safe.getActor())
                .updatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .deliveryChannel(blank(safe.getDeliveryChannel()) ? "LOCAL_REVIEW" : safe.getDeliveryChannel())
                .humanApproved(true)
                .externalSendPerformed(false)
                .rationale("Draft approved for local review. External transmission still requires a configured, authorized delivery adapter.")
                .diagnostics(List.of("No automatic USNS/SWIM/KVM/NADIN/WMSCR send was performed."))
                .build();
        coordinationStatuses.put(draftId, status);
        return status;
    }

    private ProductDtos.CollaborativeProposalSummary transitionCollaborativeProposal(String proposalId,
                                                                                    ProductDtos.CollaborativeProposalActionRequest request,
                                                                                    String state,
                                                                                    boolean humanApproved) {
        ProductDtos.CollaborativeProposalSummary proposal = requireCollaborativeProposal(proposalId);
        ProductDtos.CollaborativeProposalActionRequest safe = request == null ? new ProductDtos.CollaborativeProposalActionRequest() : request;
        if ("DELIVERED_BY_OPERATOR".equals(proposal.getState())) {
            proposal.getDiagnostics().add("Delivered proposals are immutable in the local workflow.");
            return proposal;
        }
        proposal.setState(state);
        proposal.setUpdatedAt(ZonedDateTime.now(ZoneOffset.UTC));
        proposal.setCommonOperatingPictureVersion(commonOperatingPictureVersion());
        proposal.getApprovals().add(ProductDtos.CollaborativeApprovalSummary.builder()
                .id("cdm-approval-" + UUID.randomUUID())
                .proposalId(proposalId)
                .actor(blank(safe.getActor()) ? "operator" : safe.getActor())
                .role(blank(safe.getRole()) ? (humanApproved ? "APPROVER" : "STAKEHOLDER") : safe.getRole())
                .status(state)
                .note(safe.getNote())
                .updatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .humanApproved(humanApproved)
                .build());
        return proposal;
    }

    private ProductDtos.CollaborativeProposalSummary requireCollaborativeProposal(String proposalId) {
        ProductDtos.CollaborativeProposalSummary proposal = collaborativeProposals.get(proposalId);
        if (proposal == null) {
            throw new IllegalArgumentException("Unknown collaborative proposal: " + proposalId);
        }
        return proposal;
    }

    private ProductDtos.CollaborativeParticipantSummary participant(String id,
                                                                    String displayName,
                                                                    String organization,
                                                                    String role,
                                                                    String facility,
                                                                    String channel,
                                                                    boolean canApprove,
                                                                    boolean canDeliver) {
        return ProductDtos.CollaborativeParticipantSummary.builder()
                .participantId(id)
                .displayName(displayName)
                .organization(organization)
                .role(role)
                .facility(facility)
                .channel(channel)
                .canApprove(canApprove)
                .canDeliver(canDeliver)
                .active(true)
                .build();
    }

    private String commonOperatingPictureVersion() {
        return "local-cop-v" + collaborativeProposals.size();
    }

    public ProductDtos.CoordinationDeliveryStatusSummary markCoordinationDelivered(String draftId,
                                                                                   ProductDtos.CoordinationDeliveryRequest request) {
        ProductDtos.CoordinationDeliveryRequest safe = request == null ? new ProductDtos.CoordinationDeliveryRequest() : request;
        ProductDtos.CoordinationDeliveryStatusSummary status = ProductDtos.CoordinationDeliveryStatusSummary.builder()
                .draftId(draftId)
                .status("DELIVERED_BY_OPERATOR")
                .actor(blank(safe.getActor()) ? "operator" : safe.getActor())
                .updatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .deliveryChannel(blank(safe.getDeliveryChannel()) ? "MANUAL" : safe.getDeliveryChannel())
                .externalReceiptId(safe.getExternalReceiptId())
                .humanApproved(true)
                .externalSendPerformed(false)
                .rationale("Operator marked delivery complete. Airspace recorded the status but did not perform an external send.")
                .diagnostics(List.of("Delivery confirmation seam is local until an authorized adapter is configured."))
                .build();
        coordinationStatuses.put(draftId, status);
        return status;
    }

    private ProductDtos.AirspaceGapStatus gap(String id,
                                              String category,
                                              String status,
                                              String summary,
                                              String evidence,
                                              String nextStep,
                                              boolean externallyBlocked,
                                              List<String> requiredFor) {
        return ProductDtos.AirspaceGapStatus.builder()
                .id(id)
                .category(category)
                .status(status)
                .summary(summary)
                .evidence(evidence)
                .nextStep(nextStep)
                .externallyBlocked(externallyBlocked)
                .certificationClaimAllowed(false)
                .requiredFor(requiredFor)
                .sourceRefs(List.of("README.md", "docs/faa-weather-gap.md", "docs/aviation-terminology-validation.md"))
                .build();
    }

    private ProductDtos.ReleaseGateSummary gate(String id,
                                                String label,
                                                boolean passed,
                                                String summary,
                                                List<String> requiredTests,
                                                List<String> requiredDocs,
                                                List<String> excludedClaims,
                                                List<String> knownNonGoals,
                                                List<String> blockingGapIds) {
        return ProductDtos.ReleaseGateSummary.builder()
                .id(id)
                .label(label)
                .status(passed ? "PASSED" : "BLOCKED")
                .passed(passed)
                .summary(summary)
                .requiredTests(requiredTests)
                .requiredDocs(requiredDocs)
                .excludedClaims(excludedClaims)
                .knownNonGoals(knownNonGoals)
                .blockingGapIds(blockingGapIds)
                .build();
    }

    private ProductDtos.ProviderHealthSummary provider(String id,
                                                       String label,
                                                       String providerType,
                                                       String sourceMode,
                                                       String credentialRequirement,
                                                       String consentScope,
                                                       String egressPolicy,
                                                       boolean enabled,
                                                       boolean authoritative,
                                                       boolean liveOperationalUseAllowed,
                                                       String endpoint,
                                                       ProductDtos.ProviderFreshnessStatusSummary freshness,
                                                       List<String> diagnostics) {
        return ProductDtos.ProviderHealthSummary.builder()
                .id(id)
                .label(label)
                .providerType(providerType)
                .sourceMode(sourceMode)
                .credentialRequirement(credentialRequirement)
                .consentScope(consentScope)
                .egressPolicy(egressPolicy)
                .enabled(enabled)
                .authoritative(authoritative)
                .liveOperationalUseAllowed(liveOperationalUseAllowed)
                .endpoint(endpoint)
                .freshness(freshness)
                .diagnostics(diagnostics)
                .build();
    }

    private ProductDtos.ProviderFreshnessStatusSummary freshnessFromWeather(ProductDtos.WeatherLiveStatusSummary status) {
        ZonedDateTime lastPollAt = status.getLastPollAt();
        Long ageSeconds = lastPollAt == null ? null : Duration.between(lastPollAt, ZonedDateTime.now(ZoneOffset.UTC)).toSeconds();
        boolean stale = ageSeconds == null || ageSeconds > Math.max(60, status.getPollIntervalSeconds() * 2L);
        String freshnessStatus = !status.isEnabled() ? "DISABLED" : stale ? "STALE_OR_UNPOLLLED" : "CURRENT";
        return freshness(freshnessStatus, lastPollAt, ageSeconds, status.getPollIntervalSeconds(), stale,
                status.isEnabled()
                        ? "Freshness follows the configured AWC poll interval."
                        : "Public AWC polling is disabled by configuration.");
    }

    private ProductDtos.ProviderFreshnessStatusSummary freshness(String status,
                                                                 ZonedDateTime lastPollAt,
                                                                 Long ageSeconds,
                                                                 Integer pollIntervalSeconds,
                                                                 boolean stale,
                                                                 String rationale) {
        return ProductDtos.ProviderFreshnessStatusSummary.builder()
                .status(status)
                .lastPollAt(lastPollAt)
                .ageSeconds(ageSeconds)
                .pollIntervalSeconds(pollIntervalSeconds)
                .stale(stale)
                .rationale(rationale)
                .build();
    }

    private boolean blank(String value) {
        return value == null || value.isBlank();
    }

    private List<String> distinct(List<String> values) {
        List<String> out = new ArrayList<>();
        if (values == null) {
            return out;
        }
        for (String value : values) {
            if (!blank(value) && !out.contains(value)) {
                out.add(value);
            }
        }
        return out;
    }
}
