package org.tash.extensions.simulation;

import jakarta.enterprise.context.ApplicationScoped;
import jakarta.inject.Inject;
import org.tash.extensions.product.application.AirspaceProductService;
import org.tash.extensions.product.dto.ProductDtos;
import org.tash.extensions.visualization.AirspaceFeature;
import org.tash.extensions.visualization.AirspaceFeatureCollection;
import org.tash.extensions.visualization.AirspaceGeometry;
import org.tash.extensions.workflow.ReservationWorkflowResult;

import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Optional;
import java.util.UUID;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

@ApplicationScoped
public class OperationalSimulationService {
    private final AirspaceProductService productService;
    private final Map<String, SimulationRunResult> runs = new LinkedHashMap<>();
    private final Map<String, SimulationCampaignReport> campaigns = new LinkedHashMap<>();
    private final Map<String, ScenarioBundle> scenarioBundles = new LinkedHashMap<>();
    private final Map<String, HistoricalReplayDay> historicalReplayDays = new LinkedHashMap<>();
    private final List<SimulationScenario> scenarios = new ArrayList<>(canonicalScenarios());
    private final TrafficReplayAdapter trafficReplayAdapter = new TrafficReplayAdapter();
    private final NationalDemandCapacitySimulator nationalDemandCapacitySimulator = new NationalDemandCapacitySimulator();

    @Inject
    public OperationalSimulationService(AirspaceProductService productService) {
        this.productService = productService;
        canonicalHistoricalReplayDays().forEach(day -> historicalReplayDays.put(day.getId(), day));
    }

    public List<SimulationScenario> scenarios() {
        return scenarios;
    }

    public ScenarioValidationResult validateScenario(ScenarioBundle bundle) {
        List<String> errors = new ArrayList<>();
        List<String> warnings = new ArrayList<>();
        if (bundle == null) {
            errors.add("Scenario bundle is required.");
        } else {
            if (blank(bundle.getId())) warnings.add("Bundle id missing; scenario id will be used when imported.");
            if (bundle.getScenario() == null) {
                errors.add("Bundle scenario is required.");
            } else {
                if (blank(bundle.getScenario().getId())) errors.add("Scenario id is required.");
                if (blank(bundle.getScenario().getName())) warnings.add("Scenario name missing.");
                if (bundle.getScenario().getEvents() == null || bundle.getScenario().getEvents().isEmpty()) errors.add("At least one scheduled event is required.");
                if (bundle.getScenario().getRoute() == null || bundle.getScenario().getRoute().size() < 2) errors.add("Route must include at least two points.");
                if (blank(bundle.getScenario().getExpectedFinalAction())) warnings.add("Expected final action missing; campaign pass/fail will be permissive.");
            }
            if (bundle.getWeatherEnsembleConfig() == null) warnings.add("Weather ensemble config missing; simulator defaults will be used.");
            if (bundle.getTrafficFlow() == null || bundle.getTrafficFlow().getAircraft().isEmpty()) warnings.add("Traffic flow missing; default synthetic aircraft will be generated.");
            if (bundle.getTrafficReplay() == null) warnings.add("Traffic replay missing; default SWIM/TFMS-like local replay will be generated.");
            if (bundle.getTrafficReplay() != null) warnings.addAll(trafficReplayAdapter.validate(bundle.getTrafficReplay()).stream().map(message -> "Traffic replay: " + message).toList());
            if (bundle.getAirportOps() == null) warnings.add("Airport ops timeline missing; default airport surface model will be generated.");
        }
        String scenarioId = bundle == null || bundle.getScenario() == null ? null : bundle.getScenario().getId();
        return ScenarioValidationResult.builder()
                .scenarioId(scenarioId)
                .accepted(errors.isEmpty())
                .warnings(warnings)
                .errors(errors)
                .build();
    }

    public TrafficReplayValidationResult validateTrafficReplay(TrafficReplayBundle bundle) {
        List<String> diagnostics = trafficReplayAdapter.validate(bundle);
        return TrafficReplayValidationResult.builder()
                .replayId(bundle == null ? null : bundle.getId())
                .sourceMode(bundle == null ? null : bundle.getSourceMode())
                .accepted(bundle != null && diagnostics.stream().noneMatch(message -> message.toLowerCase(Locale.US).contains("missing")))
                .flightPlanCount(bundle == null || bundle.getFlightPlans() == null ? 0 : bundle.getFlightPlans().size())
                .positionCount(bundle == null || bundle.getPositions() == null ? 0 : bundle.getPositions().size())
                .airportDemandSnapshotCount(bundle == null || bundle.getAirportDemand() == null ? 0 : bundle.getAirportDemand().size())
                .sectorDemandSnapshotCount(bundle == null || bundle.getSectorDemand() == null ? 0 : bundle.getSectorDemand().size())
                .trafficManagementInitiativeCount(bundle == null || bundle.getTrafficManagementInitiatives() == null ? 0 : bundle.getTrafficManagementInitiatives().size())
                .diagnostics(diagnostics.isEmpty()
                        ? List.of("Traffic replay bundle is structurally complete for local simulation.")
                        : diagnostics)
                .build();
    }

    public List<HistoricalReplayDay> historicalReplayDays() {
        return new ArrayList<>(historicalReplayDays.values());
    }

    public HistoricalReplayDay historicalReplayDay(String id) {
        HistoricalReplayDay day = historicalReplayDays.get(id);
        if (day == null) {
            throw new IllegalArgumentException("Unknown historical replay day " + id);
        }
        return day;
    }

    public HistoricalReplayLoadResult loadHistoricalReplay(HistoricalReplayLoadRequest request) {
        HistoricalReplayLoadRequest safe = request == null ? new HistoricalReplayLoadRequest() : request;
        HistoricalReplayDay day = dayForLoad(safe);
        TrafficReplayBundle replay = replayForHistoricalLoad(safe, day);
        TrafficReplayValidationResult validation = validateTrafficReplay(replay);
        boolean accepted = validation.isAccepted() || !safe.isStrictValidation();
        List<String> warnings = new ArrayList<>();
        if (day == null) {
            warnings.add("Replay was loaded from request payload; no named corpus day metadata was available.");
        } else {
            warnings.addAll(day.getDataQualityWarnings());
            if (!"AUTHORIZED_OPERATIONAL".equalsIgnoreCase(valueOr(day.getAuthorizationMode(), ""))) {
                warnings.add("Corpus day is not authorized operational evidence; use for local replay/calibration readiness only.");
            }
        }
        SimulationRunResult run = null;
        if (accepted && safe.isRunSimulation()) {
            SimulationRunRequest runRequest = new SimulationRunRequest();
            runRequest.setScenarioId(valueOr(safe.getScenarioId(), day == null ? null : day.getScenarioId()));
            runRequest.setHistoricalReplayDayId(day == null ? null : day.getId());
            runRequest.setTrafficReplay(replay);
            runRequest.setActor(valueOr(safe.getActor(), "historical-replay"));
            runRequest.setIncludeSensitivity(true);
            run = run(runRequest);
        }
        HistoricalReplayCalibrationReport report = safe.isIncludeCalibrationReport()
                ? historicalReplayCalibrationReport(day == null ? List.of(syntheticDayForRequest(replay)) : List.of(day))
                : null;
        return HistoricalReplayLoadResult.builder()
                .dayId(day == null ? null : day.getId())
                .replayId(replay == null ? null : replay.getId())
                .sourceMode(replay == null ? null : replay.getSourceMode())
                .authorizationMode(day == null ? replay == null ? null : replay.getAuthorizationMode() : day.getAuthorizationMode())
                .accepted(accepted)
                .ranSimulation(run != null)
                .runId(run == null ? null : run.getId())
                .finalAction(run == null ? null : run.getFinalAction())
                .flightPlanCount(validation.getFlightPlanCount())
                .positionCount(validation.getPositionCount())
                .airportDemandSnapshotCount(validation.getAirportDemandSnapshotCount())
                .sectorDemandSnapshotCount(validation.getSectorDemandSnapshotCount())
                .tmiCount(validation.getTrafficManagementInitiativeCount())
                .expectedOutcomeCount(day == null || day.getExpectedOutcomes() == null ? 0 : day.getExpectedOutcomes().size())
                .calibrationReport(report)
                .diagnostics(validation.getDiagnostics())
                .warnings(warnings)
                .build();
    }

    public HistoricalReplayCalibrationReport historicalReplayCalibrationReport(HistoricalReplayLoadRequest request) {
        HistoricalReplayLoadRequest safe = request == null ? new HistoricalReplayLoadRequest() : request;
        if (!blank(safe.getDayId())) {
            return historicalReplayCalibrationReport(List.of(historicalReplayDay(safe.getDayId())));
        }
        if (safe.getTrafficReplay() != null) {
            return historicalReplayCalibrationReport(List.of(syntheticDayForRequest(safe.getTrafficReplay())));
        }
        return historicalReplayCalibrationReport(historicalReplayDays());
    }

    public NationalDemandCapacityReport previewNationalDemandCapacity(NationalDemandCapacityConfig config) {
        return nationalDemandCapacitySimulator.run(config);
    }

    public TfmCommandCenterSummary tfmCommandCenterBoard(TfmCommandCenterRequest request) {
        TfmCommandCenterRequest safe = request == null ? TfmCommandCenterRequest.builder().build() : request;
        NationalDemandCapacityReport report = nationalDemandCapacitySimulator.run(safe.getDemandCapacityConfig());
        NationalDemandCapacitySnapshot selectedSnapshot = selectedTfmSnapshot(report, safe.getFocusMinute());
        int selectedMinute = selectedSnapshot == null ? 0 : selectedSnapshot.getMinute();
        TrafficReplayBundle replay = report.getTrafficReplay();
        List<TfmAirportDemandSummary> airportDemand = tfmAirportDemand(replay, selectedMinute, safe.getMaxAirportRows());
        List<TfmSectorLoadSummary> sectorLoad = tfmSectorLoad(replay, selectedMinute, safe.getMaxSectorRows());
        List<TfmActiveConstraintSummary> activeConstraints = tfmActiveConstraints(replay, selectedMinute, safe.getMaxConstraintRows());
        List<TfmProposedTmiSummary> proposedTmis = tfmProposedTmis(replay, selectedSnapshot, selectedMinute, safe.getMaxProposalRows());
        List<TfmRouteAlternativeSummary> routeAlternatives = tfmRouteAlternatives(activeConstraints, proposedTmis, replay, selectedMinute, safe.getMaxRouteAlternatives());
        List<String> affectedFlightCandidates = new ArrayList<>(activeConstraints.stream()
                .flatMap(item -> item.getAffectedFlightIds().stream())
                .toList());
        proposedTmis.forEach(item -> affectedFlightCandidates.addAll(item.getAffectedFlightIds()));
        routeAlternatives.forEach(item -> affectedFlightCandidates.addAll(item.getAffectedFlightIds()));
        List<String> affectedFlights = distinct(affectedFlightCandidates);
        TfmImpactTotals impactTotals = TfmImpactTotals.builder()
                .flightCount(report.getFlightCount())
                .activeFlightCount(selectedSnapshot == null ? 0 : selectedSnapshot.getActiveFlightCount())
                .airportCount(report.getAirportCount())
                .sectorCount(report.getSectorCount())
                .overloadedAirportCount(selectedSnapshot == null ? 0 : selectedSnapshot.getOverloadedAirportCount())
                .overloadedSectorCount(selectedSnapshot == null ? 0 : selectedSnapshot.getOverloadedSectorCount())
                .maxAirportDemandCapacityRatio(selectedSnapshot == null ? 0.0 : selectedSnapshot.getMaxAirportDemandCapacityRatio())
                .maxSectorDemandCapacityRatio(selectedSnapshot == null ? 0.0 : selectedSnapshot.getMaxSectorDemandCapacityRatio())
                .totalExpectedDelayMinutes(selectedSnapshot == null ? 0 : selectedSnapshot.getTotalExpectedDelayMinutes())
                .activeConstraintCount(activeConstraints.size())
                .proposedTmiCount(proposedTmis.size())
                .routeAlternativeCount(routeAlternatives.size())
                .affectedFlightCount(affectedFlights.size())
                .humanApprovalRequired(true)
                .sourceMode(report.getSourceMode())
                .sourceFreshnessStatus("LOCAL_REPLAY_SYNTHETIC_CURRENT_TICK")
                .commonOperatingPictureStatus("SHARED_REVIEW_READY")
                .build();
        List<String> sourceRefs = new ArrayList<>();
        if (replay != null) {
            sourceRefs.addAll(replay.getSourceRefs());
            if (!blank(replay.getId())) sourceRefs.add("TRAFFIC_REPLAY:" + replay.getId());
            if (!blank(replay.getSourceId())) sourceRefs.add(replay.getSourceId());
        }
        airportDemand.forEach(item -> sourceRefs.addAll(item.getSourceRefs()));
        sectorLoad.forEach(item -> sourceRefs.addAll(item.getSourceRefs()));
        activeConstraints.forEach(item -> sourceRefs.addAll(item.getSourceRefs()));
        proposedTmis.forEach(item -> sourceRefs.addAll(item.getSourceRefs()));
        routeAlternatives.forEach(item -> sourceRefs.addAll(item.getSourceRefs()));
        return TfmCommandCenterSummary.builder()
                .id("tfm-board-" + valueOr(report.getId(), "local"))
                .generatedAt(ZonedDateTime.now(ZoneOffset.UTC).toString())
                .boardMode("LOCAL_COMMAND_CENTER_PREVIEW")
                .sourceMode(report.getSourceMode())
                .authorizationMode(replay == null ? "LOCAL_SYNTHETIC_ONLY" : valueOr(replay.getAuthorizationMode(), "LOCAL_SYNTHETIC_ONLY"))
                .selectedMinute(selectedMinute)
                .demandCapacityReport(report)
                .selectedSnapshot(selectedSnapshot)
                .airportDemand(airportDemand)
                .sectorLoad(sectorLoad)
                .activeConstraints(activeConstraints)
                .proposedTmis(proposedTmis)
                .routeAlternatives(routeAlternatives)
                .impactTotals(impactTotals)
                .humanFactorsNotes(List.of(
                        "Display airport demand/capacity, sector load, active constraints, proposed TMIs, route alternatives, and network impact in one common operating picture.",
                        "Recommend the least restrictive TMI that appears suitable for the demand/capacity imbalance; escalate only when the local replay indicates saturation.",
                        "All TMIs and reroutes are proposals requiring human review, coordination, and approval before any external delivery.",
                        "Rows preserve source references so an operator can inspect the replay, demand snapshot, active constraint, or generated recommendation that drove the board."))
                .assumptions(List.of(
                        "Local synthetic/fixture-backed demand and capacity are used unless an authorized provider is configured.",
                        "This board is inspired by FSM/TSD/OIS/CDM workflows, but it is not connected to live FMDS, TFMS, SWIM, FNS, or NMS.",
                        "Capacity values and delay estimates are representative simulation inputs, not official NAS constraints."))
                .diagnostics(List.of("TFM board generated from " + report.getSnapshots().size() + " demand/capacity snapshot(s) at T+" + selectedMinute + "M."))
                .sourceRefs(distinct(sourceRefs))
                .build();
    }

    public OutcomeMetricsReport outcomeMetrics(OutcomeMetricsRequest request) {
        OutcomeMetricsRequest safe = request == null ? OutcomeMetricsRequest.builder().build() : request;
        List<SimulationRunResult> selectedRuns = new ArrayList<>();
        String scope = "TFM_BOARD";
        String runId = null;
        String campaignId = null;
        String scenarioId = safe.getScenarioId();
        if (!blank(safe.getCampaignId())) {
            SimulationCampaignReport campaign = campaignById(safe.getCampaignId());
            selectedRuns.addAll(campaign.getRuns());
            campaignId = campaign.getId();
            scope = "CAMPAIGN";
        } else if (!blank(safe.getRunId())) {
            SimulationRunResult run = runById(safe.getRunId());
            selectedRuns.add(run);
            runId = run.getId();
            scenarioId = run.getScenarioId();
            scope = "RUN";
        } else if (safe.isRunSimulation()) {
            SimulationRunRequest runRequest = safe.getSimulationRunRequest() == null ? new SimulationRunRequest() : safe.getSimulationRunRequest();
            if (!blank(safe.getScenarioId()) && blank(runRequest.getScenarioId())) {
                runRequest.setScenarioId(safe.getScenarioId());
            }
            if (safe.getDemandCapacityConfig() != null && runRequest.getNationalDemandCapacityConfig() == null) {
                runRequest.setNationalDemandCapacityConfig(safe.getDemandCapacityConfig());
            }
            SimulationRunResult run = run(runRequest);
            selectedRuns.add(run);
            runId = run.getId();
            scenarioId = run.getScenarioId();
            scope = "RUN";
        }
        TfmCommandCenterSummary tfmBoard = safe.isIncludeTfmBoard()
                ? tfmCommandCenterBoard(TfmCommandCenterRequest.builder().demandCapacityConfig(safe.getDemandCapacityConfig()).build())
                : null;
        SimulationKpiSummary kpi = selectedRuns.isEmpty() ? null : selectedRuns.size() == 1 ? selectedRuns.get(0).getKpiSummary() : aggregateKpis(selectedRuns);
        double tfmBaselineDelay = tfmBoard == null || tfmBoard.getImpactTotals() == null ? 0.0 : tfmBoard.getImpactTotals().getTotalExpectedDelayMinutes();
        double tfmDelaySaved = tfmBoard == null ? 0.0 : Math.min(tfmBaselineDelay, estimatedTmiDelaySavings(tfmBoard));
        double baselineDelay = kpi == null ? tfmBaselineDelay : kpi.getBaselineDelayMinutes() + tfmBaselineDelay;
        double delaySaved = kpi == null ? tfmDelaySaved : kpi.getDelayMinutesSaved() + tfmDelaySaved;
        double mitigatedDelay = Math.max(0.0, baselineDelay - delaySaved);
        double additionalFuel = kpi == null ? 0.0 : kpi.getAdditionalFuelPounds();
        double holdingFuelSaved = (kpi == null ? 0.0 : kpi.getHoldingFuelSavedPounds()) + (tfmDelaySaved * 120.0);
        double fuelImpact = additionalFuel - holdingFuelSaved;
        int tfmOverloadAvoided = tfmBoard == null ? 0 : estimatedOverloadAvoided(tfmBoard);
        int overloadedAirports = (kpi == null ? 0 : kpi.getOverloadedAirportCount()) + (tfmBoard == null || tfmBoard.getImpactTotals() == null ? 0 : tfmBoard.getImpactTotals().getOverloadedAirportCount());
        int overloadedSectors = (kpi == null ? 0 : kpi.getOverloadedSectorCount()) + (tfmBoard == null || tfmBoard.getImpactTotals() == null ? 0 : tfmBoard.getImpactTotals().getOverloadedSectorCount());
        int proposedTmis = tfmBoard == null || tfmBoard.getImpactTotals() == null ? 0 : tfmBoard.getImpactTotals().getProposedTmiCount();
        int routeAlternatives = (tfmBoard == null || tfmBoard.getImpactTotals() == null ? 0 : tfmBoard.getImpactTotals().getRouteAlternativeCount())
                + selectedRuns.stream().flatMap(run -> run.getSteps().stream())
                .map(SimulationStepResult::getRouteImpact)
                .filter(impact -> impact != null && impact.getCandidateComparisons() != null)
                .mapToInt(impact -> impact.getCandidateComparisons().size())
                .sum();
        int affectedFlights = tfmBoard == null || tfmBoard.getImpactTotals() == null ? 0 : tfmBoard.getImpactTotals().getAffectedFlightCount();
        double sourceCompleteness = kpi == null ? (tfmBoard == null || tfmBoard.getSourceRefs().isEmpty() ? 0.0 : 1.0) : kpi.getSourceRefCompletenessRate();
        List<String> mutableRefs = new ArrayList<>();
        selectedRuns.forEach(run -> run.getSteps().forEach(step -> mutableRefs.addAll(step.getSourceRefs())));
        if (tfmBoard != null) mutableRefs.addAll(tfmBoard.getSourceRefs());
        List<String> refs = distinct(mutableRefs);
        OutcomeMetricsReport report = OutcomeMetricsReport.builder()
                .id("outcomes-" + UUID.randomUUID())
                .generatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .scope(scope)
                .sourceMode(tfmBoard == null ? "LOCAL_SIMULATION" : tfmBoard.getSourceMode())
                .runId(runId)
                .campaignId(campaignId)
                .scenarioId(scenarioId)
                .baselineDelayMinutes(baselineDelay)
                .mitigatedDelayMinutes(mitigatedDelay)
                .delayMinutesSaved(delaySaved)
                .rerouteMiles(kpi == null ? 0.0 : kpi.getRerouteMiles())
                .additionalFuelPounds(additionalFuel)
                .holdingFuelSavedPounds(holdingFuelSaved)
                .fuelImpactPounds(fuelImpact)
                .sectorOverloadAvoidedCount((kpi == null ? 0 : kpi.getSectorOverloadAvoidedCount()) + tfmOverloadAvoided)
                .falseClearCount(kpi == null ? 0 : kpi.getFalseClearCount())
                .falseBlockCount(kpi == null ? 0 : kpi.getFalseBlockCount())
                .sourceRefCompletenessRate(sourceCompleteness)
                .operatorTimeToDecisionSeconds(kpi == null ? -1 : kpi.getOperatorTimeToDecisionSeconds())
                .rerouteCandidateCount((int) selectedRuns.stream().flatMap(run -> run.getSteps().stream())
                .filter(step -> step.getRouteImpact() != null
                        && step.getRouteImpact().getAvoidanceCandidates() != null
                        && !step.getRouteImpact().getAvoidanceCandidates().isEmpty())
                        .count())
                .routeAlternativeCount(routeAlternatives)
                .affectedFlightCount(affectedFlights)
                .overloadedAirportCount(overloadedAirports)
                .overloadedSectorCount(overloadedSectors)
                .proposedTmiCount(proposedTmis)
                .sourceRefs(refs)
                .assumptions(List.of(
                        "Delay minutes saved compare a local baseline delay estimate against available reroute/TMI mitigation in simulation.",
                        "Fuel impact is additional reroute fuel minus avoided holding/delay fuel; negative values represent estimated fuel saved.",
                        "Sector overload avoided is an estimated outcome from proposed TMIs and sector-capacity/reroute recommendations, not authoritative NAS post-event measurement.",
                        "False-clear and false-block counts require scenario expected outcomes; default TFM-only reports carry zero safety labels."))
                .diagnostics(List.of(selectedRuns.isEmpty()
                        ? "Outcome report uses TFM board evidence only; run or campaign ids provide stronger safety labels."
                        : "Outcome report aggregates " + selectedRuns.size() + " simulation run(s) plus optional TFM board evidence."))
                .build();
        report.setMetrics(outcomeMetricCards(report));
        return report;
    }

    public ScenarioBundle importScenario(ScenarioBundle bundle) {
        ScenarioValidationResult validation = validateScenario(bundle);
        if (!validation.isAccepted()) {
            throw new IllegalArgumentException("Invalid scenario bundle: " + String.join("; ", validation.getErrors()));
        }
        SimulationScenario scenario = bundle.getScenario();
        scenarios.removeIf(existing -> existing.getId().equals(scenario.getId()));
        scenarios.add(scenario);
        ScenarioBundle normalized = bundle.toBuilder()
                .id(blank(bundle.getId()) ? scenario.getId() : bundle.getId())
                .build();
        scenarioBundles.put(scenario.getId(), normalized);
        return normalized;
    }

    public ScenarioBundle scenarioBundle(String id) {
        ScenarioBundle imported = scenarioBundles.get(id);
        if (imported != null) return imported;
        SimulationScenario scenario = scenario(id);
        return defaultBundleFor(scenario);
    }

    public SimulationRunResult run(SimulationRunRequest request) {
        SimulationRunRequest safe = request == null ? new SimulationRunRequest() : request;
        HistoricalReplayDay replayDay = blank(safe.getHistoricalReplayDayId()) ? null : historicalReplayDay(safe.getHistoricalReplayDayId());
        SimulationScenario scenario = scenario(blank(safe.getScenarioId()) && replayDay != null ? replayDay.getScenarioId() : safe.getScenarioId());
        SimulationClock clock = SimulationClock.builder().build();
        ZonedDateTime startedAt = ZonedDateTime.now(ZoneOffset.UTC);
        String actor = blank(safe.getActor()) ? "simulation" : safe.getActor();
        NationalDemandCapacityReport nationalReport = safe.getNationalDemandCapacityConfig() == null
                ? null
                : nationalDemandCapacitySimulator.run(safe.getNationalDemandCapacityConfig());

        ProductDtos.MissionRequest missionRequest = new ProductDtos.MissionRequest();
        missionRequest.setMissionNumber(scenario.getMissionNumber() + "-" + UUID.randomUUID().toString().substring(0, 6));
        missionRequest.setTitle("Simulation: " + scenario.getName());
        missionRequest.setRawText(scenario.getNarrative());
        missionRequest.setActor(actor);
        ProductDtos.MissionSummary mission = productService.createMission(missionRequest);

        ProductDtos.ReservationRequest reservationRequest = new ProductDtos.ReservationRequest();
        reservationRequest.setRawText(scenario.getCarfAltrv());
        reservationRequest.setActor(actor);
        ReservationWorkflowResult reservation = productService.createReservation(mission.getId(), reservationRequest);
        String reservationId = reservation.getRecord().getId();

        List<SimulationEvent> scheduledEvents = scenario.getEvents().stream()
                .sorted(Comparator.comparingInt(SimulationEvent::getOffsetMinutes).thenComparing(SimulationEvent::getId))
                .toList();
        TrafficReplayBundle trafficReplay = trafficReplayFor(scenario, safe, nationalReport);
        int firstMinute = scheduledEvents.stream().mapToInt(SimulationEvent::getOffsetMinutes).min().orElse(0);
        int lastMinute = scheduledEvents.stream().mapToInt(SimulationEvent::getOffsetMinutes).max().orElse(0);
        if (safe.getDurationMinutes() > 0) {
            lastMinute = Math.max(lastMinute, safe.getDurationMinutes());
        }
        int minuteStep = Math.max(1, (int) Math.ceil(Math.max(1, safe.getTickIntervalSeconds()) / 60.0));
        List<SimulationStepResult> steps = new ArrayList<>();
        List<String> previousSources = new ArrayList<>();
        String previousAction = "CLEAR";
        SimulationEvent lastSourceEvent = scheduledEvents.isEmpty() ? null : scheduledEvents.get(0);
        for (int minute = firstMinute; minute <= lastMinute; minute += minuteStep) {
            SimulationEvent scheduled = eventAt(scheduledEvents, minute);
            boolean injected = scheduled != null;
            if (injected) {
                injectEvent(mission.getId(), reservationId, scheduled, actor);
                lastSourceEvent = scheduled;
            }
            SimulationEvent event = injected ? scheduled : clockTickEvent(minute, lastSourceEvent, previousAction);
            ProductDtos.MissionWeatherVerdictSummary verdict = productService.missionWeatherVerdict(mission.getId());
            ProductDtos.RouteImpactSummary routeImpact = productService.routeImpact(mission.getId(), reservationId);
            ProductDtos.CoordinationDraftRequest draftRequest = new ProductDtos.CoordinationDraftRequest();
            draftRequest.setMissionId(mission.getId());
            draftRequest.setReservationId(reservationId);
            draftRequest.setHazardOrDecisionId(event.getId());
            draftRequest.setActor(actor);
            ProductDtos.CoordinationDraftSummary coordination = productService.coordinationDraft(mission.getId(), draftRequest);
            ProductDtos.PilotBriefSummary brief = productService.pilotBrief(mission.getId(), clock.atOffsetMinutes(event.getOffsetMinutes()).minusMinutes(30).toString());
            String action = actionForStep(scenario, event, routeImpact, verdict);
            List<String> sourceRefs = sourceRefs(event, routeImpact, verdict);
            List<String> affectedDeltas = affectedDeltas(previousAction, action, previousSources, sourceRefs, event);
            List<String> routeDeltas = routeDeltas(routeImpact, action, event);
            SimulationDynamicsSnapshot dynamics = dynamicsFor(scenario, event, action, routeImpact, safe, trafficReplay, nationalReport);
            SimulationStepResult step = SimulationStepResult.builder()
                    .id("step-" + event.getOffsetMinutes() + "-" + event.getId())
                    .offsetMinutes(event.getOffsetMinutes())
                    .simulatedTime(clock.atOffsetMinutes(event.getOffsetMinutes()))
                    .injectedEvent(event)
                    .engineAction(action)
                    .recommendedAction(recommendedAction(action, event))
                    .confidence(confidenceFor(action, event, routeImpact))
                    .missionVerdict(verdict)
                    .routeImpact(routeImpact)
                    .coordinationDraft(coordination)
                    .pilotBrief(brief)
                    .features(featuresFor(scenario, event, action, mission.getId(), reservationId))
                    .dynamics(dynamics)
                    .affectedMissionDeltas(affectedDeltas)
                    .routeImpactDeltas(routeDeltas)
                    .sourceRefs(sourceRefs)
                    .replayAuditIds(Map.of(
                            "decision", "simulation-decision-" + event.getId(),
                            "replay", "simulation-replay-" + event.getId(),
                            "audit", "simulation-audit-" + event.getId()))
                    .diagnostics(List.of("Fixture-backed timestep; no live FAA/SWIM/FNS/NMS/NADIN/WMSCR provider was used."))
                    .build();
            steps.add(step);
            previousAction = action;
            previousSources = sourceRefs;
        }
        String finalAction = steps.isEmpty() ? "CLEAR" : steps.get(steps.size() - 1).getEngineAction();
        String runId = "sim-run-" + UUID.randomUUID();
        SimulationWorldState worldState = worldStateFor(runId, scenario, steps, safe, trafficReplay, nationalReport);
        SimulationReplayBundle replayBundle = replayBundleFor(runId, scenario, steps, worldState, finalAction);
        SimulationRunResult result = SimulationRunResult.builder()
                .id(runId)
                .scenarioId(scenario.getId())
                .scenarioName(scenario.getName())
                .missionId(mission.getId())
                .reservationId(reservationId)
                .missionNumber(mission.getMissionNumber())
                .startedAt(startedAt)
                .completedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .finalAction(finalAction)
                .expectedFinalAction(scenario.getExpectedFinalAction())
                .steps(steps)
                .sensitivity(sensitivity(scenario, safe))
                .kpiSummary(kpis(scenario, steps, nationalReport))
                .worldState(worldState)
                .replayBundle(replayBundle)
                .nationalDemandCapacityReport(nationalReport)
                .diagnostics(List.of("Operational decision simulator only; not a flight dynamics, cockpit avionics, or certified NAS traffic simulator."))
                .build();
        runs.put(result.getId(), result);
        return result;
    }

    public SimulationCampaignReport campaign(SimulationCampaignRequest request) {
        SimulationCampaignRequest safe = request == null ? new SimulationCampaignRequest() : request;
        List<String> ids = safe.getScenarioIds() == null || safe.getScenarioIds().isEmpty()
                ? scenarios.stream().map(SimulationScenario::getId).toList()
                : safe.getScenarioIds();
        List<SimulationRunResult> campaignRuns = new ArrayList<>();
        int iterations = Math.max(1, safe.getIterationsPerScenario());
        int simulatedDays = Math.max(1, safe.getSimulatedDayCount());
        for (int day = 0; day < simulatedDays; day++) {
            for (String id : ids) {
                for (int iteration = 0; iteration < iterations; iteration++) {
                    SimulationRunRequest runRequest = new SimulationRunRequest();
                    runRequest.setScenarioId(id);
                    runRequest.setActor(blank(safe.getActor()) ? "campaign" : safe.getActor());
                    runRequest.setIncludeSensitivity(safe.isIncludeSensitivity());
                    if (safe.isIncludeSensitivity()) {
                        runRequest.setSensitivityOverrides(Map.of(
                                "forecastConfidence", Math.max(0.35, 0.82 - (iteration * 0.03)),
                                "weatherMovementSpeedKt", 25.0 + iteration,
                                "communicationDelaySeconds", 30.0 + (day * 5.0)));
                    }
                    campaignRuns.add(run(runRequest));
                }
            }
        }
        int passed = (int) campaignRuns.stream()
                .filter(run -> actionMatches(run.getFinalAction(), run.getExpectedFinalAction()))
                .count();
        SimulationCampaignReport report = SimulationCampaignReport.builder()
                .id("sim-campaign-" + UUID.randomUUID())
                .generatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .scenarioCount(campaignRuns.size())
                .passedScenarioCount(passed)
                .runs(campaignRuns)
                .aggregateKpis(aggregateKpis(campaignRuns))
                .diagnostics(List.of("Campaign uses local deterministic scenarios and fixture-backed scoring."))
                .build();
        campaigns.put(report.getId(), report);
        return report;
    }

    public SimulationRunResult runById(String id) {
        SimulationRunResult result = runs.get(id);
        if (result == null) {
            throw new IllegalArgumentException("Unknown simulation run: " + id);
        }
        return result;
    }

    public List<SimulationStepResult> timeline(String id) {
        return runById(id).getSteps();
    }

    public AirspaceFeatureCollection features(String id) {
        AirspaceFeatureCollection out = new AirspaceFeatureCollection();
        for (SimulationStepResult step : timeline(id)) {
            if (step.getFeatures() != null) {
                out.getFeatures().addAll(step.getFeatures().getFeatures());
            }
        }
        return out;
    }

    public SimulationWorldState worldState(String id) {
        return runById(id).getWorldState();
    }

    public SimulationReplayBundle replay(String id) {
        return runById(id).getReplayBundle();
    }

    public SimulationCampaignReport campaignById(String id) {
        SimulationCampaignReport report = campaigns.get(id);
        if (report == null) {
            throw new IllegalArgumentException("Unknown simulation campaign: " + id);
        }
        return report;
    }

    public SafetyDossierExport campaignDossier(String id) {
        SimulationCampaignReport report = campaignById(id);
        String markdown = safetyDossierMarkdown(report);
        Map<String, Object> json = new LinkedHashMap<>();
        json.put("campaignId", report.getId());
        json.put("scenarioCount", report.getScenarioCount());
        json.put("passedScenarioCount", report.getPassedScenarioCount());
        json.put("timeToGuidanceSeconds", report.getAggregateKpis().getTimeToGuidanceSeconds());
        json.put("falseClearCount", report.getAggregateKpis().getFalseClearCount());
        json.put("falseBlockCount", report.getAggregateKpis().getFalseBlockCount());
        return SafetyDossierExport.builder()
                .id("sim-dossier-" + sha256(report.getId()).substring(0, 12))
                .campaignId(report.getId())
                .generatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .format("MARKDOWN_AND_JSON")
                .markdown(markdown)
                .jsonSummary(json)
                .scenarios(report.getRuns().stream().map(SimulationRunResult::getScenarioId).distinct().toList())
                .assumptions(List.of("Local deterministic replay only.", "No FAA certification claim.", "No autonomous official workflow mutation."))
                .knownGaps(List.of("Live SWIM/FNS/NMS adapters require authorization.", "Calibration requires authoritative historical outcomes.", "Aircraft performance is approximate."))
                .nonCertificationWarnings(List.of("Not an FAA-qualified FSTD.", "Not a cockpit avionics display.", "Not an operational clearance authority."))
                .replayHashes(report.getRuns().stream().map(run -> run.getReplayBundle().getReplayHashes().get("result")).toList())
                .build();
    }

    public SimulationAgentReport generateScenarios(SimulationAgentRequest request) {
        SimulationAgentRequest safe = request == null ? new SimulationAgentRequest() : request;
        int count = Math.max(1, safe.getCount());
        List<ScenarioBundle> drafts = new ArrayList<>();
        for (int i = 0; i < count; i++) {
            String type = blank(safe.getScenarioType()) ? (i % 2 == 0 ? "SEVERE_CONVECTION_BLOCKS_ROUTE" : "LOW_VISIBILITY_PROCEDURE_AMBIGUITY") : safe.getScenarioType();
            String id = "generated-" + normalizeId(type) + "-" + i;
            SimulationScenario scenario = scenario(id,
                    "Generated " + type.replace('_', ' '),
                    "Autonomous Draft Scenario",
                    type.toUpperCase(Locale.US).contains("NO_VIABLE") ? "BLOCKED" : "REROUTE_OR_CAUTION",
                    "Generated draft from simulation agent focus areas: " + String.join(", ", safe.getFocusAreas()),
                    Arrays.asList(Arrays.asList(30.0, -150.5, 24000.0), Arrays.asList(30.8, -149.4, 26000.0)),
                    event("evt-generated-" + i, 0, "WEATHER", "SIGMET GENERATED VALID 201200/201600 FROM 3000N15000W TO 3100N14900W EMBD TS TOP FL430", "Generated weather hazard", "REROUTE"));
            drafts.add(defaultBundleFor(scenario).toBuilder()
                    .id(id)
                    .name(scenario.getName())
                    .useCase("AUTONOMOUS_DRAFT_REQUIRES_HUMAN_REVIEW")
                    .build());
        }
        return SimulationAgentReport.builder()
                .id("sim-agent-generation-" + UUID.randomUUID())
                .agentType("SCENARIO_GENERATION")
                .generatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .generatedScenarioDrafts(drafts)
                .findings(List.of("Generated scenario drafts are not added to the regression corpus until imported by a human operator."))
                .citations(List.of("simulation:scenario-library", "policy:HUMAN_REVIEW_REQUIRED"))
                .policyGuards(List.of("NO_AUTONOMOUS_IMPORT", "NO_EXTERNAL_SEND", "NO_CERTIFICATION_CLAIM"))
                .build();
    }

    public SimulationAgentReport redTeam(SimulationAgentRequest request) {
        SimulationAgentRequest safe = request == null ? new SimulationAgentRequest() : request;
        SimulationRunResult run = !blank(safe.getRunId()) && runs.containsKey(safe.getRunId())
                ? runById(safe.getRunId())
                : run(new SimulationRunRequest());
        List<String> findings = new ArrayList<>();
        if (run.getKpiSummary().getFalseClearCount() > 0) findings.add("Potential false-clear detected in " + run.getId());
        if (run.getKpiSummary().getFalseBlockCount() > 0) findings.add("Potential false-block detected in " + run.getId());
        if (run.getSteps().stream().anyMatch(step -> step.getSourceRefs().isEmpty())) findings.add("One or more steps lost source references.");
        if (findings.isEmpty()) findings.add("No unsafe guidance regression found in local deterministic red-team pass.");
        return SimulationAgentReport.builder()
                .id("sim-agent-red-team-" + UUID.randomUUID())
                .agentType("UNSAFE_GUIDANCE_RED_TEAM")
                .generatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .findings(findings)
                .citations(List.of("simulation-run:" + run.getId(), "scenario:" + run.getScenarioId(), "policy:HUMAN_REVIEW_REQUIRED"))
                .policyGuards(List.of("ADVISORY_ONLY", "NO_OFFICIAL_MUTATION", "NO_EXTERNAL_SEND"))
                .build();
    }

    private HistoricalReplayDay dayForLoad(HistoricalReplayLoadRequest request) {
        if (request == null || blank(request.getDayId())) {
            return null;
        }
        return historicalReplayDay(request.getDayId());
    }

    private TrafficReplayBundle replayForHistoricalLoad(HistoricalReplayLoadRequest request, HistoricalReplayDay day) {
        if (request != null && request.getTrafficReplay() != null) {
            return request.getTrafficReplay();
        }
        return day == null ? null : day.getTrafficReplay();
    }

    private HistoricalReplayDay syntheticDayForRequest(TrafficReplayBundle replay) {
        return HistoricalReplayDay.builder()
                .id(valueOr(replay == null ? null : replay.getId(), "ad-hoc-historical-replay"))
                .name("Ad Hoc Historical Replay Payload")
                .scenarioId("low-vis-rvr-smgcs")
                .sourceMode(valueOr(replay == null ? null : replay.getSourceMode(), "LOCAL_FIXTURE_REPLAY"))
                .sourceFamily(valueOr(replay == null ? null : replay.getProviderFamily(), "USER_SUPPLIED_REPLAY"))
                .authorizationMode(valueOr(replay == null ? null : replay.getAuthorizationMode(), "USER_SUPPLIED_NOT_AUTHORIZED_OPERATIONAL"))
                .providerFamily(valueOr(replay == null ? null : replay.getProviderFamily(), "USER_SUPPLIED_REPLAY"))
                .trafficReplay(replay)
                .dataQualityWarnings(List.of("Ad hoc replay has no named corpus day or expected outcome labels."))
                .assumptions(List.of("User-supplied replay is accepted only as local calibration/readiness input."))
                .build();
    }

    private HistoricalReplayCalibrationReport historicalReplayCalibrationReport(List<HistoricalReplayDay> days) {
        List<HistoricalReplayDay> safeDays = days == null ? List.of() : days;
        List<HistoricalReplayOutcome> outcomes = safeDays.stream()
                .filter(day -> day.getExpectedOutcomes() != null)
                .flatMap(day -> day.getExpectedOutcomes().stream())
                .toList();
        Map<String, Integer> byType = new LinkedHashMap<>();
        for (HistoricalReplayOutcome outcome : outcomes) {
            String type = valueOr(outcome.getOutcomeType(), "UNKNOWN");
            byType.put(type, byType.getOrDefault(type, 0) + 1);
        }
        long actionLabeled = outcomes.stream().filter(outcome -> !blank(outcome.getExpectedAction())).count();
        long actionMatches = outcomes.stream()
                .filter(outcome -> !blank(outcome.getExpectedAction()))
                .filter(outcome -> blank(outcome.getObservedAction()) || actionMatches(outcome.getObservedAction(), outcome.getExpectedAction()))
                .count();
        long sourceLabeled = outcomes.stream().filter(outcome -> outcome.getSourceRefs() != null && !outcome.getSourceRefs().isEmpty()).count();
        List<String> sourceRefs = safeDays.stream()
                .flatMap(day -> day.getPublicSourceRefs() == null ? List.<String>of().stream() : day.getPublicSourceRefs().stream())
                .distinct()
                .toList();
        List<String> sourceModes = safeDays.stream()
                .map(HistoricalReplayDay::getSourceMode)
                .filter(mode -> mode != null && !mode.isBlank())
                .distinct()
                .toList();
        List<String> uncalibrated = new ArrayList<>(List.of(
                "CWAP-style/CWAF-like route blockage coefficient calibration",
                "storm-cell lifecycle transition calibration",
                "PIREP aging/decay validation",
                "sector demand/capacity outcome calibration"));
        boolean hasAuthorized = safeDays.stream().anyMatch(day -> "AUTHORIZED_OPERATIONAL".equalsIgnoreCase(day.getAuthorizationMode()));
        uncalibrated.add(hasAuthorized
                ? "Authorized-provider ingestion requires independent review before operational use."
                : "No authorized operational historical dataset loaded.");
        return HistoricalReplayCalibrationReport.builder()
                .id("historical-calibration-" + sha256(safeDays.stream().map(HistoricalReplayDay::getId).toList().toString()).substring(0, 12))
                .generatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .calibrationVersion("historical-replay-corpus-v1")
                .corpusDayCount(safeDays.size())
                .expectedOutcomeCount(outcomes.size())
                .routeOutcomeCount(countType(outcomes, "ROUTE"))
                .weatherOutcomeCount(countType(outcomes, "WEATHER"))
                .pirepOutcomeCount(countType(outcomes, "PIREP"))
                .notamOutcomeCount(countType(outcomes, "NOTAM"))
                .sectorCapacityOutcomeCount(countType(outcomes, "SECTOR_CAPACITY"))
                .falseClearLabelCount((int) outcomes.stream().filter(HistoricalReplayOutcome::isFalseClearLabel).count())
                .falseBlockLabelCount((int) outcomes.stream().filter(HistoricalReplayOutcome::isFalseBlockLabel).count())
                .deterministicAgreementRate(actionLabeled == 0 ? 0.0 : (double) actionMatches / actionLabeled)
                .sourceRefPreservationRate(outcomes.isEmpty() ? 0.0 : (double) sourceLabeled / outcomes.size())
                .sourceModes(sourceModes)
                .sourceRefs(sourceRefs)
                .uncalibratedCoefficientCount(uncalibrated.size())
                .uncalibratedCoefficients(uncalibrated)
                .outcomeCountsByType(byType)
                .diagnostics(List.of(
                        "Report uses synthetic/public-historical-like replay labels unless authorizationMode is AUTHORIZED_OPERATIONAL.",
                        "Calibration report is readiness evidence, not certified model validation."))
                .build();
    }

    private int countType(List<HistoricalReplayOutcome> outcomes, String token) {
        if (outcomes == null) return 0;
        return (int) outcomes.stream()
                .filter(outcome -> String.valueOf(outcome.getOutcomeType()).toUpperCase(Locale.US).contains(token))
                .count();
    }

    private List<HistoricalReplayDay> canonicalHistoricalReplayDays() {
        SimulationScenario lowVis = scenario("low-vis-rvr-smgcs");
        SimulationScenario convection = scenario("oceanic-altrv-convection");
        SimulationScenario pirep = scenario("pirep-safety-override");
        return List.of(
                historicalDay(lowVis,
                        "public-like-jfk-lowvis-opsnet-bts-awc",
                        "Public Historical-Like JFK Low Visibility Day",
                        "PUBLIC_HISTORICAL_LIKE",
                        "PUBLIC_HISTORICAL_LIKE",
                        "KJFK",
                        List.of("ZNY-N90"),
                        List.of("LOW_VISIBILITY", "RVR", "SMGCS", "BTS", "OPSNET", "NCEI", "AWC_ARCHIVE"),
                        List.of(
                                "FAA_OPSNET:public-finalized-monthly-delay-data",
                                "BTS_TRANSTATS:on-time-schedule-delay-proxy",
                                "NOAA_NCEI:ASOS_AWOS_archive",
                                "AWC_ARCHIVE:aviationweather_recent_archive"),
                        List.of(
                                outcome("jfk-lowvis-route-delay", "RUNWAY_SURFACE_ROUTE", "RVR/SMGCS ambiguity delays departure", "DELAY_OR_CAUTION", "DELAY", "AIRPORT", "KJFK", 5, 18, false, false, true, false, List.of("METAR:evt-lowvis-1", "NOTAM:evt-lowvis-2")),
                                outcome("jfk-lowvis-source-ref", "NOTAM_PROCEDURE", "NOTAM-like procedure ambiguity remains source cited", "DELAY_OR_CAUTION", "DELAY", "RUNWAY", "KJFK-04R", 5, 0, false, false, false, false, List.of("NOTAM:evt-lowvis-2"))),
                        "Public sources can support historical-like delay/weather reconstruction; this fixture is not a downloaded official day."),
                historicalDay(convection,
                        "synthetic-oceanic-convection-route-blockage",
                        "Synthetic Oceanic Convection Route Blockage Day",
                        "SYNTHETIC",
                        "LOCAL_FIXTURE_ONLY",
                        "KZNY",
                        List.of("OCEANIC-SIM-SECTOR"),
                        List.of("CWAF_CWAP_LIKE", "CONVECTION", "ALTRV", "REROUTE"),
                        List.of("LOCAL_SIMULATION:convective-cell-route-crossing"),
                        List.of(
                                outcome("oceanic-route-reroute", "ROUTE_WEATHER", "Moving convection crosses ALTRV corridor", "REROUTE_OR_BLOCKED", "REROUTE", "ROUTE", convection.getId(), 15, 22, true, true, false, false, List.of("WEATHER:evt-conv-1", "PIREP:evt-conv-2")),
                                outcome("oceanic-sector-capacity", "SECTOR_CAPACITY", "Reroute candidates compress adjacent sector capacity", "REROUTE_OR_BLOCKED", "REROUTE", "SECTOR", "OCEANIC-SIM-SECTOR", 15, 12, false, true, false, true, List.of("TMI:local-replay:" + convection.getId()))),
                        "Synthetic day exercises route-impact and sector-capacity calibration seams."),
                historicalDay(pirep,
                        "synthetic-pirep-override-weather-day",
                        "Synthetic Severe PIREP Override Day",
                        "SYNTHETIC",
                        "LOCAL_FIXTURE_ONLY",
                        "KDEN",
                        List.of("ZDV-SIM"),
                        List.of("PIREP", "ICING", "TURBULENCE", "PILOT_REPORT_OVERRIDE"),
                        List.of("LOCAL_SIMULATION:severe-pirep-override"),
                        List.of(
                                outcome("pirep-severe-caution", "PIREP_WEATHER", "Severe PIREP overrides mild forecast confidence", "CAUTION", "CAUTION", "ROUTE", pirep.getId(), 10, 8, false, false, false, false, List.of("PIREP:evt-pirep-2")),
                                outcome("pirep-aging-calibration", "PIREP_AGING", "Fresh PIREP keeps confidence high before decay", "CAUTION", "CAUTION", "WEATHER_PRODUCT", "evt-pirep-2", 10, 0, false, false, false, false, List.of("PIREP:evt-pirep-2"))),
                        "Synthetic day supplies labeled PIREP relevance and aging behavior for regression."));
    }

    private HistoricalReplayDay historicalDay(SimulationScenario scenario,
                                              String id,
                                              String name,
                                              String sourceMode,
                                              String authorizationMode,
                                              String airportId,
                                              List<String> sectorIds,
                                              List<String> tags,
                                              List<String> sourceRefs,
                                              List<HistoricalReplayOutcome> outcomes,
                                              String warning) {
        TrafficReplayBundle replay = trafficReplayAdapter.defaultReplayForScenario(scenario).toBuilder()
                .id("historical-replay-" + id)
                .sourceId("historical-corpus:" + id)
                .sourceMode(sourceMode)
                .providerFamily("HISTORICAL_REPLAY_CORPUS")
                .authorizationMode(authorizationMode)
                .providerReceiptId("local-receipt-" + id)
                .sourceRefs(sourceRefs)
                .diagnostics(List.of("Historical replay corpus fixture; not live SWIM/TFMS data."))
                .build();
        return HistoricalReplayDay.builder()
                .id(id)
                .name(name)
                .operatingDate("2026-06-20")
                .scenarioId(scenario.getId())
                .sourceMode(sourceMode)
                .sourceFamily("HISTORICAL_REPLAY_CORPUS")
                .authorizationMode(authorizationMode)
                .providerFamily("TRAFFIC_REPLAY_PLUS_PUBLIC_OUTCOME_LABELS")
                .sourceVersion("historical-replay-corpus-v1")
                .expectedFinalAction(scenario.getExpectedFinalAction())
                .trafficReplay(replay)
                .airportIds(List.of(airportId))
                .sectorIds(sectorIds)
                .tags(tags)
                .publicSourceRefs(sourceRefs)
                .providerReceiptIds(List.of("local-receipt-" + id))
                .dataQualityWarnings(List.of(warning, "No authorized operational historical feed was used."))
                .assumptions(List.of(
                        "Traffic replay is offset-minute and deterministic.",
                        "Public sources inform schema shape and outcome categories, not operational certification.",
                        "Authorized SWIM/TFMS/ASPM/NCEI payloads can map into this day contract later."))
                .expectedOutcomes(outcomes)
                .expectedSummary(Map.of(
                        "expectedFinalAction", scenario.getExpectedFinalAction(),
                        "sourceMode", sourceMode,
                        "authorizationMode", authorizationMode))
                .build();
    }

    private HistoricalReplayOutcome outcome(String id,
                                            String type,
                                            String label,
                                            String expectedAction,
                                            String observedAction,
                                            String targetType,
                                            String targetId,
                                            int offsetMinutes,
                                            int expectedDelayMinutes,
                                            boolean routeBlocked,
                                            boolean rerouteExpected,
                                            boolean staleExpected,
                                            boolean capacityCompressionExpected,
                                            List<String> sourceRefs) {
        return HistoricalReplayOutcome.builder()
                .id(id)
                .outcomeType(type)
                .label(label)
                .expectedAction(expectedAction)
                .observedAction(observedAction)
                .targetResourceType(targetType)
                .targetResourceId(targetId)
                .offsetMinutes(offsetMinutes)
                .expectedDelayMinutes(expectedDelayMinutes)
                .expectedConfidence(0.74)
                .routeBlocked(routeBlocked)
                .rerouteExpected(rerouteExpected)
                .staleDataExpected(staleExpected)
                .capacityCompressionExpected(capacityCompressionExpected)
                .sourceRefs(sourceRefs)
                .diagnostics(List.of("Fixture outcome label for deterministic replay/calibration readiness."))
                .build();
    }

    private ScenarioBundle defaultBundleFor(SimulationScenario scenario) {
        TrafficReplayBundle trafficReplay = trafficReplayAdapter.defaultReplayForScenario(scenario);
        return ScenarioBundle.builder()
                .id(scenario.getId())
                .name(scenario.getName())
                .airportId("KJFK")
                .region("LOCAL_FIXTURE")
                .useCase(scenario.getCapabilityStory())
                .scenario(scenario)
                .trafficReplay(trafficReplay)
                .trafficFlow(trafficReplayAdapter.toTrafficFlowScenario(trafficReplay))
                .airportOps(AirportOpsTimeline.builder()
                        .airportId("KJFK")
                        .sourceMode("LOCAL_FIXTURE")
                        .runwayStates(List.of(RunwayOpsState.builder()
                                .airportId("KJFK")
                                .runwayId("04R")
                                .runwayLengthFeet(8400)
                                .runwayVisualRangeFeet(6000)
                                .brakingAction("NOT_REPORTED")
                                .departureRatePerHour(24)
                                .arrivalRatePerHour(24)
                                .build()))
                        .surfaceProcedures(List.of(SurfaceProcedureState.builder()
                                .procedureFamily("LOW_VISIBILITY")
                                .localProcedureName("SMGCS/local airport low visibility procedure")
                                .confirmationStatus("SIMULATED_REVIEW_REQUIRED")
                                .rationale("Local fixture timeline; official airport procedures remain authoritative.")
                                .build()))
                        .assumptions("Airport ops are deterministic fixture approximations.")
                        .build())
                .weatherEnsembleConfig(WeatherEnsembleConfig.builder()
                        .memberCount(16)
                        .randomSeed(20260620L)
                        .movementSpeedKnots(25)
                        .forecastConfidence(0.75)
                        .growthRate(0.08)
                        .decayRate(0.03)
                        .assumptions("Deterministic seeded ensemble seam.")
                        .build())
                .kpiGates(List.of(
                        CampaignKpiGate.builder().metric("falseClearCount").operator("<=").threshold(0).blocking(true).rationale("Never silently clear an expected hazard scenario.").build(),
                        CampaignKpiGate.builder().metric("sourceRefPreservationRate").operator(">=").threshold(0.95).blocking(true).rationale("Guidance must remain source-cited.").build()))
                .expectedSummary(Map.of(
                        "expectedFinalAction", scenario.getExpectedFinalAction(),
                        "capabilityStory", scenario.getCapabilityStory()))
                .build();
    }

    private SimulationWorldState worldStateFor(String runId,
                                               SimulationScenario scenario,
                                               List<SimulationStepResult> steps,
                                               SimulationRunRequest request,
                                               TrafficReplayBundle trafficReplay,
                                               NationalDemandCapacityReport nationalReport) {
        int duration = steps.isEmpty() ? 0 : steps.get(steps.size() - 1).getOffsetMinutes();
        return SimulationWorldState.builder()
                .runId(runId)
                .scenarioId(scenario.getId())
                .tickIntervalSeconds(Math.max(1, request.getTickIntervalSeconds()))
                .durationMinutes(duration)
                .randomSeed(request.getRandomSeed())
                .generatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .nationalDemandCapacityReport(nationalReport)
                .ticks(steps.stream().map(step -> tickFor(scenario, step, request, trafficReplay, nationalReport)).toList())
                .assumptions(List.of(
                        "Renderer-neutral GeoJSON remains the map contract.",
                        "Traffic, sector, airport, weather, and behavior models are deterministic local approximations.",
                        "Official feeds and calibration are disabled unless explicitly configured."))
                .build();
    }

    private SimulationReplayBundle replayBundleFor(String runId, SimulationScenario scenario, List<SimulationStepResult> steps, SimulationWorldState worldState, String finalAction) {
        Map<String, String> hashes = new LinkedHashMap<>();
        hashes.put("scenario", sha256(scenario.getId() + ":" + scenario.getName()));
        hashes.put("worldState", sha256(worldState.getScenarioId() + ":" + worldState.getTicks().size() + ":" + worldState.getRandomSeed()));
        hashes.put("result", sha256(runId + ":" + scenario.getId() + ":" + finalAction + ":" + steps.size()));
        return SimulationReplayBundle.builder()
                .id("sim-replay-" + hashes.get("result").substring(0, 12))
                .runId(runId)
                .scenarioId(scenario.getId())
                .finalAction(finalAction)
                .generatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .worldState(worldState)
                .steps(steps)
                .replayHashes(hashes)
                .diagnostics(List.of("Replay bundle is deterministic for local review but not certified operational evidence."))
                .build();
    }

    private TrafficReplayBundle trafficReplayFor(SimulationScenario scenario,
                                                 SimulationRunRequest request,
                                                 NationalDemandCapacityReport nationalReport) {
        if (nationalReport != null && nationalReport.getTrafficReplay() != null) {
            return nationalReport.getTrafficReplay();
        }
        if (request.getTrafficReplay() != null) {
            return request.getTrafficReplay();
        }
        if (!blank(request.getHistoricalReplayDayId())) {
            HistoricalReplayDay day = historicalReplayDay(request.getHistoricalReplayDayId());
            if (day.getTrafficReplay() != null) {
                return day.getTrafficReplay();
            }
        }
        ScenarioBundle bundle = scenarioBundles.get(scenario.getId());
        if (bundle != null && bundle.getTrafficReplay() != null) {
            return bundle.getTrafficReplay();
        }
        return trafficReplayAdapter.defaultReplayForScenario(scenario);
    }

    private Optional<TrafficReplayPosition> primaryReplayPosition(TrafficReplayBundle trafficReplay, int minute) {
        if (trafficReplay == null || trafficReplay.getFlightPlans() == null || trafficReplay.getFlightPlans().isEmpty()) {
            return Optional.empty();
        }
        return trafficReplayAdapter.positionAtMinute(trafficReplay, trafficReplay.getFlightPlans().get(0).getFlightId(), minute);
    }

    private SimulationTick tickFor(SimulationScenario scenario,
                                   SimulationStepResult step,
                                   SimulationRunRequest request,
                                   TrafficReplayBundle trafficReplay,
                                   NationalDemandCapacityReport nationalReport) {
        SimulationDynamicsSnapshot dynamics = step.getDynamics();
        return SimulationTick.builder()
                .id("tick-" + step.getOffsetMinutes())
                .minute(step.getOffsetMinutes())
                .simulatedTime(step.getSimulatedTime())
                .engineAction(step.getEngineAction())
                .recommendedAction(step.getRecommendedAction())
                .confidence(step.getConfidence())
                .aircraft(aircraftFleetFor(scenario, dynamics, step.getSourceRefs(), request, trafficReplay))
                .airportOps(airportOpsFor(dynamics))
                .sectorWorkload(sectorFor(dynamics))
                .nationalDemandCapacity(nationalDemandCapacitySimulator.snapshotAtMinute(nationalReport, step.getOffsetMinutes()))
                .weatherEvolution(weatherFor(step, dynamics, request))
                .behaviorStates(behaviorsFor(dynamics))
                .sourceRefs(step.getSourceRefs())
                .replayHashes(Map.of("tick", sha256(step.getId() + ":" + step.getEngineAction() + ":" + step.getSourceRefs())))
                .build();
    }

    private List<SimulatedAircraft> aircraftFleetFor(SimulationScenario scenario,
                                                     SimulationDynamicsSnapshot dynamics,
                                                     List<String> sourceRefs,
                                                     SimulationRunRequest request,
                                                     TrafficReplayBundle trafficReplay) {
        if (request.getAircraftFleet() != null && !request.getAircraftFleet().isEmpty()) {
            return request.getAircraftFleet().stream()
                    .map(aircraft -> aircraft.toBuilder()
                            .trajectory(aircraft.getTrajectory() == null ? trajectoryFrom(dynamics, 0) : aircraft.getTrajectory())
                            .impacted(!sourceRefs.isEmpty())
                            .impactedSourceRefs(sourceRefs)
                    .build())
                    .toList();
        }
        int minute = dynamics == null ? 0 : dynamics.getMinute();
        List<SimulatedAircraft> replayAircraft = trafficReplayAdapter.aircraftAtMinute(trafficReplay, minute, sourceRefs);
        if (!replayAircraft.isEmpty()) {
            return replayAircraft;
        }
        return defaultAircraftFleet(scenario, dynamics, sourceRefs);
    }

    private List<SimulatedAircraft> defaultAircraftFleet(SimulationScenario scenario, SimulationDynamicsSnapshot dynamics, List<String> sourceRefs) {
        return Arrays.asList(
                aircraft("SIM-A1-" + scenario.getId(), "LEAD1", AircraftClass.MILITARY_REFUELING, scenario, dynamics, 0.0, sourceRefs),
                aircraft("SIM-A2-" + scenario.getId(), "TRAIL2", AircraftClass.NARROWBODY, scenario, dynamics, -0.08, sourceRefs),
                aircraft("SIM-A3-" + scenario.getId(), "FLOW3", AircraftClass.REGIONAL_JET, scenario, dynamics, 0.08, sourceRefs));
    }

    private SimulatedAircraft aircraft(String id, String callsign, AircraftClass aircraftClass, SimulationScenario scenario,
                                       SimulationDynamicsSnapshot dynamics, double routeOffset, List<String> sourceRefs) {
        AircraftTrajectoryState trajectory = trajectoryFrom(dynamics, routeOffset);
        return SimulatedAircraft.builder()
                .id(id)
                .callsign(callsign)
                .aircraftClass(aircraftClass)
                .performanceProfile(profileFor(aircraftClass))
                .flightPlan(FlightPlanIntent.builder()
                        .origin("JFK")
                        .destination("DOV")
                        .requestedAltitudeBlock("FL240-FL280")
                        .route(scenario.getRoute())
                        .rationale("Generated from simulation scenario route.")
                        .build())
                .trajectory(trajectory)
                .rerouteAssignment(sourceRefs.stream().anyMatch(ref -> ref.contains("WEATHER") || ref.contains("SIGMET")) ? "REVIEW_DOGLEG" : "NONE")
                .impacted(!sourceRefs.isEmpty())
                .impactedSourceRefs(sourceRefs)
                .build();
    }

    private AircraftTrajectoryState trajectoryFrom(SimulationDynamicsSnapshot dynamics, double routeOffset) {
        SimulationAircraftState aircraft = dynamics == null ? null : dynamics.getAircraft();
        double latitude = aircraft == null ? 30.0 : aircraft.getLatitude() + routeOffset;
        double longitude = aircraft == null ? -150.0 : aircraft.getLongitude() + routeOffset;
        double speed = aircraft == null ? 430.0 : aircraft.getGroundSpeedKnots();
        return AircraftTrajectoryState.builder()
                .latitude(latitude)
                .longitude(longitude)
                .altitudeFeet(aircraft == null ? 25000.0 : aircraft.getAltitudeFeet())
                .groundSpeedKnots(speed)
                .routeProgress(Math.max(0, Math.min(1, (latitude - 30.0) / 1.0)))
                .delaySeconds(speed <= 1 ? 300.0 : 0.0)
                .phase(speed <= 1 ? "HELD" : "ENROUTE")
                .build();
    }

    private AircraftPerformanceProfile profileFor(AircraftClass aircraftClass) {
        return switch (aircraftClass) {
            case HEAVY_JET -> AircraftPerformanceProfile.builder().aircraftClass(aircraftClass).cruiseSpeedKnots(470).climbRateFeetPerMinute(1800).descentRateFeetPerMinute(1600).fuelBurnPoundsPerMinute(210).takeoffDistanceFeet(9500).landingDistanceFeet(6200).minimumRvrFeet(1200).brakingActionPenaltyFactor(1.25).assumptions("Representative heavy jet profile.").build();
            case NARROWBODY -> AircraftPerformanceProfile.builder().aircraftClass(aircraftClass).cruiseSpeedKnots(450).climbRateFeetPerMinute(2200).descentRateFeetPerMinute(1800).fuelBurnPoundsPerMinute(145).takeoffDistanceFeet(7600).landingDistanceFeet(5200).minimumRvrFeet(1000).brakingActionPenaltyFactor(1.18).assumptions("Representative narrowbody profile.").build();
            case REGIONAL_JET -> AircraftPerformanceProfile.builder().aircraftClass(aircraftClass).cruiseSpeedKnots(410).climbRateFeetPerMinute(2000).descentRateFeetPerMinute(1700).fuelBurnPoundsPerMinute(92).takeoffDistanceFeet(6000).landingDistanceFeet(4300).minimumRvrFeet(1200).brakingActionPenaltyFactor(1.15).assumptions("Representative regional jet profile.").build();
            case TURBOPROP -> AircraftPerformanceProfile.builder().aircraftClass(aircraftClass).cruiseSpeedKnots(280).climbRateFeetPerMinute(1400).descentRateFeetPerMinute(1300).fuelBurnPoundsPerMinute(48).takeoffDistanceFeet(3800).landingDistanceFeet(3000).minimumRvrFeet(1600).brakingActionPenaltyFactor(1.10).assumptions("Representative turboprop profile.").build();
            case MILITARY_REFUELING -> AircraftPerformanceProfile.builder().aircraftClass(aircraftClass).cruiseSpeedKnots(430).climbRateFeetPerMinute(1500).descentRateFeetPerMinute(1500).fuelBurnPoundsPerMinute(180).takeoffDistanceFeet(8400).landingDistanceFeet(5600).minimumRvrFeet(1200).brakingActionPenaltyFactor(1.22).assumptions("Representative refueling aircraft profile.").build();
            default -> AircraftPerformanceProfile.builder().aircraftClass(AircraftClass.GENERIC).cruiseSpeedKnots(400).climbRateFeetPerMinute(1600).descentRateFeetPerMinute(1500).fuelBurnPoundsPerMinute(100).takeoffDistanceFeet(6500).landingDistanceFeet(4500).minimumRvrFeet(1200).brakingActionPenaltyFactor(1.15).assumptions("Generic performance profile.").build();
        };
    }

    private AirportOpsTimeline airportOpsFor(SimulationDynamicsSnapshot dynamics) {
        SimulationAirportSurfaceState surface = dynamics == null ? null : dynamics.getAirportSurface();
        return AirportOpsTimeline.builder()
                .airportId(surface == null ? "KJFK" : surface.getAirportId())
                .sourceMode("LOCAL_FIXTURE")
                .runwayStates(List.of(RunwayOpsState.builder()
                        .airportId(surface == null ? "KJFK" : surface.getAirportId())
                        .runwayId(surface == null ? "04R" : surface.getRunwayId())
                        .runwayLengthFeet(surface == null ? 8400 : surface.getRunwayLengthFeet())
                        .runwayVisualRangeFeet(surface == null ? 6000 : surface.getRunwayVisualRangeFeet())
                        .brakingAction(surface == null ? "NOT_REPORTED" : surface.getBrakingAction())
                        .smgcsActive(surface != null && surface.isSmgcsActive())
                        .lowVisibilityProcedureActive(surface != null && surface.isLowVisibilityProcedureActive())
                        .queueDepth(surface == null ? 0 : surface.getDepartureQueueDepth())
                        .surfaceDelaySeconds(surface == null ? 0 : surface.getSurfaceDelaySeconds())
                        .departureRatePerHour(surface != null && surface.getSurfaceDelaySeconds() > 0 ? 8 : 24)
                        .arrivalRatePerHour(24)
                        .build()))
                .surfaceProcedures(List.of(SurfaceProcedureState.builder()
                        .procedureFamily("LOW_VISIBILITY")
                        .active(surface != null && surface.isLowVisibilityProcedureActive())
                        .terminologyAmbiguity(surface != null && surface.isTerminologyAmbiguity())
                        .localProcedureName("SMGCS/local low visibility procedure")
                        .requestedPhraseology("LVO/LVP/SMGCS")
                        .confirmationStatus(surface != null && surface.isTerminologyAmbiguity() ? "CONFIRMATION_REQUIRED" : "SIMULATED")
                        .rationale(surface == null ? null : surface.getRationale())
                        .build()))
                .assumptions("Airport ops timeline is generated from simulated surface state.")
                .build();
    }

    private SectorWorkloadModel sectorFor(SimulationDynamicsSnapshot dynamics) {
        SimulationSectorWorkloadState sector = dynamics == null ? null : dynamics.getSectorWorkload();
        int active = sector == null ? 12 : sector.getActiveAircraft();
        int baseline = sector == null ? 20 : sector.getBaselineCapacity();
        double ratio = baseline <= 0 ? 0 : active / (double) baseline;
        return SectorWorkloadModel.builder()
                .sectorId(sector == null ? "SIM-SECTOR" : sector.getSectorId())
                .activeAircraft(active)
                .baselineCapacity(baseline)
                .workloadRatio(ratio)
                .capacityState(sector == null ? "AVAILABLE" : sector.getCapacityState())
                .controllerPositions(List.of(ControllerPositionState.builder()
                        .positionId("SIM-R")
                        .role("RADAR")
                        .staffed(true)
                        .activeAircraft(active)
                        .handoffQueueDepth(sector == null ? 0 : sector.getHandoffQueueDepth())
                        .coordinationDelaySeconds(sector == null ? 0 : sector.getEstimatedHandoffDelaySeconds())
                        .build()))
                .frequencyCongestion(FrequencyCongestionState.builder()
                        .frequencyId("SIM-FREQ")
                        .utilization(sector == null ? 0.4 : sector.getFrequencyCongestion())
                        .blockedTransmissions((int) Math.max(0, Math.round((ratio - 1.0) * 10)))
                        .averageWaitSeconds(sector == null ? 0 : Math.max(0, sector.getEstimatedHandoffDelaySeconds() / 3))
                        .congestionState(ratio >= 1.2 ? "SATURATED" : ratio >= 1.0 ? "BUSY" : "AVAILABLE")
                        .build())
                .build();
    }

    private WeatherEvolutionTick weatherFor(SimulationStepResult step, SimulationDynamicsSnapshot dynamics, SimulationRunRequest request) {
        SimulationWeatherEvolutionState weather = dynamics == null ? null : dynamics.getWeatherEvolution();
        int members = request.getWeatherEnsembleConfig() == null ? (weather == null ? 16 : weather.getEnsembleMemberCount()) : request.getWeatherEnsembleConfig().getMemberCount();
        double mean = weather == null ? 0.18 : weather.getMeanBlockedProbability();
        List<WeatherEnsembleMember> ensemble = new ArrayList<>();
        for (int i = 0; i < Math.max(1, Math.min(members, 24)); i++) {
            double offset = ((i % 5) - 2) * 0.02;
            ensemble.add(WeatherEnsembleMember.builder()
                    .id("member-" + i)
                    .latitudeOffset(offset)
                    .longitudeOffset(-offset)
                    .blockedProbability(Math.max(0, Math.min(1, mean + offset)))
                    .confidence(Math.max(0.1, step.getConfidence() - Math.abs(offset)))
                    .stormPhase(weather == null ? "BACKGROUND" : weather.getStormPhase())
                    .build());
        }
        return WeatherEvolutionTick.builder()
                .minute(step.getOffsetMinutes())
                .config(WeatherEnsembleConfig.builder()
                        .memberCount(members)
                        .randomSeed(request.getRandomSeed())
                        .movementSpeedKnots(weather == null ? 25 : weather.getMovementSpeedKnots())
                        .forecastConfidence(step.getConfidence())
                        .growthRate(weather == null ? 0.04 : weather.getGrowthRate())
                        .decayRate(weather == null ? 0.02 : weather.getDecayRate())
                        .assumptions("Deterministic seeded ensemble for local simulation.")
                        .build())
                .members(ensemble)
                .meanBlockedProbability(mean)
                .probabilitySpread(weather == null ? 0.06 : weather.getProbabilitySpread())
                .build();
    }

    private List<BehaviorStateMachine> behaviorsFor(SimulationDynamicsSnapshot dynamics) {
        SimulationPilotOperatorState behavior = dynamics == null ? null : dynamics.getPilotOperator();
        String pilotAction = behavior == null ? "CONTINUE_MONITORING" : behavior.getPilotAction();
        String controllerAction = behavior == null ? "MONITOR" : behavior.getControllerAction();
        String operatorAction = behavior == null ? "NO_MUTATION" : behavior.getOperatorAction();
        int delay = behavior == null ? 0 : behavior.getCommunicationDelaySeconds();
        double acceptance = behavior == null ? 0.95 : behavior.getAcceptanceProbability();
        return Arrays.asList(
                BehaviorStateMachine.builder().actor("PILOT").currentState("REVIEWING_GUIDANCE").nextAction(pilotAction).humanApprovalRequired(false).communicationDelaySeconds(delay).acceptanceProbability(acceptance).transitionHistory(List.of("OBSERVE", pilotAction)).build(),
                BehaviorStateMachine.builder().actor("CONTROLLER").currentState("COORDINATING").nextAction(controllerAction).humanApprovalRequired(false).communicationDelaySeconds(delay).acceptanceProbability(acceptance).transitionHistory(List.of("MONITOR", controllerAction)).build(),
                BehaviorStateMachine.builder().actor("OPERATOR").currentState("HUMAN_REVIEW").nextAction(operatorAction).humanApprovalRequired(behavior != null && behavior.isHumanApprovalRequired()).communicationDelaySeconds(delay).acceptanceProbability(acceptance).transitionHistory(List.of("DRAFT", operatorAction)).build());
    }

    private String safetyDossierMarkdown(SimulationCampaignReport report) {
        StringBuilder out = new StringBuilder();
        out.append("# Simulation Safety Dossier\n\n");
        out.append("- Campaign: ").append(report.getId()).append("\n");
        out.append("- Scenarios: ").append(report.getScenarioCount()).append("\n");
        out.append("- Passed expected action checks: ").append(report.getPassedScenarioCount()).append("\n");
        out.append("- False clear: ").append(report.getAggregateKpis().getFalseClearCount()).append("\n");
        out.append("- False block: ").append(report.getAggregateKpis().getFalseBlockCount()).append("\n");
        out.append("- Source refs: ").append(Math.round(report.getAggregateKpis().getSourceRefPreservationRate() * 100)).append("%\n\n");
        out.append("## Non-Certification Warning\n\n");
        out.append("Airspace simulation output is advisory prototype evidence only. It is not an FAA-qualified FSTD, cockpit display, dispatch release, or ATC clearance authority.\n\n");
        out.append("## Runs\n\n");
        for (SimulationRunResult run : report.getRuns()) {
            out.append("- ").append(run.getScenarioId()).append(": ").append(run.getFinalAction()).append(" / expected ").append(run.getExpectedFinalAction()).append(" / replay ").append(run.getReplayBundle().getId()).append("\n");
        }
        return out.toString();
    }

    private String normalizeId(String value) {
        if (blank(value)) return "scenario";
        return value.toLowerCase(Locale.US).replaceAll("[^a-z0-9]+", "-").replaceAll("(^-|-$)", "");
    }

    private String sha256(String value) {
        try {
            MessageDigest digest = MessageDigest.getInstance("SHA-256");
            byte[] hash = digest.digest(String.valueOf(value).getBytes(StandardCharsets.UTF_8));
            StringBuilder out = new StringBuilder();
            for (byte b : hash) out.append(String.format("%02x", b));
            return out.toString();
        } catch (NoSuchAlgorithmException e) {
            throw new IllegalStateException("SHA-256 unavailable", e);
        }
    }

    private void injectEvent(String missionId, String reservationId, SimulationEvent event, String actor) {
        if ("OPERATOR".equalsIgnoreCase(event.getFamily())) {
            return;
        }
        ProductDtos.MessageRequest message = new ProductDtos.MessageRequest();
        message.setMissionId(missionId);
        message.setReservationId(reservationId);
        message.setFamily(event.getFamily());
        message.setDirection("INBOUND");
        message.setSubject(event.getLabel());
        message.setRawText(event.getPayload());
        message.setActor(actor);
        productService.sendMessage(message);

        ProductDtos.FeedIngestRequest feed = new ProductDtos.FeedIngestRequest();
        feed.setSourceId("simulation-" + event.getFamily().toLowerCase(Locale.US));
        feed.setType(feedType(event.getFamily()));
        feed.setRawPayload(event.getPayload());
        feed.setRequired(false);
        productService.ingestFeed(feed);
    }

    private SimulationEvent eventAt(List<SimulationEvent> events, int minute) {
        return events.stream()
                .filter(event -> event.getOffsetMinutes() == minute)
                .findFirst()
                .orElse(null);
    }

    private SimulationEvent clockTickEvent(int minute, SimulationEvent lastSourceEvent, String previousAction) {
        List<String> refs = lastSourceEvent == null
                ? List.of()
                : List.of(lastSourceEvent.getFamily() + ":" + lastSourceEvent.getId());
        return SimulationEvent.builder()
                .id("clock-" + minute)
                .offsetMinutes(minute)
                .family("CLOCK")
                .eventType("TIMESTEP")
                .label("Minute-by-minute simulation tick")
                .payload("Simulation clock advanced with no new source artifact.")
                .expectedAction(previousAction)
                .sourceRefs(refs)
                .build();
    }

    private String feedType(String family) {
        String upper = family == null ? "" : family.toUpperCase(Locale.US);
        if (upper.contains("WEATHER") || upper.contains("SIGMET") || upper.contains("AIRMET") || upper.contains("METAR") || upper.contains("TAF")) {
            return "WEATHER";
        }
        if (upper.contains("PIREP")) {
            return "PIREP";
        }
        if (upper.contains("NOTAM") || upper.contains("FDC") || upper.contains("DOM")) {
            return "NOTAM";
        }
        return "USNS";
    }

    private String actionForStep(SimulationScenario scenario,
                                 SimulationEvent event,
                                 ProductDtos.RouteImpactSummary routeImpact,
                                 ProductDtos.MissionWeatherVerdictSummary verdict) {
        if (!blank(event.getExpectedAction())) {
            return event.getExpectedAction();
        }
        String payload = (event.getPayload() == null ? "" : event.getPayload()).toUpperCase(Locale.US);
        if (payload.contains("NO VIABLE") || payload.contains("BLOCK")) return "BLOCKED";
        if (payload.contains("RVR") || payload.contains("SMGCS") || payload.contains("BRAKING") || payload.contains("RWY")) return "DELAY";
        if (payload.contains("PIREP") || payload.contains("/TB SEV") || payload.contains("SEV TURB")) return "CAUTION";
        if (payload.contains("ASH") || payload.contains("CONV") || payload.contains("EMBD TS") || payload.contains("TOP FL")) return "REROUTE";
        if (payload.contains("ICE BLW FL180") || payload.contains("NEG")) return "MONITOR";
        if (routeImpact != null && !blank(routeImpact.getAction())) return routeImpact.getAction();
        if (verdict != null && !blank(verdict.getAction())) return verdict.getAction();
        return scenario.getExpectedFinalAction() == null ? "MONITOR" : scenario.getExpectedFinalAction();
    }

    private String recommendedAction(String action, SimulationEvent event) {
        if ("BLOCKED".equals(action)) return "HOLD_OR_REPLAN";
        if ("REROUTE".equals(action)) return "REVIEW_ALTERNATE_CORRIDOR";
        if ("DELAY".equals(action)) return "CONFIRM_PROCEDURE_STATE";
        if ("CAUTION".equals(action)) return "COORDINATE_AND_REVIEW_ALTITUDE";
        if ("MONITOR".equals(action)) return "MONITOR_WITH_SOURCE_REFS";
        return "CONTINUE_REVIEW";
    }

    private double confidenceFor(String action, SimulationEvent event, ProductDtos.RouteImpactSummary routeImpact) {
        double base = routeImpact == null ? 0.72 : Math.max(0.55, routeImpact.getConfidence());
        if ("BLOCKED".equals(action)) return Math.max(base, 0.88);
        if ("REROUTE".equals(action)) return Math.max(base, 0.80);
        if ("DELAY".equals(action)) return 0.76;
        if ("CAUTION".equals(action)) return 0.74;
        if (event.getPayload() != null && event.getPayload().toUpperCase(Locale.US).contains("STALE")) return 0.50;
        return base;
    }

    private List<String> sourceRefs(SimulationEvent event,
                                    ProductDtos.RouteImpactSummary routeImpact,
                                    ProductDtos.MissionWeatherVerdictSummary verdict) {
        List<String> refs = new ArrayList<>();
        if (event.getSourceRefs() == null || event.getSourceRefs().isEmpty()) {
            refs.add(event.getFamily() + ":" + event.getId());
        } else {
            refs.addAll(event.getSourceRefs());
        }
        if (routeImpact != null) refs.addAll(routeImpact.getSourceRefs());
        if (verdict != null) {
            for (ProductDtos.WeatherSourceSummary source : verdict.getSources()) {
                refs.add(source.getFamily() + ":" + source.getId());
            }
        }
        return distinct(refs);
    }

    private List<String> affectedDeltas(String previousAction,
                                        String action,
                                        List<String> previousSources,
                                        List<String> sourceRefs,
                                        SimulationEvent event) {
        List<String> deltas = new ArrayList<>();
        if (!action.equals(previousAction)) {
            deltas.add("Mission verdict changed " + previousAction + " -> " + action + " after " + event.getLabel());
        }
        for (String ref : sourceRefs) {
            if (!previousSources.contains(ref)) {
                deltas.add("New source ref " + ref);
            }
        }
        return deltas;
    }

    private SimulationDynamicsSnapshot dynamicsFor(SimulationScenario scenario,
                                                   SimulationEvent event,
                                                   String action,
                                                   ProductDtos.RouteImpactSummary routeImpact,
                                                   SimulationRunRequest request,
                                                   TrafficReplayBundle trafficReplay,
                                                   NationalDemandCapacityReport nationalReport) {
        Map<String, Double> values = sensitivity(scenario, request);
        double progress = scenario.getEvents().isEmpty()
                ? 0.0
                : Math.min(1.0, event.getOffsetMinutes() / (double) Math.max(1, scenario.getEvents().stream().mapToInt(SimulationEvent::getOffsetMinutes).max().orElse(1)));
        List<Double> routePoint = interpolatedRoutePoint(scenario.getRoute(), progress);
        TrafficReplayPosition replayPosition = primaryReplayPosition(trafficReplay, event.getOffsetMinutes()).orElse(null);
        TrafficReplayAirportDemand airportDemand = trafficReplayAdapter.airportDemandAtMinute(trafficReplay, event.getOffsetMinutes()).orElse(null);
        TrafficReplaySectorDemand sectorDemand = trafficReplayAdapter.sectorDemandAtMinute(trafficReplay, event.getOffsetMinutes()).orElse(null);
        NationalDemandCapacitySnapshot nationalSnapshot = nationalDemandCapacitySimulator.snapshotAtMinute(nationalReport, event.getOffsetMinutes());
        List<TrafficManagementInitiative> activeTmis = trafficReplayAdapter.activeTmisAtMinute(trafficReplay, event.getOffsetMinutes());
        List<TmiRecommendationModel> tmiRecommendations = trafficReplayAdapter.recommendationsAtMinute(trafficReplay, event.getOffsetMinutes(), airportDemand, sectorDemand, action);
        int activeTmiCount = activeTmis.size();
        double cruiseAltitude = routePoint.size() > 2 ? routePoint.get(2) : 25000.0;
        double rvr = payloadNumber(event.getPayload(), "RVR", 6000);
        boolean surfaceScenario = event.getPayload() != null && event.getPayload().toUpperCase(Locale.US).matches(".*(RVR|SMGCS|BRAKING|RWY|SN|SLUSH|R\\d{2}[LRC]?/[0-9]{3,5}FT).*");
        double runwayLength = surfaceScenario ? 8400.0 : 11000.0;
        double requiredRunway = aircraftRunwayRequired(action, rvr, surfaceScenario);
        double movement = values.getOrDefault("weatherMovementSpeedKt", 25.0);
        double confidence = values.getOrDefault("forecastConfidence", 0.82);
        double communicationDelay = values.getOrDefault("communicationDelaySeconds", 30.0);
        int replayActiveAircraft = trafficReplayAdapter.activeFlightCountAtMinute(trafficReplay, event.getOffsetMinutes());
        int activeAircraft = sectorDemand != null && sectorDemand.getActiveAircraft() > 0
                ? sectorDemand.getActiveAircraft()
                : replayActiveAircraft > 0 ? replayActiveAircraft : Math.max(4, (int) Math.round(12 + event.getOffsetMinutes() * 1.5 + ("REROUTE".equals(action) || "BLOCKED".equals(action) ? 8 : 0)));
        int baselineCapacity = sectorDemand != null && sectorDemand.getBaselineCapacity() > 0 ? sectorDemand.getBaselineCapacity() : 20;
        double workloadRatio = activeAircraft / (double) baselineCapacity;
        int handoffDelay = sectorDemand != null && sectorDemand.getEstimatedHandoffDelaySeconds() > 0
                ? sectorDemand.getEstimatedHandoffDelaySeconds()
                : (int) Math.round(Math.max(0, workloadRatio - 0.8) * 180 + communicationDelay);
        double blockedProbability = routeImpact == null
                ? actionProbability(action)
                : Math.max(actionProbability(action), routeImpact.getConfidence() * actionProbability(routeImpact.getAction()));
        int ensembleMembers = (int) Math.max(8, Math.round(values.getOrDefault("ensembleMembers", 16.0)));
        String trafficSourceMode = trafficReplay == null || blank(trafficReplay.getSourceMode()) ? "LOCAL_FIXTURE_REPLAY" : trafficReplay.getSourceMode();

        return SimulationDynamicsSnapshot.builder()
                .minute(event.getOffsetMinutes())
                .aircraft(SimulationAircraftState.builder()
                        .aircraftId(replayPosition == null ? "SIM-ACFT-" + scenario.getId() : replayPosition.getFlightId())
                        .latitude(replayPosition == null ? routePoint.get(0) : replayPosition.getLatitude())
                        .longitude(replayPosition == null ? routePoint.get(1) : replayPosition.getLongitude())
                        .altitudeFeet(replayPosition == null ? cruiseAltitude : replayPosition.getAltitudeFeet())
                        .groundSpeedKnots("DELAY".equals(action) || "BLOCKED".equals(action) ? 0.0 : replayPosition == null ? 430.0 : replayPosition.getGroundSpeedKnots())
                        .climbRateFeetPerMinute(event.getOffsetMinutes() < 5 ? 1200.0 : 0.0)
                        .fuelRemainingPounds(Math.max(6000.0, 48000.0 - event.getOffsetMinutes() * fuelBurnPerMinute(action)))
                        .estimatedFuelBurnPounds(event.getOffsetMinutes() * fuelBurnPerMinute(action))
                        .runwayRequiredFeet(requiredRunway)
                        .runwayAvailableFeet(runwayLength)
                        .takeoffPerformanceAcceptable(requiredRunway <= runwayLength && rvr >= 1000)
                        .performancePhase(surfaceScenario ? "DEPARTURE_SURFACE_REVIEW" : "ENROUTE_SIMULATION")
                        .rationale("Pragmatic performance approximation using route progress, action, RVR, and braking context.")
                        .build())
                .airportSurface(SimulationAirportSurfaceState.builder()
                        .airportId(airportDemand == null || blank(airportDemand.getAirportId()) ? "KJFK" : airportDemand.getAirportId())
                        .runwayId(airportDemand == null || blank(airportDemand.getRunwayConfiguration()) ? "04R" : airportDemand.getRunwayConfiguration().split("/")[0])
                        .runwayVisualRangeFeet((int) Math.round(rvr))
                        .runwayLengthFeet(runwayLength)
                        .brakingAction(surfaceScenario && event.getPayload() != null && event.getPayload().toUpperCase(Locale.US).contains("POOR") ? "POOR" : "NOT_REPORTED")
                        .smgcsActive(surfaceScenario && (event.getPayload() == null || !event.getPayload().toUpperCase(Locale.US).contains("VERIFY")))
                        .lowVisibilityProcedureActive(surfaceScenario && rvr < 1200 && (event.getPayload() == null || !event.getPayload().toUpperCase(Locale.US).contains("VERIFY")))
                        .terminologyAmbiguity(surfaceScenario && event.getPayload() != null && event.getPayload().toUpperCase(Locale.US).matches(".*(SMGCS|LVO|LVP|VERIFY).*"))
                        .departureQueueDepth(airportDemand == null ? (surfaceScenario ? Math.max(1, event.getOffsetMinutes() / 2 + 1) : 0) : airportDemand.getDepartureQueueDepth())
                        .surfaceDelaySeconds(airportDemand == null ? (surfaceScenario ? (int) Math.round(communicationDelay + Math.max(0, 5000 - rvr) / 10.0) : 0) : airportDemand.getAverageDelaySeconds())
                        .rationale(airportDemand == null
                                ? "Airport surface state is simulated from RVR, runway/braking tokens, and procedure ambiguity."
                                : "Airport surface state uses recorded replay demand/capacity snapshot " + airportDemand.getTimestamp() + ".")
                        .build())
                .sectorWorkload(SimulationSectorWorkloadState.builder()
                        .sectorId(sectorDemand == null || blank(sectorDemand.getSectorId()) ? "SIM-SECTOR" : sectorDemand.getSectorId())
                        .activeAircraft(activeAircraft)
                        .baselineCapacity(baselineCapacity)
                        .controllerPositionsStaffed(workloadRatio > 1.0 ? 2 : 1)
                        .handoffQueueDepth(sectorDemand == null ? Math.max(0, activeAircraft - baselineCapacity) : sectorDemand.getHandoffQueueDepth())
                        .estimatedHandoffDelaySeconds(handoffDelay)
                        .frequencyCongestion(sectorDemand == null ? Math.min(1.0, workloadRatio * 0.72) : sectorDemand.getFrequencyUtilization())
                        .workloadRatio(workloadRatio)
                        .capacityState(workloadRatio >= 1.25 ? "SATURATED" : workloadRatio >= 1.0 ? "COMPRESSED" : "AVAILABLE")
                        .rationale(sectorDemand == null
                                ? "Sector workload is a deterministic capacity approximation for campaign comparison."
                                : "Sector workload uses recorded replay demand snapshot " + sectorDemand.getTimestamp() + ".")
                        .build())
                .pilotOperator(SimulationPilotOperatorState.builder()
                        .behaviorModel("DETERMINISTIC_HUMAN_REVIEW")
                        .pilotAction(pilotAction(action))
                        .controllerAction(controllerAction(action))
                        .operatorAction(operatorAction(action))
                        .humanApprovalRequired(!"CLEAR".equals(action) && !"MONITOR".equals(action))
                        .communicationDelaySeconds((int) Math.round(communicationDelay))
                        .acceptanceProbability(acceptanceProbability(action, confidence))
                        .rationale("Human behavior remains advisory and deterministic; no external state is mutated automatically.")
                        .build())
                .weatherEvolution(SimulationWeatherEvolutionState.builder()
                        .evolutionModel("SEEDED_MONTE_CARLO_APPROXIMATION")
                        .ensembleMemberCount(ensembleMembers)
                        .forecastHour(Math.max(0, event.getOffsetMinutes() / 60))
                        .movementSpeedKnots(movement)
                        .growthRate(growthRate(event, action))
                        .decayRate(decayRate(event, action))
                        .meanBlockedProbability(blockedProbability)
                        .probabilitySpread(Math.max(0.03, (1.0 - confidence) * 0.5))
                        .stormPhase(stormPhase(event, action))
                        .rationale("Weather evolution is a deterministic ensemble seam, not calibrated live meteorology.")
                        .build())
                .trafficReplay(SimulationTrafficReplayState.builder()
                        .sourceMode(trafficSourceMode)
                        .replaySourceId(trafficReplay == null ? "simulation-corpus:" + scenario.getId() : trafficReplay.getId())
                        .providerFamily(trafficReplay == null ? "TFMS_LIKE_RECORDED_REPLAY" : trafficReplay.getProviderFamily())
                        .providerReceiptId(trafficReplay == null ? null : trafficReplay.getProviderReceiptId())
                        .authorizationMode(trafficReplay == null ? "LOCAL_FIXTURE_ONLY" : trafficReplay.getAuthorizationMode())
                        .replayedAircraftCount(replayActiveAircraft > 0 ? replayActiveAircraft : activeAircraft)
                        .replayedFlightPlanCount(trafficReplay == null || trafficReplay.getFlightPlans() == null ? Math.max(1, activeAircraft / 2) : trafficReplay.getFlightPlans().size())
                        .replayedPositionCount(trafficReplay == null || trafficReplay.getPositions() == null ? 0 : trafficReplay.getPositions().size())
                        .airportDemandSnapshotCount(trafficReplay == null || trafficReplay.getAirportDemand() == null ? 0 : trafficReplay.getAirportDemand().size())
                        .sectorDemandSnapshotCount(trafficReplay == null || trafficReplay.getSectorDemand() == null ? 0 : trafficReplay.getSectorDemand().size())
                        .activeTrafficManagementInitiativeCount(activeTmiCount)
                        .activeTmiRecommendationCount(tmiRecommendations.size())
                        .activeTmiTypes(activeTmis.stream()
                                .map(trafficReplayAdapter::canonicalTmiType)
                                .map(Enum::name)
                                .distinct()
                                .toList())
                        .liveSwimNasDataUsed("AUTHORIZED_OPERATIONAL".equalsIgnoreCase(trafficSourceMode))
                        .fixtureBacked(!"AUTHORIZED_OPERATIONAL".equalsIgnoreCase(trafficSourceMode))
                        .rationale("Traffic state uses a recorded SWIM/TFMS-like replay bundle; no live provider is required in local mode.")
                        .build())
                .trafficManagementRecommendations(tmiRecommendations)
                .nationalDemandCapacity(nationalSnapshot)
                .assumptions(List.of(
                        "Aircraft performance is approximate and scenario-scoped.",
                        "Sector workload is deterministic fixture math.",
                        "Weather evolution uses seeded ensemble-style fields for explainability.",
                        "Pilot/controller behavior is advisory and human-reviewed."))
                .build();
    }

    private List<String> routeDeltas(ProductDtos.RouteImpactSummary routeImpact, String action, SimulationEvent event) {
        List<String> deltas = new ArrayList<>();
        if (routeImpact != null && routeImpact.getImpactedSegmentCount() > 0) {
            deltas.add(routeImpact.getImpactedSegmentCount() + " impacted segment(s) after " + event.getLabel());
        }
        if (routeImpact != null && !routeImpact.getAvoidanceCandidates().isEmpty()) {
            deltas.add(routeImpact.getAvoidanceCandidates().size() + " avoidance candidate(s) available");
        }
        if ("BLOCKED".equals(action)) {
            deltas.add("No acceptable corridor remains in this scenario step");
        }
        return deltas;
    }

    private SimulationKpiSummary kpis(SimulationScenario scenario,
                                      List<SimulationStepResult> steps,
                                      NationalDemandCapacityReport nationalReport) {
        SimulationStepResult firstGuidance = steps.stream()
                .filter(step -> !"CLEAR".equals(step.getEngineAction()) && !"MONITOR".equals(step.getEngineAction()))
                .findFirst()
                .orElse(steps.isEmpty() ? null : steps.get(0));
        String finalAction = steps.isEmpty() ? "CLEAR" : steps.get(steps.size() - 1).getEngineAction();
        long timeToGuidance = firstGuidance == null ? -1 : firstGuidance.getOffsetMinutes() * 60L;
        long sourceSteps = steps.stream().filter(step -> !step.getSourceRefs().isEmpty()).count();
        long rerouteSteps = steps.stream().filter(step -> step.getRouteImpact() != null && !step.getRouteImpact().getAvoidanceCandidates().isEmpty()).count();
        long replaySteps = steps.stream().filter(step -> !step.getReplayAuditIds().isEmpty()).count();
        int staleDowngrades = (int) steps.stream()
                .filter(step -> "MONITOR".equals(step.getEngineAction()) && step.getInjectedEvent().getPayload() != null
                        && step.getInjectedEvent().getPayload().toUpperCase(Locale.US).contains("STALE"))
                .count();
        double peakWorkload = steps.stream()
                .filter(step -> step.getDynamics() != null && step.getDynamics().getSectorWorkload() != null)
                .mapToDouble(step -> step.getDynamics().getSectorWorkload().getWorkloadRatio())
                .max()
                .orElse(0.0);
        int maxHandoffDelay = steps.stream()
                .filter(step -> step.getDynamics() != null && step.getDynamics().getSectorWorkload() != null)
                .mapToInt(step -> step.getDynamics().getSectorWorkload().getEstimatedHandoffDelaySeconds())
                .max()
                .orElse(0);
        int maxSurfaceDelay = steps.stream()
                .filter(step -> step.getDynamics() != null && step.getDynamics().getAirportSurface() != null)
                .mapToInt(step -> step.getDynamics().getAirportSurface().getSurfaceDelaySeconds())
                .max()
                .orElse(0);
        int maxEnsembleMembers = steps.stream()
                .filter(step -> step.getDynamics() != null && step.getDynamics().getWeatherEvolution() != null)
                .mapToInt(step -> step.getDynamics().getWeatherEvolution().getEnsembleMemberCount())
                .max()
                .orElse(0);
        int trafficAircraft = steps.stream()
                .filter(step -> step.getDynamics() != null && step.getDynamics().getTrafficReplay() != null)
                .mapToInt(step -> step.getDynamics().getTrafficReplay().getReplayedAircraftCount())
                .max()
                .orElse(0);
        int nationalFlightCount = nationalReport == null ? 0 : nationalReport.getFlightCount();
        int overloadedAirports = nationalReport == null ? 0 : nationalReport.getPeakOverloadedAirportCount();
        int overloadedSectors = nationalReport == null ? 0 : nationalReport.getPeakOverloadedSectorCount();
        double peakAirportRatio = nationalReport == null ? 0 : nationalReport.getPeakAirportDemandCapacityRatio();
        double peakNationalSectorRatio = nationalReport == null ? 0 : nationalReport.getPeakSectorDemandCapacityRatio();
        int nationalRecommendationCount = nationalReport == null ? 0 : nationalReport.getTotalTmiRecommendationCount();
        OutcomeMetricInputs outcomes = outcomeMetricInputs(steps);
        return SimulationKpiSummary.builder()
                .timeToGuidanceSeconds(timeToGuidance)
                .falseClearCount("CLEAR".equals(finalAction) && !actionMatches(finalAction, scenario.getExpectedFinalAction()) ? 1 : 0)
                .falseBlockCount("BLOCKED".equals(finalAction) && !actionMatches(finalAction, scenario.getExpectedFinalAction()) ? 1 : 0)
                .sourceRefPreservationRate(steps.isEmpty() ? 1.0 : sourceSteps / (double) steps.size())
                .rerouteFoundRate(steps.isEmpty() ? 0.0 : rerouteSteps / (double) steps.size())
                .staleDataDowngradeCount(staleDowngrades)
                .coordinationDraftLatencySeconds(firstGuidance == null ? -1 : Math.max(0, firstGuidance.getOffsetMinutes() * 60L + 30))
                .pilotBriefAvailable(steps.stream().anyMatch(step -> step.getPilotBrief() != null))
                .replayVerificationPassRate(steps.isEmpty() ? 1.0 : replaySteps / (double) steps.size())
                .minuteStepCount(steps.size())
                .aircraftStateUpdateCount((int) steps.stream().filter(step -> step.getDynamics() != null && step.getDynamics().getAircraft() != null).count())
                .peakSectorWorkloadRatio(peakWorkload)
                .maxHandoffDelaySeconds(maxHandoffDelay)
                .maxSurfaceDelaySeconds(maxSurfaceDelay)
                .stochasticEnsembleMemberCount(maxEnsembleMembers)
                .behaviorDecisionCount((int) steps.stream().filter(step -> step.getDynamics() != null && step.getDynamics().getPilotOperator() != null).count())
                .trafficReplayAircraftCount(trafficAircraft)
                .nationalFlightCount(nationalFlightCount)
                .overloadedAirportCount(overloadedAirports)
                .overloadedSectorCount(overloadedSectors)
                .peakAirportDemandCapacityRatio(peakAirportRatio)
                .peakNationalSectorDemandCapacityRatio(peakNationalSectorRatio)
                .nationalTmiRecommendationCount(nationalRecommendationCount)
                .simulatedDayCount(1)
                .baselineDelayMinutes(outcomes.baselineDelayMinutes())
                .mitigatedDelayMinutes(outcomes.mitigatedDelayMinutes())
                .delayMinutesSaved(outcomes.delayMinutesSaved())
                .rerouteMiles(outcomes.rerouteMiles())
                .additionalFuelPounds(outcomes.additionalFuelPounds())
                .holdingFuelSavedPounds(outcomes.holdingFuelSavedPounds())
                .fuelImpactPounds(outcomes.fuelImpactPounds())
                .sectorOverloadAvoidedCount(outcomes.sectorOverloadAvoidedCount())
                .sourceRefCompletenessRate(outcomes.sourceRefCompletenessRate())
                .operatorTimeToDecisionSeconds(operatorTimeToDecisionSeconds(steps, timeToGuidance))
                .diagnostics(actionMatches(finalAction, scenario.getExpectedFinalAction())
                        ? List.of("Final action matched expected scenario guidance.")
                        : List.of("Final action " + finalAction + " did not match expected " + scenario.getExpectedFinalAction()))
                .build();
    }

    private SimulationKpiSummary aggregateKpis(List<SimulationRunResult> runs) {
        if (runs.isEmpty()) {
            return SimulationKpiSummary.builder().timeToGuidanceSeconds(-1).diagnostics(List.of("No campaign runs executed.")).build();
        }
        double count = runs.size();
        return SimulationKpiSummary.builder()
                .timeToGuidanceSeconds(Math.round(runs.stream().mapToLong(run -> run.getKpiSummary().getTimeToGuidanceSeconds()).average().orElse(-1)))
                .falseClearCount(runs.stream().mapToInt(run -> run.getKpiSummary().getFalseClearCount()).sum())
                .falseBlockCount(runs.stream().mapToInt(run -> run.getKpiSummary().getFalseBlockCount()).sum())
                .sourceRefPreservationRate(runs.stream().mapToDouble(run -> run.getKpiSummary().getSourceRefPreservationRate()).sum() / count)
                .rerouteFoundRate(runs.stream().mapToDouble(run -> run.getKpiSummary().getRerouteFoundRate()).sum() / count)
                .staleDataDowngradeCount(runs.stream().mapToInt(run -> run.getKpiSummary().getStaleDataDowngradeCount()).sum())
                .coordinationDraftLatencySeconds(Math.round(runs.stream().mapToLong(run -> run.getKpiSummary().getCoordinationDraftLatencySeconds()).average().orElse(-1)))
                .pilotBriefAvailable(runs.stream().allMatch(run -> run.getKpiSummary().isPilotBriefAvailable()))
                .replayVerificationPassRate(runs.stream().mapToDouble(run -> run.getKpiSummary().getReplayVerificationPassRate()).sum() / count)
                .minuteStepCount(runs.stream().mapToInt(run -> run.getKpiSummary().getMinuteStepCount()).sum())
                .aircraftStateUpdateCount(runs.stream().mapToInt(run -> run.getKpiSummary().getAircraftStateUpdateCount()).sum())
                .peakSectorWorkloadRatio(runs.stream().mapToDouble(run -> run.getKpiSummary().getPeakSectorWorkloadRatio()).max().orElse(0.0))
                .maxHandoffDelaySeconds(runs.stream().mapToInt(run -> run.getKpiSummary().getMaxHandoffDelaySeconds()).max().orElse(0))
                .maxSurfaceDelaySeconds(runs.stream().mapToInt(run -> run.getKpiSummary().getMaxSurfaceDelaySeconds()).max().orElse(0))
                .stochasticEnsembleMemberCount(runs.stream().mapToInt(run -> run.getKpiSummary().getStochasticEnsembleMemberCount()).max().orElse(0))
                .behaviorDecisionCount(runs.stream().mapToInt(run -> run.getKpiSummary().getBehaviorDecisionCount()).sum())
                .trafficReplayAircraftCount(runs.stream().mapToInt(run -> run.getKpiSummary().getTrafficReplayAircraftCount()).max().orElse(0))
                .nationalFlightCount(runs.stream().mapToInt(run -> run.getKpiSummary().getNationalFlightCount()).max().orElse(0))
                .overloadedAirportCount(runs.stream().mapToInt(run -> run.getKpiSummary().getOverloadedAirportCount()).max().orElse(0))
                .overloadedSectorCount(runs.stream().mapToInt(run -> run.getKpiSummary().getOverloadedSectorCount()).max().orElse(0))
                .peakAirportDemandCapacityRatio(runs.stream().mapToDouble(run -> run.getKpiSummary().getPeakAirportDemandCapacityRatio()).max().orElse(0.0))
                .peakNationalSectorDemandCapacityRatio(runs.stream().mapToDouble(run -> run.getKpiSummary().getPeakNationalSectorDemandCapacityRatio()).max().orElse(0.0))
                .nationalTmiRecommendationCount(runs.stream().mapToInt(run -> run.getKpiSummary().getNationalTmiRecommendationCount()).sum())
                .simulatedDayCount((int) runs.stream().map(SimulationRunResult::getScenarioId).count())
                .baselineDelayMinutes(runs.stream().mapToDouble(run -> run.getKpiSummary().getBaselineDelayMinutes()).sum())
                .mitigatedDelayMinutes(runs.stream().mapToDouble(run -> run.getKpiSummary().getMitigatedDelayMinutes()).sum())
                .delayMinutesSaved(runs.stream().mapToDouble(run -> run.getKpiSummary().getDelayMinutesSaved()).sum())
                .rerouteMiles(runs.stream().mapToDouble(run -> run.getKpiSummary().getRerouteMiles()).sum())
                .additionalFuelPounds(runs.stream().mapToDouble(run -> run.getKpiSummary().getAdditionalFuelPounds()).sum())
                .holdingFuelSavedPounds(runs.stream().mapToDouble(run -> run.getKpiSummary().getHoldingFuelSavedPounds()).sum())
                .fuelImpactPounds(runs.stream().mapToDouble(run -> run.getKpiSummary().getFuelImpactPounds()).sum())
                .sectorOverloadAvoidedCount(runs.stream().mapToInt(run -> run.getKpiSummary().getSectorOverloadAvoidedCount()).sum())
                .sourceRefCompletenessRate(runs.stream().mapToDouble(run -> run.getKpiSummary().getSourceRefCompletenessRate()).sum() / count)
                .operatorTimeToDecisionSeconds(Math.round(runs.stream().mapToLong(run -> run.getKpiSummary().getOperatorTimeToDecisionSeconds()).average().orElse(-1)))
                .diagnostics(List.of("Aggregate KPIs are arithmetic summaries across deterministic local scenarios."))
                .build();
    }

    private OutcomeMetricInputs outcomeMetricInputs(List<SimulationStepResult> steps) {
        if (steps == null || steps.isEmpty()) {
            return new OutcomeMetricInputs(0, 0, 0, 0, 0, 0, 0, 0, 1.0);
        }
        double baselineDelay = 0.0;
        double mitigatedDelay = 0.0;
        double rerouteMiles = 0.0;
        double additionalFuel = 0.0;
        int sectorOverloadAvoided = 0;
        int sourceComplete = 0;
        for (SimulationStepResult step : steps) {
            double stepBaseline = baselineDelayForStep(step);
            baselineDelay += stepBaseline;
            ProductDtos.RouteCostEstimateSummary bestCost = bestCandidateCost(step).orElse(null);
            if (bestCost != null) {
                mitigatedDelay += Math.min(stepBaseline, Math.max(0.0, bestCost.getAdditionalMinutes()));
                rerouteMiles += Math.max(0.0, bestCost.getAdditionalDistanceNm());
                additionalFuel += Math.max(0.0, bestCost.getAdditionalFuelLb());
            } else {
                mitigatedDelay += stepBaseline;
            }
            if (step.getDynamics() != null
                    && step.getDynamics().getSectorWorkload() != null
                    && step.getDynamics().getSectorWorkload().getWorkloadRatio() > 1.0
                    && step.getDynamics().getTrafficManagementRecommendations() != null
                    && !step.getDynamics().getTrafficManagementRecommendations().isEmpty()) {
                sectorOverloadAvoided++;
            }
            if (!step.getSourceRefs().isEmpty()
                    || (step.getRouteImpact() != null && step.getRouteImpact().getSourceRefs() != null && !step.getRouteImpact().getSourceRefs().isEmpty())) {
                sourceComplete++;
            }
        }
        double saved = Math.max(0.0, baselineDelay - mitigatedDelay);
        double holdingFuelSaved = saved * 120.0;
        double fuelImpact = additionalFuel - holdingFuelSaved;
        double sourceCompleteness = sourceComplete / (double) steps.size();
        return new OutcomeMetricInputs(baselineDelay, mitigatedDelay, saved, rerouteMiles, additionalFuel, holdingFuelSaved, fuelImpact, sectorOverloadAvoided, sourceCompleteness);
    }

    private double baselineDelayForStep(SimulationStepResult step) {
        if (step == null) return 0.0;
        double delay = actionDelayPenaltyMinutes(step.getEngineAction());
        if (step.getDynamics() != null && step.getDynamics().getAirportSurface() != null) {
            delay += Math.max(0, step.getDynamics().getAirportSurface().getSurfaceDelaySeconds()) / 60.0;
        }
        if (step.getDynamics() != null && step.getDynamics().getSectorWorkload() != null) {
            delay += Math.max(0, step.getDynamics().getSectorWorkload().getEstimatedHandoffDelaySeconds()) / 60.0;
        }
        return delay;
    }

    private double actionDelayPenaltyMinutes(String action) {
        if ("BLOCKED".equals(action)) return 30.0;
        if ("REROUTE".equals(action) || "AVOID".equals(action)) return 15.0;
        if ("DELAY".equals(action)) return 10.0;
        if ("CAUTION".equals(action) || "ALTITUDE_CHANGE".equals(action)) return 3.0;
        return 0.0;
    }

    private Optional<ProductDtos.RouteCostEstimateSummary> bestCandidateCost(SimulationStepResult step) {
        if (step == null || step.getRouteImpact() == null || step.getRouteImpact().getCandidateComparisons() == null) {
            return Optional.empty();
        }
        return step.getRouteImpact().getCandidateComparisons().stream()
                .filter(candidate -> candidate.getCost() != null)
                .min(Comparator.comparingInt(this::residualConstraintCount)
                        .thenComparingDouble(candidate -> candidate.getCost().getAdditionalMinutes())
                        .thenComparingDouble(candidate -> candidate.getCost().getAdditionalFuelLb()))
                .map(ProductDtos.RouteCandidateComparisonSummary::getCost);
    }

    private int residualConstraintCount(ProductDtos.RouteCandidateComparisonSummary candidate) {
        return candidate == null || candidate.getResidualConstraints() == null ? 0 : candidate.getResidualConstraints().size();
    }

    private long operatorTimeToDecisionSeconds(List<SimulationStepResult> steps, long timeToGuidanceSeconds) {
        if (steps == null || steps.isEmpty() || timeToGuidanceSeconds < 0) {
            return -1;
        }
        SimulationStepResult firstGuidance = steps.stream()
                .filter(step -> !"CLEAR".equals(step.getEngineAction()) && !"MONITOR".equals(step.getEngineAction()))
                .findFirst()
                .orElse(steps.get(0));
        int communicationDelay = firstGuidance.getDynamics() == null || firstGuidance.getDynamics().getPilotOperator() == null
                ? 30
                : Math.max(0, firstGuidance.getDynamics().getPilotOperator().getCommunicationDelaySeconds());
        return timeToGuidanceSeconds + communicationDelay + 30L;
    }

    private double estimatedTmiDelaySavings(TfmCommandCenterSummary board) {
        if (board == null || board.getProposedTmis() == null) return 0.0;
        return board.getProposedTmis().stream()
                .mapToDouble(proposal -> Math.max(0, proposal.getExpectedDelayMinutes()) * tmiMitigationFactor(proposal.getType()))
                .sum();
    }

    private double tmiMitigationFactor(String type) {
        String upper = type == null ? "" : type.toUpperCase(Locale.US);
        if (upper.contains("GROUND_STOP") || upper.contains("GDP") || upper.contains("AFP")) return 0.42;
        if (upper.contains("REROUTE") || upper.contains("SECTOR_CAPACITY")) return 0.35;
        if (upper.contains("MILES") || upper.contains("MINUTES") || upper.contains("DEPARTURE") || upper.contains("ARRIVAL")) return 0.25;
        return 0.18;
    }

    private int estimatedOverloadAvoided(TfmCommandCenterSummary board) {
        if (board == null || board.getImpactTotals() == null) return 0;
        int overloaded = board.getImpactTotals().getOverloadedSectorCount() + board.getImpactTotals().getOverloadedAirportCount();
        int relevant = (int) board.getProposedTmis().stream()
                .filter(proposal -> {
                    String type = String.valueOf(proposal.getType());
                    return type.contains("SECTOR") || type.contains("AFP") || type.contains("GDP") || type.contains("REROUTE") || type.contains("METERING");
                })
                .count();
        return Math.min(overloaded, relevant);
    }

    private List<OutcomeMetricSummary> outcomeMetricCards(OutcomeMetricsReport report) {
        return List.of(
                outcomeMetric("delay-minutes-saved", "Delay Minutes Saved", report.getDelayMinutesSaved(), "min", report.getDelayMinutesSaved() > 0 ? "IMPROVING" : "NO_GAIN", "Estimated delay avoided versus local baseline delay.", report.getSourceRefs()),
                outcomeMetric("fuel-impact", "Fuel Impact", report.getFuelImpactPounds(), "lb", report.getFuelImpactPounds() <= 0 ? "SAVED_OR_NEUTRAL" : "ADDED_FUEL", "Additional reroute fuel minus estimated holding/delay fuel avoided.", report.getSourceRefs()),
                outcomeMetric("reroute-miles", "Reroute Miles", report.getRerouteMiles(), "NM", report.getRerouteMiles() > 0 ? "REROUTE_USED" : "NO_REROUTE", "Additional nautical miles from selected local route alternatives.", report.getSourceRefs()),
                outcomeMetric("sector-overload-avoided", "Sector Overload Avoided", report.getSectorOverloadAvoidedCount(), "count", report.getSectorOverloadAvoidedCount() > 0 ? "MITIGATED" : "NONE", "Estimated overloaded airport/sector resources with matching TMI or reroute mitigation.", report.getSourceRefs()),
                outcomeMetric("false-clear", "False Clear", report.getFalseClearCount(), "count", report.getFalseClearCount() == 0 ? "PASS" : "REVIEW", "Scenario-labeled cases where a clear outcome contradicted expected guidance.", report.getSourceRefs()),
                outcomeMetric("false-block", "False Block", report.getFalseBlockCount(), "count", report.getFalseBlockCount() == 0 ? "PASS" : "REVIEW", "Scenario-labeled cases where a block outcome contradicted expected guidance.", report.getSourceRefs()),
                outcomeMetric("source-ref-completeness", "Source Ref Completeness", report.getSourceRefCompletenessRate(), "ratio", report.getSourceRefCompletenessRate() >= 0.95 ? "PASS" : "REVIEW", "Share of evaluated steps or board rows retaining source references.", report.getSourceRefs()),
                outcomeMetric("operator-time-to-decision", "Operator Time To Decision", report.getOperatorTimeToDecisionSeconds(), "sec", report.getOperatorTimeToDecisionSeconds() >= 0 && report.getOperatorTimeToDecisionSeconds() <= 300 ? "FAST" : "REVIEW", "Estimated guidance plus communication/review latency.", report.getSourceRefs())
        );
    }

    private OutcomeMetricSummary outcomeMetric(String id, String label, double value, String unit, String status, String rationale, List<String> sourceRefs) {
        return OutcomeMetricSummary.builder()
                .id(id)
                .label(label)
                .value(value)
                .unit(unit)
                .status(status)
                .rationale(rationale)
                .sourceRefs(sourceRefs == null ? List.of() : sourceRefs.stream().limit(12).toList())
                .build();
    }

    private NationalDemandCapacitySnapshot selectedTfmSnapshot(NationalDemandCapacityReport report, Integer focusMinute) {
        if (report == null || report.getSnapshots() == null || report.getSnapshots().isEmpty()) {
            return null;
        }
        if (focusMinute != null && focusMinute >= 0) {
            return nationalDemandCapacitySimulator.snapshotAtMinute(report, focusMinute);
        }
        return report.getSnapshots().stream()
                .max(Comparator
                        .comparingInt((NationalDemandCapacitySnapshot snapshot) -> snapshot.getOverloadedAirportCount() + snapshot.getOverloadedSectorCount())
                        .thenComparingInt(NationalDemandCapacitySnapshot::getTotalExpectedDelayMinutes)
                        .thenComparingDouble(snapshot -> Math.max(snapshot.getMaxAirportDemandCapacityRatio(), snapshot.getMaxSectorDemandCapacityRatio())))
                .orElse(report.getSnapshots().get(0));
    }

    private List<TfmAirportDemandSummary> tfmAirportDemand(TrafficReplayBundle replay, int minute, int maxRows) {
        if (replay == null || replay.getAirportDemand() == null) {
            return List.of();
        }
        Map<String, TrafficReplayAirportDemand> latest = new LinkedHashMap<>();
        replay.getAirportDemand().stream()
                .filter(item -> !blank(item.getAirportId()))
                .filter(item -> item.getOffsetMinutes() <= minute)
                .sorted(Comparator.comparingInt(TrafficReplayAirportDemand::getOffsetMinutes))
                .forEach(item -> latest.put(item.getAirportId(), item));
        if (latest.isEmpty()) {
            replay.getAirportDemand().stream()
                    .filter(item -> !blank(item.getAirportId()))
                    .sorted(Comparator.comparingInt(TrafficReplayAirportDemand::getOffsetMinutes))
                    .forEach(item -> latest.putIfAbsent(item.getAirportId(), item));
        }
        return latest.values().stream()
                .map(this::tfmAirportSummary)
                .sorted(Comparator
                        .comparingDouble(TfmAirportDemandSummary::getDemandCapacityRatio).reversed()
                        .thenComparing(TfmAirportDemandSummary::getAirportId, Comparator.nullsLast(String::compareTo)))
                .limit(Math.max(1, maxRows))
                .toList();
    }

    private TfmAirportDemandSummary tfmAirportSummary(TrafficReplayAirportDemand item) {
        double ratio = airportDemandRatio(item);
        return TfmAirportDemandSummary.builder()
                .airportId(item.getAirportId())
                .offsetMinutes(item.getOffsetMinutes())
                .timestamp(valueOr(item.getTimestamp(), "T+" + item.getOffsetMinutes() + "M"))
                .departureDemandPerHour(item.getDepartureDemandPerHour())
                .arrivalDemandPerHour(item.getArrivalDemandPerHour())
                .departureCapacityPerHour(item.getDepartureCapacityPerHour())
                .arrivalCapacityPerHour(item.getArrivalCapacityPerHour())
                .demandCapacityRatio(ratio)
                .departureQueueDepth(item.getDepartureQueueDepth())
                .arrivalQueueDepth(item.getArrivalQueueDepth())
                .averageDelayMinutes(Math.max(0, item.getAverageDelaySeconds() / 60))
                .runwayConfiguration(item.getRunwayConfiguration())
                .status(loadStatus(ratio, item.getDepartureQueueDepth() + item.getArrivalQueueDepth()))
                .sourceRefs(item.getSourceRefs() == null ? List.of() : item.getSourceRefs())
                .build();
    }

    private List<TfmSectorLoadSummary> tfmSectorLoad(TrafficReplayBundle replay, int minute, int maxRows) {
        if (replay == null || replay.getSectorDemand() == null) {
            return List.of();
        }
        Map<String, TrafficReplaySectorDemand> latest = new LinkedHashMap<>();
        replay.getSectorDemand().stream()
                .filter(item -> !blank(item.getSectorId()))
                .filter(item -> item.getOffsetMinutes() <= minute)
                .sorted(Comparator.comparingInt(TrafficReplaySectorDemand::getOffsetMinutes))
                .forEach(item -> latest.put(item.getSectorId(), item));
        if (latest.isEmpty()) {
            replay.getSectorDemand().stream()
                    .filter(item -> !blank(item.getSectorId()))
                    .sorted(Comparator.comparingInt(TrafficReplaySectorDemand::getOffsetMinutes))
                    .forEach(item -> latest.putIfAbsent(item.getSectorId(), item));
        }
        return latest.values().stream()
                .map(this::tfmSectorSummary)
                .sorted(Comparator
                        .comparingDouble(TfmSectorLoadSummary::getWorkloadRatio).reversed()
                        .thenComparing(TfmSectorLoadSummary::getSectorId, Comparator.nullsLast(String::compareTo)))
                .limit(Math.max(1, maxRows))
                .toList();
    }

    private TfmSectorLoadSummary tfmSectorSummary(TrafficReplaySectorDemand item) {
        double ratio = item.getBaselineCapacity() <= 0 ? 0.0 : item.getActiveAircraft() / (double) item.getBaselineCapacity();
        return TfmSectorLoadSummary.builder()
                .sectorId(item.getSectorId())
                .offsetMinutes(item.getOffsetMinutes())
                .timestamp(valueOr(item.getTimestamp(), "T+" + item.getOffsetMinutes() + "M"))
                .activeAircraft(item.getActiveAircraft())
                .baselineCapacity(item.getBaselineCapacity())
                .workloadRatio(ratio)
                .handoffQueueDepth(item.getHandoffQueueDepth())
                .frequencyUtilization(item.getFrequencyUtilization())
                .estimatedHandoffDelaySeconds(item.getEstimatedHandoffDelaySeconds())
                .status(loadStatus(ratio, item.getHandoffQueueDepth()))
                .sourceRefs(item.getSourceRefs() == null ? List.of() : item.getSourceRefs())
                .build();
    }

    private List<TfmActiveConstraintSummary> tfmActiveConstraints(TrafficReplayBundle replay, int minute, int maxRows) {
        return trafficReplayAdapter.activeTmisAtMinute(replay, minute).stream()
                .map(this::tfmActiveConstraint)
                .sorted(Comparator
                        .comparingInt(TfmActiveConstraintSummary::getExpectedDelayMinutes).reversed()
                        .thenComparing(TfmActiveConstraintSummary::getId, Comparator.nullsLast(String::compareTo)))
                .limit(Math.max(1, maxRows))
                .toList();
    }

    private TfmActiveConstraintSummary tfmActiveConstraint(TrafficManagementInitiative tmi) {
        TrafficManagementInitiativeType type = trafficReplayAdapter.canonicalTmiType(tmi);
        List<String> affected = tmi.getAffectedFlightIds() == null ? List.of() : tmi.getAffectedFlightIds();
        return TfmActiveConstraintSummary.builder()
                .id(valueOr(tmi.getId(), "tmi-" + type.name()))
                .type(type.name())
                .status(valueOr(tmi.getStatus(), "ACTIVE_OR_PROPOSED"))
                .scope(valueOr(tmi.getScope(), "TRAFFIC_FLOW"))
                .targetResourceId(valueOr(tmi.getTargetResourceId(), tmi.getConstraintId()))
                .reason(tmi.getReason())
                .startOffsetMinutes(tmi.getStartOffsetMinutes())
                .endOffsetMinutes(tmi.getEndOffsetMinutes())
                .expectedDelayMinutes(tmi.getExpectedDelayMinutes())
                .confidence(tmi.getConfidence() <= 0 ? 0.70 : tmi.getConfidence())
                .affectedFlightIds(affected)
                .affectedFlightCount(affected.size())
                .sourceRefs(tmi.getSourceRefs() == null ? List.of() : tmi.getSourceRefs())
                .build();
    }

    private List<TfmProposedTmiSummary> tfmProposedTmis(TrafficReplayBundle replay,
                                                        NationalDemandCapacitySnapshot snapshot,
                                                        int minute,
                                                        int maxRows) {
        Map<String, TfmProposedTmiSummary> proposals = new LinkedHashMap<>();
        trafficReplayAdapter.activeTmisAtMinute(replay, minute).forEach(tmi -> {
            TmiRecommendationModel recommendation = tmi.getRecommendation() == null ? recommendationFromActiveConstraint(tmi, minute) : tmi.getRecommendation();
            TfmProposedTmiSummary summary = tfmProposedTmi(recommendation, valueOr(tmi.getStatus(), "PROPOSED"));
            proposals.put(valueOr(summary.getId(), "proposal-" + proposals.size()), summary);
        });
        if (snapshot != null && snapshot.getRecommendations() != null) {
            for (TmiRecommendationModel recommendation : snapshot.getRecommendations()) {
                TfmProposedTmiSummary summary = tfmProposedTmi(recommendation, "RECOMMENDED");
                proposals.putIfAbsent(valueOr(summary.getId(), "proposal-" + proposals.size()), summary);
            }
        }
        return proposals.values().stream()
                .sorted(Comparator
                        .comparingInt(TfmProposedTmiSummary::getExpectedDelayMinutes).reversed()
                        .thenComparingDouble(TfmProposedTmiSummary::getConfidence).reversed())
                .limit(Math.max(1, maxRows))
                .toList();
    }

    private TmiRecommendationModel recommendationFromActiveConstraint(TrafficManagementInitiative tmi, int minute) {
        TrafficManagementInitiativeType type = trafficReplayAdapter.canonicalTmiType(tmi);
        return TmiRecommendationModel.builder()
                .id("rec-" + valueOr(tmi.getId(), "tmi-" + minute))
                .recommendedType(type == TrafficManagementInitiativeType.UNKNOWN ? TrafficManagementInitiativeType.LOCAL_REVIEW : type)
                .action("REVIEW_" + (type == TrafficManagementInitiativeType.UNKNOWN ? "TMI" : type.name()))
                .targetResourceType(valueOr(tmi.getScope(), "TRAFFIC_FLOW"))
                .targetResourceId(valueOr(tmi.getTargetResourceId(), tmi.getConstraintId()))
                .trigger(valueOr(tmi.getReason(), "Active TMI at T+" + minute + "M"))
                .severity(tmi.getExpectedDelayMinutes() >= 30 ? "HIGH" : "MEDIUM")
                .expectedDelayMinutes(Math.max(0, tmi.getExpectedDelayMinutes()))
                .confidence(tmi.getConfidence() <= 0 ? 0.70 : tmi.getConfidence())
                .rationale("Active replay constraint should be reviewed in the TFM board.")
                .sourceTmiIds(List.of(valueOr(tmi.getId(), "tmi-" + minute)))
                .affectedFlightIds(tmi.getAffectedFlightIds() == null ? List.of() : tmi.getAffectedFlightIds())
                .sourceRefs(tmi.getSourceRefs() == null ? List.of() : tmi.getSourceRefs())
                .build();
    }

    private TfmProposedTmiSummary tfmProposedTmi(TmiRecommendationModel recommendation, String status) {
        TrafficManagementInitiativeType type = recommendation == null || recommendation.getRecommendedType() == null
                ? TrafficManagementInitiativeType.LOCAL_REVIEW
                : recommendation.getRecommendedType();
        List<String> affected = recommendation == null || recommendation.getAffectedFlightIds() == null ? List.of() : recommendation.getAffectedFlightIds();
        return TfmProposedTmiSummary.builder()
                .id(recommendation == null ? null : recommendation.getId())
                .type(type.name())
                .action(recommendation == null ? "REVIEW_TMI" : recommendation.getAction())
                .targetResourceType(recommendation == null ? null : recommendation.getTargetResourceType())
                .targetResourceId(recommendation == null ? null : recommendation.getTargetResourceId())
                .status(status)
                .severity(recommendation == null ? "REVIEW" : recommendation.getSeverity())
                .expectedDelayMinutes(recommendation == null ? 0 : Math.max(0, recommendation.getExpectedDelayMinutes()))
                .confidence(recommendation == null || recommendation.getConfidence() <= 0 ? 0.70 : recommendation.getConfidence())
                .affectedFlightIds(affected)
                .affectedFlightCount(affected.size())
                .trigger(recommendation == null ? null : recommendation.getTrigger())
                .rationale(recommendation == null ? "Human review required." : recommendation.getRationale())
                .requiresHumanApproval(true)
                .sourceTmiIds(recommendation == null || recommendation.getSourceTmiIds() == null ? List.of() : recommendation.getSourceTmiIds())
                .sourceRefs(recommendation == null || recommendation.getSourceRefs() == null ? List.of() : recommendation.getSourceRefs())
                .build();
    }

    private List<TfmRouteAlternativeSummary> tfmRouteAlternatives(List<TfmActiveConstraintSummary> activeConstraints,
                                                                  List<TfmProposedTmiSummary> proposedTmis,
                                                                  TrafficReplayBundle replay,
                                                                  int minute,
                                                                  int maxRows) {
        Map<String, TfmRouteAlternativeSummary> alternatives = new LinkedHashMap<>();
        for (TrafficManagementInitiative tmi : trafficReplayAdapter.activeTmisAtMinute(replay, minute)) {
            if (tmi.getRerouteAdvisory() == null) {
                continue;
            }
            RerouteAdvisoryModel advisory = tmi.getRerouteAdvisory();
            List<String> refs = new ArrayList<>();
            refs.addAll(tmi.getSourceRefs() == null ? List.of() : tmi.getSourceRefs());
            refs.addAll(advisory.getSourceRefs() == null ? List.of() : advisory.getSourceRefs());
            List<String> affected = advisory.getAffectedFlightIds() == null || advisory.getAffectedFlightIds().isEmpty()
                    ? tmi.getAffectedFlightIds()
                    : advisory.getAffectedFlightIds();
            String id = valueOr(advisory.getAdvisoryId(), valueOr(tmi.getId(), "route-alt-" + alternatives.size()));
            alternatives.put(id, TfmRouteAlternativeSummary.builder()
                    .id(id)
                    .routeName(advisory.getRouteName())
                    .routeText(advisory.getRouteText())
                    .advisoryType(advisory.getAdvisoryType())
                    .required(advisory.isRequired())
                    .targetResourceId(tmi.getTargetResourceId())
                    .reason(valueOr(advisory.getReason(), tmi.getReason()))
                    .startOffsetMinutes(advisory.getStartOffsetMinutes())
                    .endOffsetMinutes(advisory.getEndOffsetMinutes())
                    .expectedDelayMinutes(Math.max(0, tmi.getExpectedDelayMinutes()))
                    .confidence(tmi.getConfidence() <= 0 ? 0.70 : tmi.getConfidence())
                    .affectedFlightIds(affected == null ? List.of() : affected)
                    .affectedFlightCount(affected == null ? 0 : affected.size())
                    .routePoints(advisory.getRoutePoints() == null ? List.of() : advisory.getRoutePoints())
                    .residualRisk(tmi.getConfidence() >= 0.85 ? "LOW_IF_ACCEPTED" : "REVIEW_CAPACITY_AND_WEATHER_BEFORE_DELIVERY")
                    .sourceRefs(distinct(refs))
                    .build());
        }
        for (TfmProposedTmiSummary proposal : proposedTmis) {
            if (!String.valueOf(proposal.getType()).contains("REROUTE")) {
                continue;
            }
            String id = valueOr(proposal.getId(), "proposal-route-alt-" + alternatives.size());
            alternatives.putIfAbsent(id, TfmRouteAlternativeSummary.builder()
                    .id(id)
                    .routeName(valueOr(proposal.getTargetResourceId(), "TFM route review"))
                    .routeText(valueOr(proposal.getRationale(), "Review reroute advisory against active constraints."))
                    .advisoryType(proposal.getType())
                    .required("REQUIRED_REROUTE".equals(proposal.getType()))
                    .targetResourceId(proposal.getTargetResourceId())
                    .reason(proposal.getTrigger())
                    .startOffsetMinutes(minute)
                    .endOffsetMinutes(minute)
                    .expectedDelayMinutes(proposal.getExpectedDelayMinutes())
                    .confidence(proposal.getConfidence())
                    .affectedFlightIds(proposal.getAffectedFlightIds())
                    .affectedFlightCount(proposal.getAffectedFlightCount())
                    .residualRisk(proposal.getConfidence() >= 0.80 ? "LOW_IF_ACCEPTED" : "HUMAN_REVIEW_REQUIRED")
                    .sourceRefs(proposal.getSourceRefs())
                    .build());
        }
        if (alternatives.isEmpty() && activeConstraints.stream().anyMatch(item -> "SECTOR_CAPACITY".equals(item.getType()) || "AFP".equals(item.getType()))) {
            TfmActiveConstraintSummary first = activeConstraints.get(0);
            String id = "route-review-" + valueOr(first.getTargetResourceId(), "network");
            alternatives.put(id, TfmRouteAlternativeSummary.builder()
                    .id(id)
                    .routeName("Flow reroute review")
                    .routeText("Evaluate reroute advisories, MIT/MINIT, or AFP alternatives around " + valueOr(first.getTargetResourceId(), "the constrained resource") + ".")
                    .advisoryType("LOCAL_REVIEW")
                    .targetResourceId(first.getTargetResourceId())
                    .reason(first.getReason())
                    .startOffsetMinutes(first.getStartOffsetMinutes())
                    .endOffsetMinutes(first.getEndOffsetMinutes())
                    .expectedDelayMinutes(first.getExpectedDelayMinutes())
                    .confidence(first.getConfidence())
                    .affectedFlightIds(first.getAffectedFlightIds())
                    .affectedFlightCount(first.getAffectedFlightCount())
                    .residualRisk("REQUIRES_OPERATOR_COORDINATION")
                    .sourceRefs(first.getSourceRefs())
                    .build());
        }
        return alternatives.values().stream()
                .limit(Math.max(1, maxRows))
                .toList();
    }

    private double airportDemandRatio(TrafficReplayAirportDemand item) {
        int demand = Math.max(item.getDepartureDemandPerHour(), item.getArrivalDemandPerHour());
        int departureCapacity = item.getDepartureCapacityPerHour() <= 0 ? 1 : item.getDepartureCapacityPerHour();
        int arrivalCapacity = item.getArrivalCapacityPerHour() <= 0 ? 1 : item.getArrivalCapacityPerHour();
        return demand / (double) Math.max(1, Math.min(departureCapacity, arrivalCapacity));
    }

    private String loadStatus(double ratio, int queueDepth) {
        if (ratio >= 1.30) return "SATURATED";
        if (ratio > 1.0) return "COMPRESSED";
        if (queueDepth > 0) return "METERING_REVIEW";
        if (ratio >= 0.85) return "NEAR_CAPACITY";
        return "NOMINAL";
    }

    private record OutcomeMetricInputs(double baselineDelayMinutes,
                                       double mitigatedDelayMinutes,
                                       double delayMinutesSaved,
                                       double rerouteMiles,
                                       double additionalFuelPounds,
                                       double holdingFuelSavedPounds,
                                       double fuelImpactPounds,
                                       int sectorOverloadAvoidedCount,
                                       double sourceRefCompletenessRate) {
    }

    private List<Double> interpolatedRoutePoint(List<List<Double>> route, double progress) {
        if (route == null || route.isEmpty()) {
            return Arrays.asList(30.0, -150.0, 25000.0);
        }
        if (route.size() == 1) {
            return route.get(0);
        }
        double scaled = progress * (route.size() - 1);
        int low = Math.min(route.size() - 2, Math.max(0, (int) Math.floor(scaled)));
        int high = low + 1;
        double fraction = scaled - low;
        List<Double> a = route.get(low);
        List<Double> b = route.get(high);
        return Arrays.asList(
                lerp(a.get(0), b.get(0), fraction),
                lerp(a.get(1), b.get(1), fraction),
                lerp(a.size() > 2 ? a.get(2) : 25000.0, b.size() > 2 ? b.get(2) : 25000.0, fraction));
    }

    private double lerp(double a, double b, double fraction) {
        return a + ((b - a) * fraction);
    }

    private double payloadNumber(String payload, String marker, double fallback) {
        if (payload == null) return fallback;
        String upper = payload.toUpperCase(Locale.US);
        if ("RVR".equals(marker)) {
            Matcher runwayVisualRange = Pattern.compile("R\\d{2}[LRC]?/([0-9]{3,5})FT").matcher(upper);
            if (runwayVisualRange.find()) {
                return Double.parseDouble(runwayVisualRange.group(1));
            }
            Matcher plainRvr = Pattern.compile("RVR\\s*([0-9]{3,5})").matcher(upper);
            if (plainRvr.find()) {
                return Double.parseDouble(plainRvr.group(1));
            }
        }
        int idx = upper.indexOf(marker);
        if (idx < 0) return fallback;
        String tail = upper.substring(idx + marker.length()).replaceAll("^[^0-9]+", "");
        String digits = tail.replaceAll("^([0-9]+).*$", "$1");
        if (digits.isBlank() || digits.equals(tail) && !tail.matches("^[0-9]+.*")) return fallback;
        try {
            return Double.parseDouble(digits);
        } catch (NumberFormatException ex) {
            return fallback;
        }
    }

    private double aircraftRunwayRequired(String action, double rvr, boolean surfaceScenario) {
        double base = surfaceScenario ? 7600.0 : 6900.0;
        if ("DELAY".equals(action) || "BLOCKED".equals(action)) base += 1200.0;
        if (rvr < 1200) base += 900.0;
        return base;
    }

    private double fuelBurnPerMinute(String action) {
        if ("DELAY".equals(action) || "BLOCKED".equals(action)) return 120.0;
        if ("REROUTE".equals(action)) return 180.0;
        return 150.0;
    }

    private double actionProbability(String action) {
        if ("BLOCKED".equals(action)) return 0.92;
        if ("REROUTE".equals(action)) return 0.72;
        if ("DELAY".equals(action)) return 0.64;
        if ("CAUTION".equals(action)) return 0.48;
        return 0.18;
    }

    private double growthRate(SimulationEvent event, String action) {
        String payload = event.getPayload() == null ? "" : event.getPayload().toUpperCase(Locale.US);
        if (payload.contains("GROW") || "REROUTE".equals(action) || "BLOCKED".equals(action)) return 0.18;
        return 0.04;
    }

    private double decayRate(SimulationEvent event, String action) {
        String payload = event.getPayload() == null ? "" : event.getPayload().toUpperCase(Locale.US);
        if (payload.contains("DECAY") || "MONITOR".equals(action)) return 0.10;
        return 0.02;
    }

    private String stormPhase(SimulationEvent event, String action) {
        String payload = event.getPayload() == null ? "" : event.getPayload().toUpperCase(Locale.US);
        if (payload.contains("GROW")) return "GROWING";
        if (payload.contains("DECAY")) return "DECAYING";
        if ("BLOCKED".equals(action) || "REROUTE".equals(action)) return "MATURE";
        return "BACKGROUND";
    }

    private String pilotAction(String action) {
        if ("BLOCKED".equals(action)) return "REJECT_DEPARTURE_OR_REQUEST_REPLAN";
        if ("REROUTE".equals(action)) return "REQUEST_DEVIATION";
        if ("DELAY".equals(action)) return "ASK_ATC_FOR_PROCEDURE_CONFIRMATION";
        if ("CAUTION".equals(action)) return "REQUEST_ALTITUDE_OR_RIDE_REPORTS";
        return "CONTINUE_MONITORING";
    }

    private String controllerAction(String action) {
        if ("BLOCKED".equals(action)) return "STOP_RELEASE_AND_COORDINATE_FLOW";
        if ("REROUTE".equals(action)) return "COORDINATE_REROUTE";
        if ("DELAY".equals(action)) return "CONFIRM_SURFACE_OR_LOCAL_PROCEDURE_STATE";
        if ("CAUTION".equals(action)) return "SOLICIT_PIREPS_AND_ADVISE";
        return "MONITOR";
    }

    private String operatorAction(String action) {
        if ("BLOCKED".equals(action)) return "OPEN_BLOCKAGE_REVIEW";
        if ("REROUTE".equals(action)) return "COMPARE_CANDIDATE_CORRIDORS";
        if ("DELAY".equals(action)) return "HOLD_FOR_CONFIRMATION";
        if ("CAUTION".equals(action)) return "REVIEW_SOURCE_ARTIFACTS";
        return "NO_MUTATION";
    }

    private double acceptanceProbability(String action, double confidence) {
        if ("BLOCKED".equals(action)) return Math.max(0.85, confidence);
        if ("REROUTE".equals(action)) return Math.max(0.65, confidence - 0.05);
        if ("DELAY".equals(action)) return 0.78;
        if ("CAUTION".equals(action)) return 0.62;
        return 0.95;
    }

    private AirspaceFeatureCollection featuresFor(SimulationScenario scenario, SimulationEvent event, String action, String missionId, String reservationId) {
        AirspaceFeatureCollection collection = new AirspaceFeatureCollection();
        collection.getFeatures().add(routeFeature(scenario, event, missionId, reservationId));
        collection.getFeatures().add(eventFeature(scenario, event, action, missionId, reservationId));
        return collection;
    }

    private AirspaceFeature routeFeature(SimulationScenario scenario, SimulationEvent event, String missionId, String reservationId) {
        Map<String, Object> props = new LinkedHashMap<>();
        props.put("displayLayer", "flight-paths");
        props.put("sourceFamily", "CARF/ALTRV");
        props.put("featureKind", "simulation-route");
        props.put("missionId", missionId);
        props.put("reservationId", reservationId);
        props.put("label", scenario.getName() + " route");
        props.put("simulationStep", event.getOffsetMinutes());
        return AirspaceFeature.builder()
                .id("simulation-route-" + event.getId())
                .geometry(lineGeometry(scenario.getRoute()))
                .properties(props)
                .build();
    }

    private AirspaceFeature eventFeature(SimulationScenario scenario, SimulationEvent event, String action, String missionId, String reservationId) {
        Map<String, Object> props = new LinkedHashMap<>();
        props.put("displayLayer", layerFor(event.getFamily(), action));
        props.put("sourceFamily", event.getFamily());
        props.put("featureKind", "simulation-event");
        props.put("missionId", missionId);
        props.put("reservationId", reservationId);
        props.put("label", event.getLabel());
        props.put("action", action);
        props.put("severity", severityFor(action));
        props.put("confidence", confidenceFor(action, event, null));
        props.put("sourceRefs", List.of(event.getFamily() + ":" + event.getId()));
        props.put("simulationStep", event.getOffsetMinutes());
        return AirspaceFeature.builder()
                .id("simulation-event-" + event.getId())
                .geometry(pointGeometry(pointForEvent(scenario, event)))
                .properties(props)
                .build();
    }

    private String layerFor(String family, String action) {
        String upper = family == null ? "" : family.toUpperCase(Locale.US);
        if (upper.contains("NOTAM")) return "notams";
        if (upper.contains("PIREP")) return "pireps";
        if ("BLOCKED".equals(action) || "REROUTE".equals(action)) return "route-impacts";
        if (upper.contains("WEATHER") || upper.contains("SIGMET") || upper.contains("AIRMET") || upper.contains("METAR")) return "weather";
        return "other";
    }

    private String severityFor(String action) {
        if ("BLOCKED".equals(action)) return "EXTREME";
        if ("REROUTE".equals(action) || "DELAY".equals(action)) return "SEVERE";
        if ("CAUTION".equals(action)) return "MODERATE";
        return "LOW";
    }

    private AirspaceGeometry lineGeometry(List<List<Double>> route) {
        List<double[]> coordinates = new ArrayList<>();
        for (List<Double> point : route) {
            if (point.size() >= 2) coordinates.add(new double[]{point.get(1), point.get(0), point.size() > 2 ? point.get(2) : 0.0});
        }
        return AirspaceGeometry.builder().type("LineString").coordinates(coordinates).build();
    }

    private AirspaceGeometry pointGeometry(List<Double> point) {
        return AirspaceGeometry.builder().type("Point").coordinates(Arrays.asList(point.get(1), point.get(0))).build();
    }

    private List<Double> pointForEvent(SimulationScenario scenario, SimulationEvent event) {
        int index = Math.min(Math.max(0, event.getOffsetMinutes() / 5), scenario.getRoute().size() - 1);
        List<Double> base = scenario.getRoute().get(index);
        return Arrays.asList(base.get(0) + 0.12, base.get(1) + 0.12);
    }

    private Map<String, Double> sensitivity(SimulationScenario scenario, SimulationRunRequest request) {
        Map<String, Double> values = new LinkedHashMap<>(scenario.getSensitivityDefaults());
        if (request.isIncludeSensitivity() && request.getSensitivityOverrides() != null) {
            values.putAll(request.getSensitivityOverrides());
        }
        return values;
    }

    private boolean actionMatches(String actual, String expected) {
        if (blank(expected)) return true;
        return expected.toUpperCase(Locale.US).contains(String.valueOf(actual).toUpperCase(Locale.US));
    }

    private SimulationScenario scenario(String id) {
        if (blank(id)) {
            return scenarios.get(0);
        }
        return scenarios.stream()
                .filter(item -> item.getId().equals(id))
                .findFirst()
                .orElseThrow(() -> new IllegalArgumentException("Unknown simulation scenario: " + id));
    }

    private List<String> distinct(List<String> values) {
        return values.stream().filter(value -> value != null && !value.isBlank()).distinct().toList();
    }

    private String valueOr(String value, String fallback) {
        return blank(value) ? fallback : value;
    }

    private boolean blank(String value) {
        return value == null || value.trim().isEmpty();
    }

    private static List<SimulationScenario> canonicalScenarios() {
        List<List<Double>> route = Arrays.asList(
                Arrays.asList(30.0, -150.5, 24000.0),
                Arrays.asList(30.5, -149.8, 25000.0),
                Arrays.asList(31.0, -149.0, 26000.0));
        return List.of(
                scenario("low-vis-rvr-smgcs", "Low Visibility Procedure Ambiguity", "Low Visibility Procedure Ambiguity", "DELAY_OR_CAUTION",
                        "RVR 1000 feet plus terminology mismatch drives confirm-procedure-state guidance.",
                        route,
                        event("evt-lowvis-1", 0, "METAR", "METAR KJFK 201200Z 00000KT 1/8SM R04R/1000FT FG VV002 A2992", "Visibility drops below local review threshold", "MONITOR"),
                        event("evt-lowvis-2", 5, "NOTAM", "!JFK 06/020 JFK RWY 04R RVR OUT OF SERVICE SMGCS LOW VIS PROC VERIFY", "RVR/SMGCS procedure ambiguity", "DELAY")),
                scenario("oceanic-altrv-convection", "Oceanic ALTRV Moving Convection", "Weather Avoidance", "REROUTE_OR_BLOCKED",
                        "A moving convective cell crosses an oceanic reservation and should create reroute guidance.",
                        route,
                        event("evt-conv-1", 0, "WEATHER", "SIGMET CONV VALID 201200/201600 FROM 3000N15000W TO 3100N14900W EMBD TS TOP FL450 MOV E 25KT", "Convective SIGMET enters route", "REROUTE"),
                        event("evt-conv-2", 15, "PIREP", "UA /OV 3030N14950W/TM 1215/FL250/TP B738/TB MOD/RM DEVIATING EAST OF CELL", "Pilot reports deviation", "REROUTE")),
                scenario("refuel-icing-altitude-separated", "Refueling Corridor Icing Altitude Separation", "Altitude-Separated Icing", "MONITOR_OR_CLEAR_ABOVE_LAYER",
                        "Icing below route altitude should stay visible without causing false blockage.",
                        route,
                        event("evt-ice-1", 0, "AIRMET", "AIRMET ZULU VALID 201200/201800 FROM 3000N15000W TO 3100N14900W MOD ICE BLW FL180", "Icing below corridor", "MONITOR"),
                        event("evt-ice-2", 10, "PIREP", "UA /OV 3030N14950W/TM 1210/FL250/TP KC46/IC NEG/RM ABOVE ICING LAYER", "Negative icing PIREP at route altitude", "MONITOR")),
                scenario("pirep-safety-override", "Severe PIREP Safety Override", "PIREP Safety Override", "CAUTION_OR_ALTITUDE_CHANGE",
                        "An urgent severe PIREP should elevate guidance even when the forecast is mild.",
                        route,
                        event("evt-pirep-1", 0, "TAF", "TAF KJFK 201130Z 2012/2118 P6SM SCT030", "Mild terminal forecast", "MONITOR"),
                        event("evt-pirep-2", 6, "PIREP", "UUA /OV 3030N14950W/TM 1206/FL250/TP B738/TB SEV/RM ABRUPT ALTITUDE CHANGE", "Urgent severe turbulence PIREP", "CAUTION")),
                scenario("volcanic-ash-oceanic", "Volcanic Ash Oceanic Reroute", "Weather Avoidance", "REROUTE_OR_BLOCKED",
                        "Volcanic ash in oceanic routing produces reroute review with residual risk.",
                        route,
                        event("evt-ash-1", 0, "WEATHER", "SIGMET VA VALID 201200/201800 BOUNDED BY 3000N15000W 3100N14900W 3050N14880W VA CLD TOP FL350 MOV NE", "Volcanic ash cloud near route", "REROUTE")),
                scenario("runway-surface-contamination", "Runway Surface Contamination Review", "NOTAM + Weather Compound Constraint", "DELAY_OR_CAUTION",
                        "Runway braking and snow/slush messages should drive operator performance review.",
                        route,
                        event("evt-surface-1", 0, "METAR", "SPECI KJFK 201205Z 02012KT 1/2SM SN R04R/1800FT OVC004 M02/M04 A2988", "Snow and low visibility", "CAUTION"),
                        event("evt-surface-2", 8, "NOTAM", "!JFK 06/021 JFK RWY 04R BRAKING ACTION POOR MU 20 SLUSH", "Poor braking action NOTAM", "DELAY")),
                scenario("sector-capacity-compression", "Sector Capacity Compression", "Weather Avoidance", "REROUTE_OR_BLOCKED",
                        "Several affected missions compress through one sector and should show capacity-impact reroute guidance.",
                        route,
                        event("evt-sector-1", 0, "WEATHER", "CWAP SECTOR VALID 201200/201800 WITHIN 45NM OF 3030N14950W TOP FL430 GROWING", "Growing sector weather constraint", "REROUTE"),
                        event("evt-sector-2", 12, "OPERATOR", "SECTOR_DEMAND ZNY-TEST ACTIVE 32 BASELINE 20", "Sector demand exceeds baseline", "REROUTE")),
                scenario("blocked-no-viable-reroute", "Blocked Route With No Viable Reroute", "Blocked Route With No Viable Reroute", "BLOCKED",
                        "Weather and NOTAM constraints cover the corridor and should return blocked guidance.",
                        route,
                        event("evt-block-1", 0, "WEATHER", "SIGMET BLOCK VALID 201200/202000 BOUNDED BY 2950N15100W 3150N15100W 3150N14800W 2950N14800W EMBD TS TOP FL500", "Broad convective block", "REROUTE"),
                        event("evt-block-2", 5, "NOTAM", "!FDC BLOCK NOTAM AIRSPACE CLSD WI AN AREA 2950N15100W-3150N14800W SFC-FL600 NO VIABLE CORRIDOR", "Airspace closure removes alternate corridor", "BLOCKED")),
                scenario("viable-reroute-residual-risk", "Viable Reroute With Residual Risk", "Viable Reroute With Residual Risk", "REROUTE_WITH_CANDIDATE",
                        "A deterministic dogleg should avoid the primary hazard while retaining residual risk disclosure.",
                        route,
                        event("evt-reroute-1", 0, "WEATHER", "SIGMET REROUTE VALID 201200/201600 FROM 3000N15000W TO 3100N14900W EMBD TS TOP FL430 MOV E 20KT", "Route hazard has room for dogleg", "REROUTE"))
        );
    }

    private static SimulationScenario scenario(String id, String name, String story, String expectedAction, String narrative,
                                               List<List<Double>> route, SimulationEvent... events) {
        String mission = "SIM-" + id.toUpperCase(Locale.US).replace('-', '_');
        return SimulationScenario.builder()
                .id(id)
                .name(name)
                .capabilityStory(story)
                .narrative(narrative)
                .missionNumber(mission)
                .carfAltrv("A. " + mission + "\nB. 1KC135/M\nC. JFK 201200Z\nD. 3000N15000W 3050N14950W 3100N14900W\nE. DOV\nF. FL240-FL280\nG. " + narrative)
                .route(route)
                .events(Arrays.asList(events))
                .expectedFinalAction(expectedAction)
                .expectedGuidance(narrative)
                .expectedSourceFamilies(Arrays.asList("CARF/ALTRV", "NOTAM", "WEATHER", "PIREP"))
                .sensitivityDefaults(Map.of(
                        "weatherMovementSpeedKnots", 25.0,
                        "forecastConfidence", 0.75,
                        "pirepAgeMinutes", 20.0,
                        "routeCorridorBufferNm", 40.0,
                        "altitudeToleranceFeet", 2000.0,
                        "communicationDelaySeconds", 30.0,
                        "sectorDemandLevel", 1.0))
                .build();
    }

    private static SimulationEvent event(String id, int offset, String family, String payload, String label, String expectedAction) {
        return SimulationEvent.builder()
                .id(id)
                .offsetMinutes(offset)
                .family(family)
                .eventType("SCHEDULED_INPUT")
                .label(label)
                .payload(payload)
                .expectedAction(expectedAction)
                .sourceRefs(List.of(family + ":" + id))
                .build();
    }
}
