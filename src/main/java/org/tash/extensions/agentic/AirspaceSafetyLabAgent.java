package org.tash.extensions.agentic;

import jakarta.enterprise.context.ApplicationScoped;
import jakarta.inject.Inject;
import org.tash.extensions.product.application.AirspaceProductService;
import org.tash.extensions.product.application.AirspaceReadinessService;
import org.tash.extensions.product.dto.ProductDtos;
import org.tash.extensions.simulation.HistoricalReplayCalibrationReport;
import org.tash.extensions.simulation.HistoricalReplayDay;
import org.tash.extensions.simulation.NationalDemandCapacityConfig;
import org.tash.extensions.simulation.NationalDemandCapacityReport;
import org.tash.extensions.simulation.OperationalSimulationService;
import org.tash.extensions.simulation.OutcomeMetricsReport;
import org.tash.extensions.simulation.OutcomeMetricsRequest;
import org.tash.extensions.simulation.SimulationAgentReport;
import org.tash.extensions.simulation.SimulationAgentRequest;
import org.tash.extensions.simulation.TfmCommandCenterSummary;
import org.tash.extensions.simulation.TfmProposedTmiSummary;
import org.tash.extensions.simulation.TrafficReplayValidationResult;

import java.time.ZoneOffset;
import java.time.ZonedDateTime;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;

@ApplicationScoped
public class AirspaceSafetyLabAgent {
    private static final List<String> DEFAULT_GUARDS = List.of(
            "ADVISORY_ONLY",
            "NO_EXTERNAL_SEND",
            "NO_OFFICIAL_MUTATION",
            "HUMAN_APPROVAL_REQUIRED",
            "CITED_EVIDENCE_REQUIRED",
            "LOCAL_OR_REPLAY_FIRST");

    @Inject
    OperationalSimulationService simulationService;
    @Inject
    AirspaceProductService productService;
    @Inject
    AirspaceReadinessService readinessService;

    public List<AgentWorkloadDefinition> workloads() {
        return List.of(
                workload("UNSAFE_GUIDANCE_RED_TEAM", "Unsafe Guidance Red-Team", "SAFETY_ASSURANCE",
                        "false-clear, false-block, stale data, missing source refs",
                        "Runs a local red-team pass against simulation/replay outputs and opens review tasks for unsafe guidance candidates."),
                workload("OUTCOME_METRICS_AUDITOR", "Outcome Metrics Auditor", "OUTCOME_METRICS",
                        "delay saved, fuel impact, reroute miles, overload avoided, source-ref completeness",
                        "Audits modeled delay, fuel, reroute, overload, false-clear/block, and operator-latency metrics."),
                workload("SCENARIO_GENERATION", "Scenario Generation Agent", "CORPUS_EXPANSION",
                        "draft scenarios for TFM/weather/NOTAM/PIREP edge cases",
                        "Generates draft scenario bundles that require human review before import."),
                workload("TMI_RECOMMENDATION_AUDITOR", "TMI Recommendation Auditor", "TFM_REVIEW",
                        "GDP, AFP, FCA/FEA, MIT, reroute, metering, ground stop proposals",
                        "Checks proposed TMIs for least-restrictive rationale, affected flights, source refs, and human approval."),
                workload("BRIEF_DELTA_AGENT", "Brief Delta Agent", "PILOT_CONTROLLER_BRIEF",
                        "what changed since last brief",
                        "Summarizes changed hazards, affected missions, source freshness, and coordination needs."),
                workload("REPLAY_INTEGRITY_AGENT", "Replay Integrity Agent", "DATA_QUALITY",
                        "flight plans, ETAs, ETDs, positions, filed routes, airport/sector demand",
                        "Validates recorded SWIM/TFMS-like replay bundles and flags missing or inconsistent replay evidence."),
                workload("HISTORICAL_CALIBRATION_CURATOR", "Historical Calibration Curator", "CALIBRATION",
                        "expected outcomes, false-clear labels, false-block labels, coefficient readiness",
                        "Reviews historical-like replay days for calibration labels, source refs, and uncalibrated coefficients."),
                workload("NATIONAL_DEMAND_STRESS_AGENT", "National Demand Stress Agent", "SCALE_TESTING",
                        "hundreds/thousands of flights across airports and sectors",
                        "Runs local demand/capacity preview and reports overload, TMI, and candidate-load signals."),
                workload("COLLABORATIVE_DECISION_FACILITATOR", "Collaborative Decision Facilitator", "CDM_WORKFLOW",
                        "FAA/airline/operator proposal states, comments, approvals, receipts",
                        "Summarizes the common operating picture, pending proposals, stale providers, and delivery receipts."),
                workload("PROVIDER_FRESHNESS_WATCHER", "Provider Freshness Watcher", "PROVIDER_READINESS",
                        "provider mode, freshness, credentials, consent, egress",
                        "Reviews source mode and freshness posture without activating live external providers.")
        );
    }

    public AgentRunResult run(AgentRunRequest request) {
        String type = normalize(request == null ? null : request.getAgentType());
        return switch (type) {
            case "UNSAFE_GUIDANCE_RED_TEAM" -> unsafeGuidanceRedTeam(request);
            case "OUTCOME_METRICS_AUDITOR" -> outcomeMetricsAuditor(request);
            case "SCENARIO_GENERATION" -> scenarioGeneration(request);
            case "TMI_RECOMMENDATION_AUDITOR" -> tmiRecommendationAuditor(request);
            case "BRIEF_DELTA_AGENT" -> briefDelta(request);
            case "REPLAY_INTEGRITY_AGENT" -> replayIntegrity(request);
            case "HISTORICAL_CALIBRATION_CURATOR" -> historicalCalibrationCurator(request);
            case "NATIONAL_DEMAND_STRESS_AGENT" -> nationalDemandStress(request);
            case "COLLABORATIVE_DECISION_FACILITATOR" -> collaborativeDecisionFacilitator(request);
            case "PROVIDER_FRESHNESS_WATCHER" -> providerFreshnessWatcher(request);
            case "SAFETY_LAB_ALL" -> runAll(request);
            default -> runAll(request);
        };
    }

    public AgentRunResult runAll(AgentRunRequest request) {
        List<AgentRunResult> runs = List.of(
                unsafeGuidanceRedTeam(request),
                outcomeMetricsAuditor(request),
                scenarioGeneration(request),
                tmiRecommendationAuditor(request),
                briefDelta(request),
                replayIntegrity(request),
                historicalCalibrationCurator(request),
                nationalDemandStress(request),
                collaborativeDecisionFacilitator(request),
                providerFreshnessWatcher(request));
        List<AgentFinding> findings = new ArrayList<>();
        List<AgentRecommendation> recommendations = new ArrayList<>();
        List<AgentTask> tasks = new ArrayList<>();
        List<AgentSourceCitation> citations = new ArrayList<>();
        List<AgentEvidenceReceipt> receipts = new ArrayList<>();
        List<String> diagnostics = new ArrayList<>();
        double confidence = 0.0;
        for (AgentRunResult run : runs) {
            findings.addAll(run.getFindings());
            recommendations.addAll(run.getRecommendations());
            tasks.addAll(run.getTasks());
            citations.addAll(run.getCitations());
            receipts.addAll(run.getEvidenceReceipts());
            diagnostics.addAll(run.getDiagnostics());
            confidence += run.getConfidence();
        }
        return result("SAFETY_LAB_ALL",
                "Safety Lab ran " + runs.size() + " autonomous workload(s): " + findings.size()
                        + " finding(s), " + recommendations.size() + " recommendation(s), " + tasks.size() + " review task(s).",
                findings, recommendations, tasks, citations, receipts, diagnostics,
                runs.isEmpty() ? 0.0 : confidence / runs.size(), 0.035);
    }

    private AgentRunResult unsafeGuidanceRedTeam(AgentRunRequest request) {
        SimulationAgentRequest simRequest = new SimulationAgentRequest();
        simRequest.setRunId(value(request == null ? null : request.getRunId(), null));
        simRequest.setCampaignId(value(request == null ? null : request.getCampaignId(), null));
        simRequest.setFocusAreas(List.of("false-clear", "false-block", "source-refs", "stale-data"));
        SimulationAgentReport report = simulation().redTeam(simRequest);
        List<AgentSourceCitation> citations = citations(report.getCitations());
        List<AgentFinding> findings = report.getFindings().stream()
                .map(message -> finding("UNSAFE_GUIDANCE_RED_TEAM", severityForRedTeam(message), message, 0.88, citations))
                .toList();
        return result("UNSAFE_GUIDANCE_RED_TEAM",
                "Unsafe guidance red-team completed with " + findings.size() + " cited finding(s).",
                findings,
                List.of(recommendation("REVIEW_RED_TEAM_FINDINGS", "Review unsafe guidance findings",
                        "Human safety review should disposition every false-clear, false-block, stale-data, and missing-source finding before release.", citations)),
                List.of(task("Review red-team findings", "HIGH", "SAFETY_REVIEW", "/config",
                        "Unsafe guidance findings remain advisory until reviewed by a human safety reviewer.", citations)),
                citations, receipts(citations), report.getPolicyGuards(), 0.88, 0.006);
    }

    private AgentRunResult outcomeMetricsAuditor(AgentRunRequest request) {
        OutcomeMetricsReport report = simulation().outcomeMetrics(OutcomeMetricsRequest.builder()
                .scenarioId(value(request == null ? null : request.getScenarioId(), "oceanic-altrv-convection"))
                .runId(value(request == null ? null : request.getRunId(), null))
                .campaignId(value(request == null ? null : request.getCampaignId(), null))
                .runSimulation(request == null || request.getRunId() == null && request.getCampaignId() == null)
                .includeTfmBoard(true)
                .build());
        List<AgentSourceCitation> citations = citations(report.getSourceRefs());
        List<AgentFinding> findings = new ArrayList<>();
        findings.add(finding("OUTCOME_METRICS_AUDIT", "INFO",
                "Modeled delay saved " + round(report.getDelayMinutesSaved()) + " min; fuel impact "
                        + round(report.getFuelImpactPounds()) + " lb; reroute miles " + round(report.getRerouteMiles()) + " NM.",
                0.9, citations));
        if (report.getFalseClearCount() > 0 || report.getFalseBlockCount() > 0) {
            findings.add(finding("OUTCOME_SAFETY_LABEL_REVIEW", "CRITICAL",
                    "Scenario labels include " + report.getFalseClearCount() + " false-clear and "
                            + report.getFalseBlockCount() + " false-block candidate(s).",
                    0.93, citations));
        }
        if (report.getSourceRefCompletenessRate() < 0.95) {
            findings.add(finding("SOURCE_REF_COMPLETENESS", "HIGH",
                    "Source-ref completeness is " + round(report.getSourceRefCompletenessRate() * 100.0)
                            + "%; release review should require evidence repair.",
                    0.9, citations));
        }
        return result("OUTCOME_METRICS_AUDITOR",
                "Outcome metrics audit covered delay, fuel, reroute, overload, false-clear/block, source refs, and operator latency.",
                findings,
                List.of(recommendation("REVIEW_OUTCOME_METRICS", "Review local outcome metrics",
                        "Treat delay/fuel/overload values as local modeled outcomes until authoritative historical calibration is attached.", citations)),
                List.of(task("Review outcome metric assumptions", "MEDIUM", "SAFETY_REVIEW", "/outcomes",
                        "Confirm local outcome formulas and source-ref coverage before public claims.", citations)),
                citations, receipts(citations), report.getDiagnostics(), 0.9, 0.008);
    }

    private AgentRunResult scenarioGeneration(AgentRunRequest request) {
        SimulationAgentRequest simRequest = new SimulationAgentRequest();
        simRequest.setScenarioType("TFM_WEATHER_REPLAY_EDGE_CASE");
        simRequest.setMissionNumber("SAFETY-LAB");
        simRequest.setCount(2);
        simRequest.setFocusAreas(List.of("GDP", "AFP", "FCA", "MIT", "low-vis", "PIREP", "reroute"));
        SimulationAgentReport report = simulation().generateScenarios(simRequest);
        List<AgentSourceCitation> citations = citations(report.getCitations());
        return result("SCENARIO_GENERATION",
                "Generated " + report.getGeneratedScenarioDrafts().size() + " draft scenario bundle(s); none were imported automatically.",
                List.of(finding("SCENARIO_DRAFTS_CREATED", "INFO",
                        "Draft scenarios are review-only and remain outside the regression corpus until a human imports them.",
                        0.91, citations)),
                List.of(recommendation("REVIEW_DRAFT_SCENARIOS", "Review generated scenario drafts",
                        "Scenario authors should inspect expected outcomes and source families before importing drafts.", citations)),
                List.of(task("Review generated scenario drafts", "MEDIUM", "SIMULATION_AUTHOR", "/simulation/author",
                        "Generated scenarios are draft artifacts and require human review.", citations)),
                citations, receipts(citations), report.getPolicyGuards(), 0.91, 0.007);
    }

    private AgentRunResult tmiRecommendationAuditor(AgentRunRequest request) {
        TfmCommandCenterSummary board = simulation().tfmCommandCenterBoard(null);
        List<AgentSourceCitation> citations = citations(board.getSourceRefs());
        long missingApproval = board.getProposedTmis().stream().filter(tmi -> !tmi.isRequiresHumanApproval()).count();
        TfmProposedTmiSummary highestDelay = board.getProposedTmis().stream()
                .max(Comparator.comparingInt(TfmProposedTmiSummary::getExpectedDelayMinutes))
                .orElse(null);
        List<AgentFinding> findings = new ArrayList<>();
        findings.add(finding("TMI_RECOMMENDATION_AUDIT", missingApproval > 0 ? "HIGH" : "INFO",
                board.getProposedTmis().size() + " proposed TMI(s), " + board.getImpactTotals().getAffectedFlightCount()
                        + " affected flight(s), " + missingApproval + " missing human-approval flag(s).",
                0.89, citations));
        if (highestDelay != null) {
            findings.add(finding("TMI_DELAY_DRIVER", "MEDIUM",
                    "Largest proposed TMI delay is " + highestDelay.getExpectedDelayMinutes() + " min for "
                            + highestDelay.getType() + " at " + highestDelay.getTargetResourceId() + ".",
                    0.84, citations(highestDelay.getSourceRefs())));
        }
        return result("TMI_RECOMMENDATION_AUDITOR",
                "TFM proposal audit checked proposed TMIs for approval, source refs, affected flights, and delay rationale.",
                findings,
                List.of(recommendation("REVIEW_TMI_PROPOSALS", "Review proposed TMIs",
                        "Traffic managers should approve, revise, or reject TMI proposals before any delivery.", citations)),
                List.of(task("Review proposed TMI set", "HIGH", "TRAFFIC_MANAGER", "/tfm",
                        "Proposed TMIs remain local recommendations and require human approval.", citations)),
                citations, receipts(citations), board.getHumanFactorsNotes(), 0.89, 0.007);
    }

    private AgentRunResult briefDelta(AgentRunRequest request) {
        List<ProductDtos.AffectedMissionSummary> affected = product() == null
                ? Collections.emptyList()
                : product().affectedMissions(null, 25);
        ProductDtos.WeatherLiveStatusSummary weather = product() == null ? null : product().liveWeatherStatus();
        List<AgentSourceCitation> citations = affected.isEmpty()
                ? List.of(AgentSupport.citation("BRIEF", "delta", "Local brief delta", "/weather"))
                : affected.stream().flatMap(item -> item.getSourceRefs().stream()).map(AgentSupport::citation).toList();
        if (citations.isEmpty()) citations = List.of(AgentSupport.citation("BRIEF", "delta", "Local brief delta", "/weather"));
        return result("BRIEF_DELTA_AGENT",
                "Brief delta found " + affected.size() + " affected mission(s); provider freshness "
                        + (weather == null ? "unknown" : (weather.isEnabled() ? "enabled" : "fixture-disabled")) + ".",
                List.of(finding("BRIEF_DELTA", affected.isEmpty() ? "INFO" : "MEDIUM",
                        "Changed-mission queue contains " + affected.size() + " affected item(s) for controller/pilot handoff.",
                        0.82, citations)),
                List.of(recommendation("GENERATE_BRIEF_DELTA", "Generate what-changed brief",
                        "Use the delta as a controller/pilot handoff draft; official coordination remains human-approved.", citations)),
                List.of(task("Review what-changed brief", affected.isEmpty() ? "LOW" : "MEDIUM", "PLANNER", "/weather",
                        "Confirm changed hazards, NOTAMs, PIREPs, and route impacts before handoff.", citations)),
                citations, receipts(citations), List.of("Brief delta is based on local workbench state and provider freshness."), 0.82, 0.005);
    }

    private AgentRunResult replayIntegrity(AgentRunRequest request) {
        HistoricalReplayDay day = replayDay(request);
        List<AgentSourceCitation> citations = replayCitations(day);
        if (day == null) {
            return result("REPLAY_INTEGRITY_AGENT",
                    "Replay validation could not find a historical replay day for this request.",
                    List.of(finding("REPLAY_INTEGRITY_MISSING_DAY", "HIGH",
                            "No replay day was available; calibration, replay verification, and source receipts cannot be trusted until a replay bundle is attached.",
                            0.92, citations)),
                    List.of(recommendation("ATTACH_REPLAY_BUNDLE", "Attach replay bundle",
                            "Provide a recorded SWIM/TFMS-like replay day before using this case for calibration or safety claims.", citations)),
                    List.of(task("Attach replay evidence", "HIGH", "DATA_STEWARD", "/simulation",
                            "Replay evidence is required before this case can support evaluation claims.", citations)),
                    citations, receipts(citations), List.of("Historical replay day was not found."), 0.92, 0.005);
        }
        TrafficReplayValidationResult validation = simulation().validateTrafficReplay(day.getTrafficReplay());
        String severity = validation.isAccepted() ? "INFO" : "HIGH";
        return result("REPLAY_INTEGRITY_AGENT",
                "Replay validation inspected " + validation.getFlightPlanCount() + " flight plan(s), "
                        + validation.getPositionCount() + " position(s), and " + validation.getTrafficManagementInitiativeCount() + " TMI(s).",
                List.of(finding("REPLAY_INTEGRITY", severity,
                        validation.isAccepted() ? "Replay bundle is structurally complete for local simulation."
                                : "Replay bundle has structural diagnostics: " + validation.getDiagnostics(),
                        0.9, citations)),
                List.of(recommendation("REVIEW_REPLAY_RECEIPTS", "Review replay evidence receipts",
                        "Replay bundles should retain provider/source mode and receipt IDs before use as evaluation evidence.", citations)),
                List.of(task("Review replay integrity report", validation.isAccepted() ? "LOW" : "HIGH", "DATA_STEWARD", "/simulation",
                        "Resolve missing replay fields before using this day for calibration or safety claims.", citations)),
                citations, receipts(citations), validation.getDiagnostics(), 0.9, 0.006);
    }

    private AgentRunResult historicalCalibrationCurator(AgentRunRequest request) {
        HistoricalReplayCalibrationReport report = simulation().historicalReplayCalibrationReport(null);
        List<AgentSourceCitation> citations = citations(report.getSourceRefs());
        if (citations.isEmpty()) citations = List.of(AgentSupport.citation("CALIBRATION", report.getId(), "Calibration report", "/config"));
        return result("HISTORICAL_CALIBRATION_CURATOR",
                "Calibration curator found " + report.getExpectedOutcomeCount() + " expected outcome label(s), "
                        + report.getFalseClearLabelCount() + " false-clear label(s), "
                        + report.getFalseBlockLabelCount() + " false-block label(s).",
                List.of(finding("CALIBRATION_READINESS", report.getUncalibratedCoefficientCount() > 0 ? "MEDIUM" : "INFO",
                        report.getUncalibratedCoefficientCount() + " coefficient(s) still require authoritative calibration evidence.",
                        0.87, citations)),
                List.of(recommendation("EXPAND_CALIBRATION_LABELS", "Expand calibration labels",
                        "Add authorized historical outcomes before treating coefficients as operationally calibrated.", citations)),
                List.of(task("Review calibration readiness", "MEDIUM", "SAFETY_REVIEW", "/config",
                        "Curate false-clear, false-block, route, weather, PIREP, NOTAM, and sector-capacity labels.", citations)),
                citations, receipts(citations), report.getDiagnostics(), 0.87, 0.006);
    }

    private AgentRunResult nationalDemandStress(AgentRunRequest request) {
        NationalDemandCapacityReport report = simulation().previewNationalDemandCapacity(NationalDemandCapacityConfig.builder()
                .id("safety-lab-demand")
                .flightCount(1200)
                .airportCount(12)
                .sectorCount(18)
                .durationMinutes(180)
                .tickIntervalMinutes(15)
                .demandSpikeFactor(1.5)
                .capacityReductionFactor(0.68)
                .includeWeatherCapacityReduction(true)
                .sourceMode("LOCAL_SYNTHETIC_NAS_SCALE")
                .build());
        List<AgentSourceCitation> citations = citations(report.getTrafficReplay() == null ? List.of("TFM:safety-lab-demand") : report.getTrafficReplay().getSourceRefs());
        return result("NATIONAL_DEMAND_STRESS_AGENT",
                "National demand stress preview covered " + report.getFlightCount() + " flight(s), "
                        + report.getAirportCount() + " airport(s), " + report.getSectorCount() + " sector(s).",
                List.of(finding("NATIONAL_DEMAND_STRESS", report.getPeakOverloadedSectorCount() > 0 ? "HIGH" : "INFO",
                        "Peak airport ratio " + round(report.getPeakAirportDemandCapacityRatio())
                                + ", peak sector ratio " + round(report.getPeakSectorDemandCapacityRatio())
                                + ", total TMI recommendations " + report.getTotalTmiRecommendationCount() + ".",
                        0.86, citations)),
                List.of(recommendation("RUN_SCALE_REGRESSION", "Run scale regression campaign",
                        "Use synthetic scale results to compare indexed candidate reduction and TMI effectiveness before demo claims.", citations)),
                List.of(task("Review national-scale stress preview", "HIGH", "TRAFFIC_MANAGER", "/tfm",
                        "Inspect overloaded sectors/airports and proposed TMIs before accepting the campaign baseline.", citations)),
                citations, receipts(citations), report.getDiagnostics(), 0.86, 0.009);
    }

    private AgentRunResult collaborativeDecisionFacilitator(AgentRunRequest request) {
        ProductDtos.CommonOperatingPictureSummary cop = readiness() == null ? null : readiness().commonOperatingPicture();
        List<AgentSourceCitation> citations = List.of(AgentSupport.citation("COP", cop == null ? "local" : cop.getId(), "Common operating picture", "/config"));
        String summary = cop == null
                ? "Common operating picture service unavailable; collaboration facilitation opened a review task."
                : "Common operating picture has " + cop.getActiveMissionCount() + " active mission(s), "
                + cop.getAffectedMissionCount() + " affected mission(s), " + cop.getPendingApprovalCount() + " pending approval(s).";
        return result("COLLABORATIVE_DECISION_FACILITATOR",
                summary,
                List.of(finding("COMMON_OPERATING_PICTURE", cop == null || cop.getPendingApprovalCount() > 0 ? "MEDIUM" : "INFO",
                        summary, 0.83, citations)),
                List.of(recommendation("REVIEW_COLLABORATIVE_PROPOSALS", "Review collaborative proposal queue",
                        "Accept/reject/approve/deliver states remain human-driven and delivery receipts remain local unless an authorized adapter is configured.", citations)),
                List.of(task("Review collaboration queue", "MEDIUM", "TRAFFIC_MANAGER", "/config",
                        "Disposition pending proposals and verify common operating picture sync state.", citations)),
                citations, receipts(citations), List.of("No external delivery performed by agent."), 0.83, 0.005);
    }

    private AgentRunResult providerFreshnessWatcher(AgentRunRequest request) {
        List<ProductDtos.ProviderHealthSummary> providers = readiness() == null ? Collections.emptyList() : readiness().providersStatus();
        List<AgentSourceCitation> citations = providers.isEmpty()
                ? List.of(AgentSupport.citation("PROVIDER", "status", "Provider status", "/config"))
                : providers.stream().map(provider -> AgentSupport.citation("PROVIDER", provider.getId(), provider.getLabel(), "/config")).toList();
        long stale = providers.stream()
                .filter(provider -> provider.getFreshness() != null && provider.getFreshness().isStale())
                .count();
        long credentialsRequired = providers.stream()
                .filter(provider -> String.valueOf(provider.getCredentialRequirement()).toUpperCase(Locale.US).contains("REQUIRED"))
                .count();
        return result("PROVIDER_FRESHNESS_WATCHER",
                "Provider freshness watcher reviewed " + providers.size() + " provider(s), " + stale
                        + " stale, " + credentialsRequired + " requiring credentials.",
                List.of(finding("PROVIDER_FRESHNESS", stale > 0 ? "HIGH" : "INFO",
                        "Provider status remains local/fixture/public-api-gated; live operational modes require credentials, consent, egress policy, and approval.",
                        0.85, citations)),
                List.of(recommendation("REVIEW_PROVIDER_MODES", "Review provider source modes",
                        "Do not claim authoritative live coverage while providers are fixture-backed, stale, or credentials-required.", citations)),
                List.of(task("Review provider freshness", stale > 0 ? "HIGH" : "MEDIUM", "ADMIN", "/config",
                        "Confirm freshness and authorization mode before running integration or evaluation demos.", citations)),
                citations, receipts(citations), List.of("Provider watcher did not activate external feeds."), 0.85, 0.004);
    }

    private AgentRunResult result(String type,
                                  String summary,
                                  List<AgentFinding> findings,
                                  List<AgentRecommendation> recommendations,
                                  List<AgentTask> tasks,
                                  List<AgentSourceCitation> citations,
                                  List<AgentEvidenceReceipt> receipts,
                                  List<String> diagnostics,
                                  double confidence,
                                  double estimatedCostUsd) {
        List<AgentSourceCitation> safeCitations = citations == null || citations.isEmpty()
                ? List.of(AgentSupport.citation("SAFETY_LAB", type, type, "/config"))
                : citations;
        return AgentRunResult.builder()
                .id(AgentSupport.id("agent", type + ":" + summary + ":" + safeCitations.size()))
                .agentType(type)
                .summary(summary)
                .confidence(confidence)
                .accepted(true)
                .humanApprovalRequired(true)
                .externalSendPerformed(false)
                .officialStateMutationPerformed(false)
                .generatedAt(ZonedDateTime.now(ZoneOffset.UTC))
                .costBudget(AgentCostBudget.builder()
                        .maxCostUsd(0.05)
                        .estimatedCostUsd(estimatedCostUsd)
                        .timeoutMillis(5_000)
                        .retryCap(1)
                        .circuitBreakerArmed(true)
                        .fallbackMode("DETERMINISTIC_LOCAL_REVIEW")
                        .build())
                .findings(findings)
                .recommendations(recommendations)
                .tasks(tasks)
                .citations(safeCitations)
                .evidenceReceipts(receipts == null || receipts.isEmpty() ? receipts(safeCitations) : receipts)
                .policyGuards(DEFAULT_GUARDS)
                .toolCalls(List.of(AgentToolCall.builder()
                        .id(AgentSupport.id("tool", type))
                        .toolName("LOCAL_SAFETY_LAB_WORKLOAD")
                        .status("COMPLETE")
                        .startedAt(ZonedDateTime.now(ZoneOffset.UTC))
                        .completedAt(ZonedDateTime.now(ZoneOffset.UTC))
                        .resultSummary("No external provider call, external send, or official mutation performed.")
                        .build()))
                .diagnostics(diagnostics == null ? DEFAULT_GUARDS : merge(DEFAULT_GUARDS, diagnostics))
                .build();
    }

    private AgentFinding finding(String category, String severity, String message, double confidence, List<AgentSourceCitation> citations) {
        return AgentFinding.builder()
                .id(AgentSupport.id("finding", category + ":" + message))
                .category(category)
                .severity(severity)
                .message(message)
                .confidence(confidence)
                .citations(citations)
                .build();
    }

    private AgentRecommendation recommendation(String action, String summary, String rationale, List<AgentSourceCitation> citations) {
        return AgentRecommendation.builder()
                .id(AgentSupport.id("rec", action + ":" + summary))
                .action(action)
                .summary(summary)
                .rationale(rationale)
                .confidence(0.86)
                .humanApprovalRequired(true)
                .citations(citations)
                .build();
    }

    private AgentTask task(String title, String priority, String role, String route, String rationale, List<AgentSourceCitation> citations) {
        return AgentTask.builder()
                .id(AgentSupport.id("task", title + ":" + route))
                .title(title)
                .status("OPEN")
                .priority(priority)
                .assignedRole(role)
                .route(route)
                .rationale(rationale)
                .citations(citations)
                .build();
    }

    private AgentWorkloadDefinition workload(String id, String label, String category, String gapCoverage, String description) {
        return AgentWorkloadDefinition.builder()
                .id(id)
                .label(label)
                .category(category)
                .gapCoverage(gapCoverage)
                .description(description)
                .defaultTrigger("manual-or-nightly-regression")
                .humanApprovalRequired(true)
                .externalSendAllowed(false)
                .policyGuards(DEFAULT_GUARDS)
                .build();
    }

    private List<AgentSourceCitation> citations(List<String> refs) {
        return AgentSupport.citations(refs);
    }

    private List<AgentEvidenceReceipt> receipts(List<AgentSourceCitation> citations) {
        return citations.stream()
                .map(citation -> AgentEvidenceReceipt.builder()
                        .id(AgentSupport.id("receipt", citation.getSourceFamily() + ":" + citation.getSourceId()))
                        .sourceFamily(citation.getSourceFamily())
                        .sourceId(citation.getSourceId())
                        .label(citation.getLabel())
                        .route(citation.getRoute())
                        .receiptHash(AgentSupport.id("sha", citation.getSourceFamily() + ":" + citation.getSourceId()))
                        .build())
                .toList();
    }

    private HistoricalReplayDay replayDay(AgentRunRequest request) {
        String dayId = request == null ? null : request.getHistoricalReplayDayId();
        if (dayId != null && !dayId.isBlank()) {
            return simulation().historicalReplayDay(dayId);
        }
        List<HistoricalReplayDay> days = simulation().historicalReplayDays();
        return days.isEmpty() ? null : days.get(0);
    }

    private List<AgentSourceCitation> replayCitations(HistoricalReplayDay day) {
        if (day == null) {
            return List.of(AgentSupport.citation("REPLAY", "missing", "Missing replay day", "/simulation"));
        }
        List<String> refs = new ArrayList<>(day.getPublicSourceRefs());
        if (refs.isEmpty()) refs.add("REPLAY:" + day.getId());
        return citations(refs);
    }

    private String severityForRedTeam(String message) {
        String upper = String.valueOf(message).toUpperCase(Locale.US);
        if (upper.contains("FALSE-CLEAR")) return "CRITICAL";
        if (upper.contains("FALSE-BLOCK") || upper.contains("LOST SOURCE")) return "HIGH";
        return "INFO";
    }

    private String normalize(String value) {
        return value == null || value.trim().isEmpty()
                ? "SAFETY_LAB_ALL"
                : value.trim().replace('-', '_').replace(' ', '_').toUpperCase(Locale.US);
    }

    private String value(String value, String fallback) {
        return value == null || value.trim().isEmpty() ? fallback : value;
    }

    private String round(double value) {
        return String.format(Locale.US, "%.1f", value);
    }

    private List<String> merge(List<String> left, List<String> right) {
        List<String> values = new ArrayList<>(left);
        if (right != null) values.addAll(right);
        return values;
    }

    private OperationalSimulationService simulation() {
        if (simulationService == null) {
            simulationService = new OperationalSimulationService(product());
        }
        return simulationService;
    }

    private AirspaceProductService product() {
        return productService;
    }

    private AirspaceReadinessService readiness() {
        if (readinessService == null && productService != null) {
            readinessService = new AirspaceReadinessService(productService);
        }
        return readinessService;
    }
}
