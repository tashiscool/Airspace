package org.tash.extensions.agentic;

import jakarta.enterprise.context.ApplicationScoped;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.Optional;

@ApplicationScoped
public class AgentRiskCatalog {
    public List<AgentDefinition> agents() {
        return Arrays.asList(
                agent("WEATHER_IMPACT", "Weather Impact Watch Agent",
                        "Monitors mission weather, PIREP, NOTAM, and route-impact evidence for advisory findings.",
                        HumanReviewMode.REVIEW_ONLY,
                        "Read-only weather/PIREP/NOTAM evidence summarization; no official state change.",
                        "Incorrect advisory emphasis could cause unnecessary review, but no autonomous action occurs.",
                        Arrays.asList("airspace.weather.affected_missions", "airspace.mission.weather_verdict", "airspace.pireps.relevant")),
                agent("MISSION_RISK", "Mission Risk Analyst Agent",
                        "Summarizes mission-level risk and typed what-changed deltas from cited Airspace artifacts.",
                        HumanReviewMode.REVIEW_ONLY,
                        "Read-only mission risk assessment grounded in local product and engine records.",
                        "Could understate or overstate mission risk if source artifacts are stale or poisoned.",
                        Arrays.asList("airspace.mission.weather_verdict", "airspace.mission.route_impact", "airspace.decision.summary")),
                agent("REROUTE_ANALYSIS", "Reroute Analyst Agent",
                        "Creates reroute and avoidance recommendations from route-impact evidence.",
                        HumanReviewMode.PUSH_APPROVAL,
                        "Draft-only reroute guidance; operator must approve before operational use.",
                        "A misleading route suggestion could affect planning if accepted without human review.",
                        Arrays.asList("airspace.mission.route_impact", "airspace.decision.summary")),
                agent("COORDINATION_DRAFT", "Coordination Draft Agent",
                        "Drafts ATC/weather coordination language from cited mission, hazard, and decision sources.",
                        HumanReviewMode.PUSH_APPROVAL,
                        "Draft-only coordination text; cannot send external messages.",
                        "A bad draft could confuse coordination if a human transmits it without review.",
                        Collections.singletonList("airspace.coordination.draft")),
                agent("PILOT_BRIEF", "Pilot Brief Agent",
                        "Generates read-only pilot/controller handoff summaries from cited decision evidence.",
                        HumanReviewMode.REVIEW_ONLY,
                        "Read-only briefing generation; not a clearance or dispatch release.",
                        "An incomplete brief could omit relevant source context if source records are incomplete.",
                        Arrays.asList("airspace.decision.summary", "airspace.pireps.relevant")),
                agent("DATA_INTEGRITY", "Data Integrity Agent",
                        "Flags malformed, stale, contradictory, source-family-confused, or unsupported evidence.",
                        HumanReviewMode.PULL_CLARIFICATION,
                        "Read-only quality triage that opens clarification needs for suspect artifacts.",
                        "Could create review load from false positives; no workflow mutation occurs.",
                        Arrays.asList("airspace.feed.artifact_transactions", "airspace.reference.lookup")),
                agent("REPLAY_AUDIT", "Replay Audit Agent",
                        "Explains decision replay, trace, hashes, and source references.",
                        HumanReviewMode.REVIEW_ONLY,
                        "Read-only replay/audit explanation from persisted local records.",
                        "Could mislead audit review if replay payloads are malformed; citations and hashes remain visible.",
                        Collections.singletonList("airspace.decision.replay_audit")),
                agent("ALL", "Aggregate Agentic Operations Review",
                        "Runs the full deterministic agent set and combines cited findings, tasks, and drafts.",
                        HumanReviewMode.PUSH_APPROVAL,
                        "Composite advisory/draft review across all agent surfaces.",
                        "Largest local blast radius is operator confusion from aggregated advisory/draft output.",
                        Arrays.asList("airspace.weather.affected_missions", "airspace.mission.route_impact", "airspace.coordination.draft"))
        );
    }

    public Optional<AgentDefinition> agent(String agentType) {
        String normalized = normalize(agentType);
        return agents().stream()
                .filter(agent -> normalize(agent.getAgentType()).equals(normalized))
                .findFirst();
    }

    private AgentDefinition agent(String type,
                                  String name,
                                  String description,
                                  HumanReviewMode reviewMode,
                                  String autonomyScope,
                                  String blastRadius,
                                  List<String> tools) {
        return AgentDefinition.builder()
                .agentType(type)
                .name(name)
                .description(description)
                .recommendedToolIds(tools)
                .riskProfile(AgenticRiskProfile.builder()
                        .autonomyScope(autonomyScope)
                        .toolSurface("Agent type " + type + " with curated local MCP evidence where available.")
                        .worstCaseBlastRadius(blastRadius)
                        .permissionScope("Local Airspace read/draft scope only; no official send or workflow mutation.")
                        .dataEgress("None by default. Local mock LLM/MCP remains in-process; real external integrations are setup-required.")
                        .poisonedDataExposure("Can reflect stale, malformed, spoofed, or contradictory local feed artifacts; citations and HITL mode are required.")
                        .auditAttribution("Agent audit envelope, evaluation summary, reasoning envelope, citations, and optional MCP receipts.")
                        .toolProvenance("Airspace first-party deterministic agent catalog version agent-risk-v1.")
                        .modelProvenance("Deterministic draft-only by default; local mock/HTTP model providers require readiness gates.")
                        .rollbackPath("Disable the agent route, set policy stability mode to ENFORCE, or rely on deterministic output only.")
                        .costClass("LOCAL_CPU")
                        .slaClass("LOCAL_IN_PROCESS")
                        .requiredHumanReviewMode(reviewMode)
                        .build())
                .build();
    }

    private String normalize(String value) {
        if (value == null || value.trim().isEmpty()) {
            return "";
        }
        return value.trim().replace('-', '_').replace(' ', '_').toUpperCase(Locale.US);
    }
}
