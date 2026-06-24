# Airspace Safety Lab Agents

Airspace includes deterministic, local-first agent workloads for simulation, traffic-flow, weather, replay, and safety review. These workloads are designed to make the engine easier to test and explain. They are not clearance authority, they do not send external messages, and they do not mutate official workflow state without human approval.

## Purpose

The Safety Lab closes the autonomous-AI gap around the Traffic Flow Management and weather-safety features already present in Airspace:

- identify unsafe guidance patterns before an operator sees them,
- audit outcome metrics such as false clear, false block, delay, fuel, source-ref completeness, and time to decision,
- propose draft scenarios for human-reviewed regression growth,
- inspect GDP, AFP, FCA/FEA, miles-in-trail, reroute, ground-stop, metering, arrival-rate, and sector-capacity recommendations,
- compare "what changed since last brief" across replay ticks and simulated days,
- verify replay/audit integrity,
- curate calibration candidates from historical-like replay fixtures,
- stress national demand/capacity simulation surfaces,
- draft collaborative-decision review notes,
- monitor provider freshness and fixture-vs-authoritative source modes.

The core principle is simple: agents can triage, explain, draft, audit, and red-team. Humans approve operational action.

## API Surface

```http
GET /api/agents/workloads
POST /api/agents/safety-lab
POST /api/agents/run
```

`GET /api/agents/workloads` returns the catalog of available workloads with scope, evidence sources, enabled state, cost budget, and policy guards.

`POST /api/agents/safety-lab` runs the full local safety-lab suite by default:

```json
{
  "agentType": "SAFETY_LAB_ALL",
  "scenarioId": "oceanic-altrv-convection",
  "actor": "planner"
}
```

`POST /api/agents/run` can run an individual workload by `agentType`, for example:

```json
{
  "agentType": "UNSAFE_GUIDANCE_RED_TEAM",
  "scenarioId": "low-vis-rvr-smgcs"
}
```

## Workload Catalog

| Workload | What it does | Primary evidence |
|---|---|---|
| `UNSAFE_GUIDANCE_RED_TEAM` | Searches for false-clear, false-block, stale-data overconfidence, source-ref gaps, and policy violations. | Simulation scenario, decision trace, safety dossier |
| `OUTCOME_METRICS_AUDITOR` | Reviews delay, fuel, reroute miles, overload avoided, false-clear, false-block, and source-ref completeness. | Outcome metrics, simulation KPIs |
| `SCENARIO_GENERATION` | Drafts new regression scenarios from current gaps and canonical scenario patterns. | Scenario catalog, weather/NOTAM/PIREP gap registry |
| `TMI_RECOMMENDATION_AUDITOR` | Audits GDP/AFP/FCA/FEA/miles-in-trail/reroute/ground-stop/departure-metering/arrival-rate recommendations. | TFM board, demand/capacity summaries |
| `BRIEF_DELTA_AGENT` | Summarizes what changed since a prior brief, replay tick, or simulated day. | Pilot brief, replay timeline, mission deltas |
| `REPLAY_INTEGRITY_AGENT` | Verifies replay, audit, source refs, hashes, and typed replay availability. | Replay bundle, audit envelope |
| `HISTORICAL_CALIBRATION_CURATOR` | Flags fixture-backed calibration candidates and uncalibrated coefficients. | Historical replay corpus, calibration reports |
| `NATIONAL_DEMAND_STRESS_AGENT` | Reviews national-scale demand/capacity stress runs and bottleneck sectors/airports. | National demand simulation, TFM board |
| `COLLABORATIVE_DECISION_FACILITATOR` | Drafts human-review coordination notes and common-operating-picture updates. | Collaborative decision workflow, recipient/role model |
| `PROVIDER_FRESHNESS_WATCHER` | Flags stale, fixture-backed, credential-required, or non-authoritative provider modes. | Provider status, source freshness |

## Guardrails

Every Safety Lab result carries explicit safety metadata:

- `humanApprovalRequired=true`
- `externalSendPerformed=false`
- `officialStateMutationPerformed=false`
- policy guards:
  - `ADVISORY_ONLY`
  - `NO_EXTERNAL_SEND`
  - `NO_OFFICIAL_MUTATION`
  - `HUMAN_APPROVAL_REQUIRED`
  - `CITED_EVIDENCE_REQUIRED`
  - `LOCAL_OR_REPLAY_FIRST`
- evidence receipts with source type, source ID, source hash, citation text, and generated time,
- cost budget with max cost, timeout, retry cap, fallback mode, and circuit-breaker state.

The policy enforcer rejects any result that claims an external send or official workflow mutation when the policy does not explicitly allow those actions.

## Frontend

The Agentic Ops panel exposes Safety Lab controls for:

- running the full Safety Lab suite,
- running unsafe-guidance red-team review,
- running outcome-metrics review,
- running TMI recommendation review,
- inspecting the workload catalog,
- reading policy guards, evidence receipts, cost budget, citations, findings, review tasks, and recommendations.

The panel is intentionally framed as an audit/review workbench. It should never imply that an AI agent is issuing clearances, releasing TMIs, changing mission state, or sending external coordination traffic.

## Non-Claims

The Safety Lab is not:

- FAA-certified,
- an operational SWIM/FNS/NMS/NADIN/WMSCR adapter,
- a calibrated historical outcome model,
- a substitute for ATC, TMU, dispatch, meteorology, or pilot authority,
- an autonomous messaging or state-transition system.

It is a production-shaped prototype layer for explainable, replayable, human-reviewed automation.
