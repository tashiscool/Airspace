# Airspace

Airspace is a Java 17 framework for modeling airspace reservations, legacy FAA-style messaging, NOTAM constraints, aviation weather hazards, PIREPs, route impact, and explainable operational decisions.

The project started as a general airspace-modeling sandbox, but the current codebase is now centered on an engine that can turn CARF/ALTRV reservations, USNS traffic, NOTAMs, weather products, and aircraft reports into structured constraints, conflict checks, route blockage predictions, and auditable recommendations.

## FAA Weather Safety Problem This Addresses

The product is built around this operational gap:

> The FAA still has unresolved flight-safety gaps around weather: real-time cockpit weather, turbulence and icing prediction, route avoidance, PIREPs, ATC/weather coordination, and pilot decision support. The problem is not that pilots lack weather apps; it is that the system still does not reliably turn live weather, aircraft reports, forecasts, and ATC constraints into clear operational guidance fast enough.

Airspace tackles that as a cooperative aviation decision loop:

1. **Observe:** ingest USNS/NADIN/WMSCR-shaped messages, CARF/ALTRV reservations, NOTAM constraints, METAR/TAF/SIGMET/AIRMET/CWAP-style weather, PIREPs, reference data, and route candidates.
2. **Normalize:** convert every input into typed artifacts with source refs, retained raw text, validity windows, altitude bands, geometry, freshness, confidence, provenance, route/reservation IDs, and diagnostics.
3. **Fuse:** combine weather, PIREPs, NOTAMs, CARF/ALTRV reservations, deconfliction, route geometry, and workflow state into common operational constraints.
4. **Predict:** evaluate affected missions, route segment impacts, altitude/time overlap, blockage probability, candidate alternatives, capacity/cost impact seams, and residual risk.
5. **Guide:** produce CLEAR, MONITOR, CAUTION, DELAY, ALTITUDE CHANGE, AVOID, REROUTE, or BLOCKED with rationale, confidence, source IDs, and coordination recommendations.
6. **Coordinate:** draft weather-desk, ATC, USNS, mission-owner, and pilot-handoff communications from exact source artifacts, with human approval before official send.
7. **Explain:** answer what changed, why a mission is affected, why one route is preferred, which sources mattered, and what remains uncertain.
8. **Audit:** retain request hashes, result hashes, rule-catalog versions, agent policy versions, source refs, replay bundles, and operator actions.

### Current Coverage Of The FAA Weather Gaps

| Safety gap | What Airspace ships today | Remaining boundary |
|---|---|---|
| Real-time cockpit/weather awareness | Weather and PIREP traffic appears in Mission Explorer, Weather, Mission, Decision, and Pilot Brief surfaces. Operators get weather deltas, freshness, affected missions, source chips, time-to-guidance metrics, and map overlays for coordinate-bearing products. | Local prototype only; no certified cockpit display or live avionics integration. |
| Turbulence and icing prediction | Weather models and parsers represent turbulence, icing, convection, ceiling/visibility, SIGMET/AIRMET, METAR/TAF, CWAP-style/CWAF-like products, forecast slices, confidence, validity, movement, and PIREP-derived hazards. The UI highlights turbulence/icing observations and route/altitude guidance. | Deterministic prototype scoring; needs historical calibration against authoritative outcomes. |
| Route avoidance | Route-impact APIs and UI panels expose impacted segments, blocking constraints, weather/PIREP/NOTAM/CARF source refs, action, confidence, and suggested avoidance candidates. The map keeps route impacts separate from reservations and NOTAMs. | Avoidance is a prototype suggestion path, not a certified NAS-scale routing solver. |
| PIREPs | PIREPs are parsed and modeled separately from forecast products, with stale/duplicate/incomplete diagnostics, urgency, route/altitude relevance, aging/decay, and dedicated map/table behavior. | Needs operational calibration and live dissemination feedback from authoritative networks. |
| ATC/weather coordination | Severe, urgent, stale, or low-confidence hazards produce coordination recommendations. Mission and Weather pages can open prefilled USNS/message drafts with mission, action, impact, and source refs. | Needs real recipient/position directory, live delivery confirmations, and operational staffing integration. |
| Pilot decision support | The decision engine fuses USNS, CARF/ALTRV, NOTAM, weather, PIREP, and route inputs into one explainable action. Decision and Pilot Brief pages show rationale, trace, blocking constraints, source refs, map features, audit, and replay. | Not FAA-certified; needs larger real-world corpus and validated confidence models. |

The acceptance target for the prototype is: a new SIGMET/PIREP/feed artifact should identify affected active missions locally in under 5 seconds, show a per-mission verdict badge, expose route impacts and source artifacts, support a coordination draft, and generate a printable pilot handoff brief.

### Readiness Registry And Release Gates

Airspace now exposes a machine-readable readiness layer so public copy, UI status, and safety claims stay aligned:

- `GET /api/gaps` lists the current gap registry across weather, PIREPs, NOTAMs, CARF/ALTRV, route avoidance, live feeds, calibration, UI workflow, replay/audit, scale, and safety claims.
- `GET /api/gaps/release-gates` reports `prototype-ready`, `public-review-ready`, `integration-ready`, and `operational-evaluation-ready` gates with required tests, required docs, excluded claims, non-goals, and blocking gaps.
- `GET /api/providers/status` reports provider mode, credentials/consent requirements, egress policy, freshness, diagnostics, and whether a source is fixture-backed, public API, authenticated external, or authorized operational.
- `POST /api/providers/weather/poll` is the configuration-gated Aviation Weather Center public-data seam. It is disabled by default and is not an operational SWIM/NAS feed.
- `POST /api/calibration/run` and `GET /api/calibration/reports` generate fixture-backed calibration reports that identify uncalibrated CWAP-style/CWAF-like, PIREP aging, storm lifecycle, and sector-demand coefficients.
- `GET /api/safety/dossier` summarizes scenarios tested, sources used, known gaps, rejected overclaims, replay-hash expectations, coverage summaries, and human-review checkpoints.

The first three gates can pass for local review and integration preparation. `operational-evaluation-ready` is intentionally blocked until authorized live feeds, historical calibration datasets, real recipient/delivery integration, high-volume operational testing, and formal safety review exist. This project is still advisory software and is not FAA-certified.

### Simulation Workbench

Airspace now includes a local aerospace operations simulation workbench at `/simulation`. It runs fixture-backed, minute-stepped scenarios that inject weather, PIREPs, NOTAMs, CARF/ALTRV reservations, and operator events, then records mission verdict changes, multi-aircraft state, runway/RVR/SMGCS surface state, sector workload, pilot/controller/operator behavior, stochastic weather-evolution seams, recorded SWIM/TFMS-like traffic replay, route-impact deltas, coordination drafts, pilot briefs, map playback, source refs, replay/audit IDs, and KPI summaries.

The traffic replay layer is local/replay-first: scenario authors can provide `TrafficReplayBundle` JSON with flight plans, scheduled/estimated departure and arrival times, aircraft position snapshots, filed routes, airport demand/capacity snapshots, sector demand/workload snapshots, and typed Traffic Flow Management primitives. GDP, AFP, FEA/FCA, miles-in-trail, reroute advisory, ground stop, departure metering, arrival rate, sector capacity, and TMI recommendations are modeled explicitly so the simulator can drive tick-level aircraft positions, airport queues/surface delay, sector workload/frequency congestion, affected-flight TMI review, and human-approved TFM recommendations while keeping `liveSwimNasDataUsed=false` unless a future authorized provider explicitly changes the source mode. `POST /api/simulations/traffic-replay/validate` checks replay structure before a run. `POST /api/simulations/national-demand/preview` and `SimulationRunRequest.nationalDemandCapacityConfig` generate local NAS-scale demand/capacity reports across hundreds or thousands of synthetic flights, airport resources, sector resources, overload snapshots, and advisory TFM recommendations.

Historical replay is modeled as a named day corpus over the same `TrafficReplayBundle` contract. `GET /api/simulations/historical-replay/days`, `POST /api/simulations/historical-replay/load`, and `POST /api/simulations/historical-replay/calibrate` expose synthetic and public-historical-like replay days with expected outcomes, source refs, data-quality warnings, and calibration-readiness reports. These days are useful for regression and demos, but they are not authorized operational evidence until future SWIM/TFMS/ASPM/NCEI/provider data is loaded with receipts and independent review. See [docs/HISTORICAL_REPLAY_CORPUS.md](docs/HISTORICAL_REPLAY_CORPUS.md).

The collaborative decision workflow is local/replay-first too. `GET /api/collaboration/common-operating-picture` assembles mission counts, affected missions, provider freshness, participants, proposals, pending approvals, delivery receipts, and source refs into a common operating picture. Operators can create proposals, comment, accept, reject, approve locally, and record delivery receipts through `/api/collaboration/proposals/...`; every proposal remains human-approved and `externalSendPerformed=false` unless a future authorized delivery adapter is configured. See [docs/COLLABORATIVE_DECISION_WORKFLOW.md](docs/COLLABORATIVE_DECISION_WORKFLOW.md).

The simulator is meant to show Airspace capabilities in motion: low-visibility RVR/SMGCS ambiguity, oceanic ALTRV weather avoidance, altitude-separated icing, PIREP safety override, volcanic ash reroute, runway surface contamination, sector capacity compression, blocked routes, and viable reroutes with residual risk. Campaign controls can sweep scenarios across multiple deterministic iterations and simulated days. `/simulation/author` supports bundle validation/import and advisory scenario-generation/red-team workloads, while `/simulation/runs/{runId}/replay` replays tick-level world state, source refs, aircraft, airport, sector, behavior, map overlays, and replay hashes. Campaigns can export a safety dossier with assumptions, KPI summaries, known gaps, replay hashes, and non-certification warnings.

The map remains GeoJSON-driven and now exposes a renderer registry: OpenLayers is active today, while Leaflet, Cesium, deck.gl, and generic GeoJSON adapter modes can be selected without changing backend contracts. It is not a physics-grade flight simulator, FAA-qualified FSTD, certified cockpit display, or live NAS traffic simulator. See [docs/SIMULATION_GUIDE.md](docs/SIMULATION_GUIDE.md), [docs/TRAFFIC_REPLAY_FORMAT.md](docs/TRAFFIC_REPLAY_FORMAT.md), [docs/TRAFFIC_FLOW_MANAGEMENT_PRIMITIVES.md](docs/TRAFFIC_FLOW_MANAGEMENT_PRIMITIVES.md), and [docs/NATIONAL_DEMAND_CAPACITY_SIMULATION.md](docs/NATIONAL_DEMAND_CAPACITY_SIMULATION.md).

### TFM Command Center Board

Airspace now has a dedicated Traffic Flow Management board at `/tfm`, backed by `GET/POST /api/tfm/board`. It turns the local SWIM/TFMS-like replay and national demand/capacity simulator into a command-center view: airport demand/capacity, sector load, active constraints, proposed TMIs, route alternatives, impact totals, and human-factors notes. The UI is modeled on public FSM/TSD/OIS/CDM concepts: show what is overloaded, show active constraints separately from recommended restrictions, preserve source refs, prefer the least restrictive suitable TMI, and require human approval before any external coordination or delivery.

The board is intentionally honest about source mode. Today it uses local synthetic or fixture-backed demand/capacity, not live FMDS/TFMS/SWIM/FNS/NMS/NADIN/WMSCR data. Capacity values, delays, and recommendations are review/demonstration artifacts until future authorized providers, calibration datasets, operational role directories, and external safety review are configured. See [docs/TFM_COMMAND_CENTER_BOARD.md](docs/TFM_COMMAND_CENTER_BOARD.md).

### Outcome Metrics

Airspace now has a local outcome-metrics surface at `/outcomes`, backed by `GET/POST /api/outcomes/metrics`. It aggregates simulation, route-impact, and TFM-board evidence into review metrics for delay minutes saved, fuel impact, reroute miles, sector overload avoided, false-clear, false-block, source-reference completeness, and operator time-to-decision.

These metrics are intentionally labeled as local modeled outcomes. Delay and fuel values compare simulation baselines against available reroute/TMI mitigation; source-ref completeness measures whether every guidance step keeps evidence attached; false-clear/false-block require scenario expected-outcome labels. They are useful for regression campaigns, safety-case review, and demos, but they are not FAA ASPM/TFMS post-event measurements or calibrated operational claims until authorized historical datasets and independent review are attached. See [docs/OUTCOME_METRICS.md](docs/OUTCOME_METRICS.md).

### Example Scenario: RVR / LVO / SMGCS Terminology Mismatch

One concrete reason Airspace has a USNS/NOTAM subsystem is to support low-visibility operational clarity. In a scenario where runway visual range is reported at 1000 feet and an international crew asks whether "LVO" or "LVP" is in force, a U.S. airport may use local/FAA terminology such as SMGCS or airport-specific low-visibility procedures instead of the exact phrase the crew used. The safety problem is not only the weather; it is that weather state, airport procedure state, operator minima, and ATC phraseology can fail to line up quickly enough.

Airspace is intended to make that ambiguity visible:

```text
Inputs:
- USNS/NADIN/WMSCR traffic containing DOM/FDC/ICAO/Canadian NOTAMs
- RVR/RVRT/RVRM/RVRR equipment or runway-visibility NOTAM text
- METAR/SPECI weather with low visibility or RVR groups
- Airport/reference procedure profile terms such as SMGCS, LVO, LVP
- Mission/reservation/runway/route context

Expected guidance:
- Classify NOTAMs as procedural constraints, not ALTRV reservations
- Flag low-visibility/RVR relevance and source the raw messages
- Preserve both FAA/local and ICAO/operator terminology
- Ask for coordination/confirmation when procedure state is ambiguous
- Produce advisory CAUTION/DELAY-style guidance until authorized local, company, airport-ops, or ATC sources confirm the applicable low-vis protections
```

The current engine and workbench support this prototype path by recognizing `RVR`, `RVRT`, `RVRM`, `RVRR`, `SMGCS`, `LVO`, `LVP`, and low-visibility wording in domestic NOTAM reduction, preserving source refs and warnings, surfacing low-vis/RVR notices in Feed and NOTAM views, and keeping these constraints distinct from CARF/ALTRV reservations. This is decision-support and coordination context only; official airport procedures, ATC instructions, company minima, and pilot authority remain authoritative.

Airspace also escalates related NOTAM families into mission guidance instead of leaving them buried in message tables:

- low-visibility/RVR procedure ambiguity can drive an advisory `DELAY / CONFIRM PROCEDURE STATE` posture until local procedure state is confirmed by authorized sources,
- approach/minima and approach-lighting NOTAMs are surfaced as arrival/departure capability review constraints,
- runway braking action, MU/friction, snow, ice, slush, and surface contamination are surfaced as operator performance-review constraints,
- pilot briefs and coordination drafts inherit those source refs and rationale.

## Public-Interest Release

Airspace is released as a donation-only public-interest safety prototype by **LeosSoftwareLLC**.

- **License:** [AGPL-3.0-or-later](LICENSE.md), chosen to keep network-service modifications public.
- **Governance:** [GOVERNANCE.md](GOVERNANCE.md), including anti-capture rules for core safety logic.
- **Public use and AI recreation policy:** [PUBLIC_USE_AND_AI_POLICY.md](PUBLIC_USE_AND_AI_POLICY.md), requiring disclosure and source publication for use, deployment, reference implementation, or LLM-assisted recreation.
- **Compliance disclosure:** [COMPLIANCE.md](COMPLIANCE.md) and [docs/COMPLIANCE_COLLECTOR.md](docs/COMPLIANCE_COLLECTOR.md), providing transparent self-attestation, public disclosure, and redacted operational-configuration publication paths without hidden telemetry or clone-time collection.
- **Contributing:** [CONTRIBUTING.md](CONTRIBUTING.md), including tests, fixture expectations, and certification-language guardrails.
- **Safety notice:** [SAFETY.md](SAFETY.md), clarifying that Airspace is not certified, not operational authority, and not a replacement for official FAA/NWS/ATC systems.
- **Whitepaper:** [docs/SAFETY_WHITEPAPER.md](docs/SAFETY_WHITEPAPER.md), a concise explanation of the safety problem, prototype approach, claims, and review questions.
- **Evaluation guide:** [docs/EVALUATION_GUIDE.md](docs/EVALUATION_GUIDE.md), local run steps and reviewer checklist.
- **Demo script:** [docs/DEMO_SCRIPT.md](docs/DEMO_SCRIPT.md), a 3-5 minute walkthrough script for a public review video.
- **Outreach templates:** [docs/PUBLIC_REVIEW_OUTREACH.md](docs/PUBLIC_REVIEW_OUTREACH.md), short messages for safety researchers, operators, and public-interest reviewers.

The preferred path is public review and collaboration, not contractor capture. Anyone using Airspace, adapting it, or using LLM/code-generation systems against the code, docs, screenshots, fixtures, schemas, prompts, or workflows to recreate similar software is expected to disclose that use and publish the resulting implementation source. Feedback should focus on safety usefulness, missing scenarios, misleading guidance, parser gaps, route-impact assumptions, traceability, and what would need calibration before any real operational use.

Airspace does not include hidden telemetry or a clone-time beacon. Instead, it ships a transparent compliance path:

- `GET /api/compliance/policy`
- `GET /api/compliance/manifest`
- `POST /api/compliance/attestations`
- `GET /api/compliance/attestations`
- `./scripts/airspace-compliance-manifest.sh`
- `.github/ISSUE_TEMPLATE/airspace-use-disclosure.yml`

The manifest and attestation flow asks users to publish source and redacted operational configuration for derivative deployments, integrations, and LLM-assisted recreations while explicitly forbidding publication of secrets, credentials, private operational data, personal data, or sensitive aviation-security details.

### Competitive Positioning

The European-style benchmark is a live weather rerouting console: affected flights turn red, operators compare the original route against alternatives, and the system estimates extra distance, delay, fuel, and cost. Airspace now includes that prototype-grade reroute comparison surface, but the product goal is broader: **weather rerouting plus FAA-style operational fusion**.

Airspace combines reroute candidates with USNS/NADIN/WMSCR-style messaging, NOTAM constraints, CARF/ALTRV reservations, PIREPs, protected-volume deconfliction, source traceability, audit/replay, and pilot handoff. The route-impact API returns structured candidate comparisons with original-route estimates, added distance, delay, fuel, cost, avoided hazards, residual constraints, and “Why this reroute?” trace rule IDs/source refs. The deterministic cost seam exposes its assumptions directly in the UI: cruise speed, fuel burn per nautical mile, fuel price, and delay cost per minute, so prototype economics stay inspectable rather than magic. The workbench exposes affected mission map mode, route-candidate map overlays, selected-feature reroute cost readouts, weather event drilldowns, and source-family chips so controllers can move from hazard to affected mission to reroute review to coordination draft without losing provenance.

Airspace now also includes an agentic operations layer. Unlike a black-box AI recommender, the agent layer operates over typed engine artifacts and cited tool outputs: weather impact, mission risk, reroute comparison, coordination draft, pilot brief, data-integrity scan, and replay audit. Agent output is labeled as assistant analysis or draft material, includes source refs/input-output hashes/policy version, and cannot mutate official workflow state or send external messages without a human operator.

The autonomous-AI extension of that layer is the **Airspace Safety Lab**: a local-first suite of evidence-cited workloads for unsafe-guidance red-team review, outcome-metrics auditing, draft scenario generation, TMI recommendation review, brief-delta comparison, replay-integrity checks, calibration curation, national-demand stress review, collaborative-decision facilitation, and provider-freshness monitoring. The catalog and guardrails are documented in [docs/AIRSPACE_SAFETY_LAB_AGENTS.md](docs/AIRSPACE_SAFETY_LAB_AGENTS.md).

## What Is Implemented

### CARF / ALTRV / Reservation Engine

- CARF route-message parsing and reservation construction.
- Modern ALTRV parser package under `org.tash.extensions.carf.altrv`.
- A-G and stationary reservation section models, grammar-oracle route families, route events with source spans/metadata, route graph building, route graph validation, diagnostics, and spatial mapping.
- Legacy `ALTRV.g` behavior is implemented as modern typed parsing for standard, partial, common, branch, alternate departure, reverse, implicit, stationary, and maneuver routes, including bounded polygons, radius areas, line corridors, timing triangles, maneuver areas, enter/exit associations, route-event fix/time/altitude metadata, and unresolved-fix diagnostics.
- ALTRV geometry intent now survives downstream mapping: `POLYGON`, `POINT_RADIUS`, `LINE_CORRIDOR`, `TIMING_TRIANGLE`, `MANEUVER_AREA`, and `STATIONARY_RESERVATION` are retained in reservation events, source text, diagnostics, and GeoJSON feature properties instead of being flattened into one generic reservation shape.
- Reservation conflict detection with explicit lateral, longitudinal, vertical, and time explanations.
- Regression coverage for the 2010 legacy CARF examples that motivated the project, including:
  - lateral-separation-safe parallel routes,
  - head-on examples,
  - too-short/zero-duration conflicts,
  - navaid/fix-backed route examples.
- CARF reference-data abstractions and schema catalog under `org.tash.extensions.carf.refdata`.

Key entry points:

```java
CarfAnalysisService service = new CarfAnalysisService();
CarfAnalysisResult result = service.parseValidateMap(rawCarfOrAltrvText);
```

### USNS / Messaging / NOTAM Compatibility

- USNS envelope parsing for MRS/NADIN/WMSCR-style traffic.
- Transaction splitting and family classification for DOM, FDC, ICAO NOTAM, Canadian domestic, SNOWTAM, BIRDTAM, ASHTAM, GENOT, service requests, CARF-like bodies, and weather/advisory traffic.
- USNS ingest results now expose typed ICAO/Canadian NOTAM field summaries alongside domestic NOTAM parse results, so feed artifacts can retain Q/A/B/C/D/E/F/G metadata, PERM/EST flags, geometry presence, and missing-geometry diagnostics without confusing NOTAMs with ALTRV reservations.
- The Feed workspace surfaces retained NOTAM fields, typed service request/table command fields, and classified legacy family metadata directly in transaction tables and notices, distinguishing geometry-capable constraints, non-geometric NOTAM metadata, RQ/TBL administrative traffic, and GENOT/SNOWTAM/BIRDTAM/ASHTAM retained traffic that must stay table/audit-only.
- Product search indexes typed feed transaction metadata, including rehydrated persisted feed artifacts, so operators can search by ICAO q-code, NOTAM affected location, DOM2 reducer rule ID, domestic keyword, q23/q45, semantic family, service request operation, table command, or retained family semantic after restart and jump back to the retained feed artifact.
- Search results distinguish full feed artifacts from individual feed transactions; selecting a NOTAM transaction preserves the feed artifact context while marking the source family as a NOTAM constraint in the workbench context strip.
- Batch-oriented parse diagnostics and routing outcomes.
- Domestic NOTAM parser support for DOM1 record-shape behavior, cancellations, edits, comments, WEF/TIL, PERM/EST, 10/12-digit date forms, keyword defaults, unofficial/Canadian-crossover cases, and malformed-duration diagnostics.
- DOM2 semantic classification is implemented as a second-stage reducer with facility family, condition/action, q23/q45 where known, reducer rule IDs, recognized contractions, and fallback warnings while preserving accepted raw records.
- RVR/RVRT/RVRM/RVRR, SMGCS, LVO/LVP, and low-visibility procedure wording are retained as procedural coordination constraints. Runway-level RVR records reduce canonically as `DOM2.RWY.RVR`, aerodrome-wide RVR outages as `DOM2.AD.RVR_ALL`, and legacy service-style RVR traffic remains compatibility-handled with DOM2-compatible rejection of malformed `SVC <runway> RVR ...` records that omit the `RWY` keyword. This is meant to support runway-visibility/low-visibility terminology mismatch cases without turning NOTAMs into ALTRV reservations.
- Approach/minima, approach-lighting, runway-friction, and braking-action NOTAM semantics feed mission weather verdicts, affected-mission queues, coordination drafts, and pilot briefs as operational guidance drivers.
- ICAO/Canadian NOTAM field parsing retains NOTAMN/NOTAMR/NOTAMC/NOTAMJ Q/A/B/C/D/E/F/G metadata even when no route-usable geometry exists; geometry-capable NOTAMs still map to native airspace restrictions and non-geometric records retain diagnostics instead of becoming fake map features.
- NOTAM access-policy abstractions and in-memory reference data.

Key entry point:

```java
UsnsIngestService ingest = new UsnsIngestService();
UsnsIngestResult result = ingest.parse(rawUsnsMessage);
```

### Weather Decision Engine

- Structured weather product model for convection, turbulence, icing, ceiling, visibility, SIGMET, AIRMET, METAR, TAF, NEXRAD/CWAP-style/CWAF-like advisories, PIREP-derived hazards, and generic forecast hazards.
- Product-specific decoders for pragmatic METAR, TAF, SIGMET, AIRMET, CWAP-style/CWAF-like, and PIREP parsing.
- Configuration-gated live NOAA/AWC polling through `LiveAviationWeatherAdapter`; live mode is disabled by default and keeps tests/offline demos network-free.
- Deterministic weather-pattern mapping normalizes retained products into convection, turbulence, icing, wind shear, volcanic ash, ceiling/visibility, precipitation, PIREP cluster, terminal forecast, and generic advisory patterns with source refs, geometry intent, time/altitude windows, confidence, freshness, and diagnostics.
- METAR/TAF products without usable route geometry remain station/time guidance artifacts; Airspace does not create fake polygons for non-geometric products.
- Forecast slicing for TAF/CWAP-style products.
- Route weather decision support with actions such as clear, monitor, caution, delay, altitude change, reroute, avoid, and blocked.
- Route blockage prediction with severity, echo tops, growth/decay, storm phase, lead-time confidence, stale-product diagnostics, ensemble uncertainty, and capacity-impact seams.
- PIREP ingestion, duplicate detection, quality diagnostics, automated draft capture, and dissemination status modeling.
- ATC/weather coordination models for review items, operational constraints, controller handoff notes, and meteorologist review priority.
- Mission-scoped weather verdicts, affected-mission summaries, relevant-PIREP filtering, route-impact summaries, coordination drafts, and pilot-brief summaries in the product service layer.
- Weather visualization metadata for hazard type, severity, confidence, forecast hour, validity, movement, source product, stale state, blocked route segments, and recommended action.

Key entry point:

```java
WeatherDecisionSupportService weather = new WeatherDecisionSupportService();
RouteWeatherAdvisory advisory = weather.adviseRoute(request);
```

### Unified Operational Decision Engine

- Top-level fusion facade under `org.tash.extensions.engine`.
- Fuses raw USNS messages, raw CARF/ALTRV text, structured weather products, PIREPs, NOTAM restrictions, route candidates, reference data, and decision time.
- Produces reservations, conflicts, weather products, PIREP results, route blockage predictions, coordination advisories, recommended actions, and a structured decision trace.
- Normalizes CARF reservations, NOTAM restrictions, weather hazards, PIREPs, route blockage, and conflicts into common operational constraints.
- Keeps source refs attached through parse, classify, map, fuse, index, route-impact, rule-evaluation, action, audit, replay, and visualization surfaces.
- Supports route-impact scoring breakdowns, rule-catalog IDs, confidence math, stale-data warnings, calibration seams, ensemble uncertainty fields, storm lifecycle seams, sector capacity impact seams, and deterministic replay bundles.
- Includes deterministic audit/replay support:
  - canonical JSON,
  - HMAC signing,
  - request/result hashes,
  - engine/rule-catalog versions,
  - replay verification.

Key entry point:

```java
OperationalDecisionEngine engine = new OperationalDecisionEngine();
OperationalDecisionResult result = engine.evaluate(request);
```

### Agentic NextGen Operations Layer

- Additive semi-autonomous agent layer under `org.tash.extensions.agentic`.
- Deterministic implementations work without external LLM keys. `ConfigurableLlmReasoningProvider` is present but disabled by default; `airspace.agentic.llm.mode=local-test` enables a no-network cited test provider, and `airspace.agentic.llm.mode=http` can call a local/model gateway endpoint while still passing citation and policy validation before output is accepted.
- Agents currently provided:
  - Weather Impact Watch Agent,
  - Mission Risk Analyst Agent,
  - Reroute Analyst Agent,
  - Coordination Draft Agent,
  - Pilot Brief Agent,
  - Data Integrity Agent,
  - Replay Audit Agent.
- Safety Lab autonomous-review workloads:
  - Unsafe Guidance Red Team,
  - Outcome Metrics Auditor,
  - Scenario Generation Agent,
  - TMI Recommendation Auditor,
  - Brief Delta Agent,
  - Replay Integrity Agent,
  - Historical Calibration Curator,
  - National Demand Stress Agent,
  - Collaborative Decision Facilitator,
  - Provider Freshness Watcher.
- Every agent result includes source citations, confidence, diagnostics, generated time, policy version, input hash, output hash, audit envelope, and an explicit NextGen operating-loop snapshot from observe through audit.
- Safety Lab results also include evidence receipts, policy guards, a cost/time/retry budget, and explicit booleans proving the run did not send external messages or mutate official workflow state. Human approval is required for operational use of any draft, finding, coordination note, or recommendation.
- Mission-risk and replay-audit output use `OperationalDeltaService` for typed “what changed” deltas, including action changes, confidence changes, new source refs, route candidate changes, blocking-constraint changes, and fallback mission weather/PIREP/NOTAM deltas.
- Data-integrity scans flag malformed or unsupported inputs, stale products, missing geometry, ambiguous DOM2 semantic reduction, malformed domestic NOTAM records, ICAO/Canadian NOTAMs retained without route-usable geometry, empty/ambiguous ICAO Q-field FIR prefixes, contradictory severe weather vs smooth/negative PIREPs, off-route/off-altitude PIREP relevance mismatches, missing source refs, and route candidates with residual NOTAM/CARF/ALTRV risk.
- Agent policy enforcement blocks future autonomous external-send or official workflow-mutation recommendations unless policy explicitly permits them.
- Agent evaluation summaries score citation coverage, policy violations, source-family coverage, claim counts, and delta counts so future model-backed reasoning can be gated before display.
- Agent evaluation now includes fixed-shape `AgentAssessment` envelopes: claim, verdict, evidence, counter-evidence, uncertainty, required human action, human-review mode, and citations. This makes schema validity explicit while preserving the rule that shape does not prove judgment soundness.
- Human review is typed instead of generic: `REVIEW_ONLY` for advisory output, `PULL_CLARIFICATION` for missing/ambiguous/malformed/conflicting evidence, and `PUSH_APPROVAL` for draft or official-action-adjacent recommendations. Existing `humanApprovalRequired` remains for compatibility and maps to push approval.
- Each run also carries a model-ready reasoning envelope with prompt version, draft hash, deterministic/model mode, input summary, allowed facts, required output rules, prohibited actions, and citations. The default mode is `DETERMINISTIC_DRAFT_ONLY`, so no external LLM key is required and future model providers must reason from the cited draft context.
- The agent layer now includes a governed MCP-style tool plane. `/api/agents/mcp/servers`, `/api/agents/mcp/servers/{serverId}/tools`, `/api/agents/mcp/tools/call`, and `/api/agents/mcp/receipts` expose curated tool discovery, first-party invocation, redacted evidence receipts, and setup-required external MCP placeholders.
- First-party MCP tools cover affected missions, mission weather verdicts, route impact, relevant PIREPs, decision summaries/replay audits, feed artifact transactions, reference lookup, and coordination drafts. They are `READ_ONLY` or `DRAFT_ONLY`; mutating and external-send tools remain blocked by default policy.
- MCP tool descriptors include an `AgenticRiskProfile` with autonomy scope, tool surface, worst-case blast radius, permission scope, data egress, poisoned-data exposure, audit attribution, tool/model provenance, rollback path, cost/SLA class, and required human-review mode. `/api/agents/risk-assessments` exposes the same checklist for the Agentic Ops panel and audits.
- `AgentJobQueueService` and `AgentJobConsumer` provide an in-process queue/consumer that plans policy-allowed MCP tools, collects receipts, attaches `AgentToolCall` evidence to the run, and then executes the existing deterministic agents. `/api/agents/jobs` exposes local job submission and retained job history.
- `POST /api/agents/evaluate/stability` runs repeated local agent evaluations and reports accepted/verdict agreement, cited-source Jaccard, finding/recommendation/task count wobble, action/category agreement, task route/priority agreement, and MCP receipt hash agreement. This measures repeatability; it does not certify correctness or authorize action.
- Replay/trace questions return a structured `AgentTraceAnswer` with the operator question, evidence-grounded answer, confidence, source refs, rule IDs, unsupported-claim diagnostics, and citations.
- Run/task history is behind an `AgentRunStore` seam with in-memory and JSON-file implementations plus searchable detail/list API endpoints for audit drilldown. Runs can be filtered by agent type, mission, reservation, decision, acceptance state, and source family; tasks can be filtered by status, priority, role, source family, and workspace route. The JSON store keeps local/demo agent findings, trace answers, reasoning envelopes, and task acknowledgments restart-safe without adding a database dependency.
- Set `airspace.agentic.store.path=/path/to/agent-runs.json` to enable the JSON-backed store; leave it blank for default in-memory behavior.
- `/api/agents/status` and the Agentic Ops audit card expose the active store mode, durability, configured path, retained run count, retained task count, and latest run timestamp.
- `/api/agents/metrics` summarizes retained agent runs, accepted/rejected counts, task status/priority counts, policy violations, uncited claims, and store durability for local observability.
- `/api/metrics` includes the same `agentic.*` counters alongside product/feed/weather metrics, and `/api/config` reports the active agentic store mode/durability.
- Citation validation blocks unsupported operational claims from being accepted as guidance.
- Product endpoints are available under `/api/agents/*`, including `/api/agents/run`, `/api/agents/weather-impact`, `/api/agents/mission-risk`, `/api/agents/reroute-analysis`, `/api/agents/coordination-draft`, `/api/agents/pilot-brief`, `/api/agents/data-integrity`, `/api/agents/replay-audit`, `/api/agents/delta`, `/api/agents/scenario/generate`, `/api/agents/safety-lab`, `/api/agents/workloads`, `/api/agents/evaluate/stability`, `/api/agents/risk-assessments`, `/api/agents/runs`, `/api/agents/tasks`, `/api/agents/mcp/*`, and `/api/agents/jobs/*`.
- The React workbench shell includes an **Agentic Ops** panel that shows active findings, fixed-shape assessments, review tasks, recommendations, HITL modes, typed “what changed” deltas, trace answers, citations, evaluation coverage, policy issues, reasoning-envelope prompt/draft hashes, available/blocked MCP tools, MCP risk/blast-radius metadata, evidence receipts, Safety Lab workload catalog, cost budget, policy guards, queued jobs, stability metrics, the observe/normalize/fuse/predict/guide/coordinate/explain/audit loop, audit hashes, retained run count, and task acknowledgment controls.
- `ScenarioFixtureGenerator` creates deterministic, human-reviewable scenario bundles for severe convection, altitude-separated icing/turbulence, PIREP clusters, NOTAM/weather compound constraints, viable reroutes, blocked routes, and malformed retained inputs. Calibration/live-feed readiness is represented by fixture-backed weather outcome, storm lifecycle, sector demand, and local replay validation seams; these are not live operational calibration claims.
- See [docs/agentic-mcp.md](docs/agentic-mcp.md), [docs/agentic-stability-harness.md](docs/agentic-stability-harness.md), and [docs/agentic-risk-checklist.md](docs/agentic-risk-checklist.md) for the curated tool catalog, evidence-receipt contract, queue behavior, stability evaluation, risk checklist, HITL taxonomy, and draft-only autonomy boundary.

Key entry point:

```java
AgenticOperationsService agents = new AgenticOperationsService();
AgentRunResult result = agents.run(request);
```

Safety Lab catalog and run APIs:

```http
GET /api/agents/workloads
POST /api/agents/safety-lab
```

### Operations Product Layer

- Quarkus product API resources under `org.tash.extensions.product.api` for auth, missions, messages, feed ingest, decisions, reference data, config, history, and metrics.
- Product DTOs keep HTTP payloads separate from engine models and persistence entities.
- Baseline PostgreSQL/Flyway schema under `src/main/resources/db/migration`.
- JPA entity/repository seams under `org.tash.extensions.product.persistence` for missions, reservations, messages, NOTAMs, APREQs, approvals, weather products, PIREPs, operational decisions, users, reference points, and parsed ALTRV route graph targets.
- Quarkus ORM/Flyway/JDBC wiring is active: dev/test use H2 in PostgreSQL mode, while prod points at PostgreSQL through `AIRSPACE_JDBC_URL`, `AIRSPACE_DB_USERNAME`, and `AIRSPACE_DB_PASSWORD`.
- `ProductPersistenceService` provides transactional JPA persistence for product aggregates; the product API can run in in-memory mode or persisted mode through `airspace.product.persistence.enabled`.
- Persisted product aggregates currently include missions, mission locks, reservation workflow summaries, messages, weather products, and operational decisions/audit payloads.
- Persisted decisions can be reloaded into typed decision/audit/replay objects for restart-safe replay checks and GeoJSON feature generation. If a historical payload cannot be fully rehydrated, the API falls back to stored JSON hash/signature verification and returns explicit diagnostics.
- Product workflow APIs include distinct reservation parse/deconflict and force-parse/force-deconflict actions, message reply/forward actions, feed transaction inspection, reservation supplements, decision replay verification, and decision feature export.
- Reservation supplement lifecycle transitions are exposed for NOTAM/APREQ/approval-style records so operator actions can move draft/open records through submitted, approved, rejected, or cancelled states.
- Product search surfaces cover missions, messages, feed artifacts, decisions, and history, with a `ProductSearchService` facade for typed search entry points.
- The second product migration captures the useful legacy `MissionModelMapping.hbm.xml` shape without restoring legacy Hibernate/XML runtime: ALTRV messages, route groups, routes, route events, fix-times, areas, exits/destinations, notes/images, message recipients, airspace info, separation groups, aircraft validation, and preferred navaids.
- Local adapter seams are present for USNS/NADIN/WMSCR traffic, weather traffic, KVM-style bridge input, and reference-data sync. The checked-in implementations are file/in-memory replay adapters only; they are shaped for future live adapters without requiring credentials or network services.
- Reference-data import preview/apply endpoints accept local CSV-style navaid/fix/aerodrome/global-account records and preserve source/version metadata in reference-point records.
- Local RBAC auth is available for product development with default users:
  - `planner` / `planner`
  - `supervisor` / `supervisor`
  - `admin` / `admin`
- When `airspace.product.auth.required=true`, product APIs enforce role-aware workflow access: planners can create/edit/submit, supervisors can approve/reject, and admins can manage reference/config surfaces.

The first product API path is intentionally local/test-friendly: the engine remains usable without a live database, while the production Postgres schema and JPA model are present for deployment hardening.

### React Operations Console

- A React + TypeScript + Vite frontend lives in `frontend/`.
- It provides the initial operations shell for login, a four-pane CARF-style Mission Explorer, mission/reservation workspace, reservation save/lock/parse/submit/approve/reject/cancel/complete actions, deconfliction review, messaging, decision review, search, history, and config.
- The map stack is OpenLayers and consumes the backend’s GeoJSON-compatible feature collections.
- Mission Explorer is the time-to-guidance board: per-mission weather verdicts, affected active missions, weather/PIREP/NOTAM deltas, attention filters, source-family chips, and coordinate actions.
- Weather and PIREP pages expose real-time-style product intake, route blockage counts, PIREP workflow, coordination queue, freshness, no-geometry notices for METAR/TAF, and map/table coupling for coordinate-bearing hazards.
- Weather Pattern APIs expose live status, optional AWC polling, normalized patterns, grouped events, GeoJSON feature output, and deterministic route sampling for trajectory/corridor-style review.
- Mission, Reservation, Decision, and Pilot Brief pages show route impacts, blocking constraints, avoidance candidates, exact source refs, trace/audit/replay, and a readable handoff summary.
- Mission, Reservation, Decision, and Pilot Brief pages include route candidate comparison panels: original vs alternate corridor, added distance, estimated delay, fuel, cost, avoided hazards, residual constraints, confidence, and “Why this reroute?” rule/source trace.
- Route candidate panels and selected route-impact map features show the deterministic cost assumptions behind those estimates, keeping prototype distance, delay, fuel, and cost math operator-auditable.
- Mission and Reservation maps include derived route-candidate GeoJSON overlays from the structured comparison DTOs, so selecting **Show On Map** on an alternate route highlights the matching route-impact layer.
- Affected mission summaries include route geometry, allowing the Weather board to draw affected active mission routes directly in affected-mission map mode instead of only highlighting the weather source overlays.
- Affected mission summaries also expose the best alternate route penalty: added distance, delay, fuel, cost, avoided constraint count, and residual constraint count, so the Weather board behaves more like a real-time reroute operations list while retaining FAA-style source refs.
- The Weather page includes drilldowns for volcanic ash, hurricane/convection, icing/turbulence, PIREP clusters, ceiling/visibility, and generic weather, with affected mission counts and map/table coupling.
- The map keeps CARF/ALTRV reservations, route impacts, conflicts, NOTAMs, weather, PIREPs, and reference points distinct, with forecast/freshness/altitude filtering, affected mission mode, selected feature risk readout, visible risk counts, retained ALTRV geometry intent, raw grammar source text, and parser diagnostics.

Frontend commands:

```bash
cd frontend
npm install
npm test -- --run
npm run build
npm run test:e2e
```

Screenshot walkthrough commands:

```bash
# Terminal 1: local API with H2/PostgreSQL-mode dev storage
JAVA_HOME=$(/usr/libexec/java_home -v 17) mvn quarkus:dev -Dquarkus.enforceBuildGoal=false -Dquarkus.http.port=8090 -Dquarkus.test.continuous-testing=disabled

# Terminal 2: browser console
cd frontend
npm run dev -- --host 127.0.0.1

# Terminal 3: seed demo data through the real APIs and capture product screenshots
cd frontend
npx playwright install chromium
npm run screenshots
```

The screenshot script uses the real local product APIs to create a mission, reservation, NOTAM/APREQ/approval supplements, USNS/feed traffic, weather/PIREP messages, reference points, and a replayable operational decision. It then drives the React workbench with Playwright and writes PNGs to `docs/screenshots/`.

### Product Walkthrough

The current operator-facing product path is working locally as an integrated prototype. These screenshots are generated from the checked-in app, not mockups.

The safety loop this product is trying to close is explicit in the workbench:

- **Weather product intake:** METAR/TAF/SIGMET/AIRMET/CWAP-style products, PIREPs, and generic weather advisories enter through USNS/feed/message paths.
- **Route blockage and avoidance:** weather products are fused with route candidates, CARF/ALTRV reservations, NOTAM constraints, altitude bands, timing windows, and route-segment metadata.
- **Route candidate comparison:** original routes and alternatives are compared with distance, delay, fuel, cost, avoided hazards, residual constraints, confidence, and a “Why this reroute?” trace.
- **Affected mission map mode:** affected active mission routes/source-linked overlays can be emphasized while unrelated map layers are dimmed.
- **Weather event drilldown:** volcanic ash, hurricane/convection, icing/turbulence, PIREP clusters, ceiling/visibility, and generic weather are grouped for fast inspection.
- **PIREP handling:** aircraft reports are modeled separately from forecasts so turbulence/icing observations can drive review priority and route/altitude guidance.
- **ATC/weather coordination:** severe, urgent, stale, or low-confidence products become review/coordination items instead of disappearing into raw text.
- **Pilot/operator decision support:** the decision engine returns a single recommended action with confidence, rationale, blocking constraints, map features, trace steps, audit envelope, and replay bundle.
- **Agentic review:** cited assistant findings, review tasks, reroute recommendations, coordination drafts, pilot briefs, data-quality warnings, and replay explanations sit on top of the deterministic engine without becoming an autonomous clearance authority.

This is not a certified cockpit system or live FAA feed integration. It is a local, auditable operations prototype showing the engine and workbench path for turning weather, reports, forecasts, reservations, NOTAMs, and ATC constraints into clear guidance quickly.

The screenshots below are organized around that safety loop:

- **Mission Explorer and Mission Workspace:** show the operational queue, affected missions, weather verdicts, what changed since last brief, source-family chips, route impact summaries, and route candidate comparison.
- **Pilot Brief:** converts the same fused decision into a read-only handoff with verdict, route impact, route alternatives, coordination guidance, source artifacts, and printable trace summary.
- **Reservation and Deconfliction:** keep CARF/ALTRV parsing, protected volumes, conflicts, NOTAM/APREQ/approval supplements, and force-action diagnostics visible to the operator.
- **Messaging and Feed:** show retained USNS/weather/PIREP traffic, parsed transaction details, diagnostics, and downstream artifact IDs.
- **Decision Summary, Trace, and Map:** show the replayable engine action alongside the mission-context route action, confidence, blocking constraints, route predictions, route comparison, rule trace, source refs, audit/replay, selected reroute candidate overlays, and geospatial source context.
- **NOTAM and Weather/PIREP:** keep NOTAM constraints separate from reservations while weather products and PIREPs drive route-blockage, altitude-change, delay, event drilldown, and coordination guidance.
- **Weather Pattern Mapping:** shows configuration-gated live AWC status, deterministic pattern/event grouping, route/corridor-style sampling surfaces, source-cited pattern overlays, and map filters for source family, confidence, freshness, altitude, and movement metadata.

![Login](/docs/screenshots/01-login.png)

| Mission Explorer | Agentic Ops |
| --- | --- |
| ![Mission Explorer](/docs/screenshots/02-mission-explorer.png) | ![Agentic Ops](/docs/screenshots/03-agentic-ops.png) |

| Mission Workspace | Pilot Brief |
| --- | --- |
| ![Mission Workspace](/docs/screenshots/04-mission-workspace.png) | ![Pilot Brief](/docs/screenshots/05-pilot-brief.png) |

| Reservation Sections | Reservation Supplements |
| --- | --- |
| ![Reservation Sections](/docs/screenshots/06-reservation-sections.png) | ![Reservation Supplements](/docs/screenshots/07-reservation-supplements.png) |

| Deconfliction Review | Messaging |
| --- | --- |
| ![Deconfliction Review](/docs/screenshots/08-deconfliction-review.png) | ![Messaging](/docs/screenshots/09-messaging.png) |

| USNS Feed | Decision Summary |
| --- | --- |
| ![USNS Feed](/docs/screenshots/10-usns-feed.png) | ![Decision Summary](/docs/screenshots/11-decision-summary.png) |

| Decision Trace | Decision Map |
| --- | --- |
| ![Decision Trace](/docs/screenshots/12-decision-trace.png) | ![Decision Map](/docs/screenshots/13-decision-map.png) |

| NOTAM Constraints | Weather And PIREPs |
| --- | --- |
| ![NOTAM Constraints](/docs/screenshots/14-notam-constraints.png) | ![Weather And PIREPs](/docs/screenshots/15-weather-pirep.png) |

| Weather Pattern Events | Weather Pattern Map Filters |
| --- | --- |
| ![Weather Pattern Events](/docs/screenshots/20-weather-pattern-events.png) | ![Weather Pattern Map Filters](/docs/screenshots/21-weather-pattern-map-filters.png) |

| Config / Reference Data | Agentic System Observability |
| --- | --- |
| ![Config Reference](/docs/screenshots/16-config-reference.png) | ![Agentic System Observability](/docs/screenshots/17-agentic-system.png) |

| Search | History And Audit |
| --- | --- |
| ![Search](/docs/screenshots/18-search.png) | ![History And Audit](/docs/screenshots/19-history-audit.png) |

Local product dev:

```bash
JAVA_HOME=$(/usr/libexec/java_home -v 17) mvn quarkus:dev -Dquarkus.enforceBuildGoal=false -Dquarkus.http.port=8090 -Dquarkus.test.continuous-testing=disabled
cd frontend
npm run dev -- --host 127.0.0.1
```

The Vite dev proxy defaults to `http://localhost:8090` and can be overridden with `VITE_API_PROXY_TARGET`.

### Product Replay Corpus

A self-contained product replay corpus lives under:

```text
src/test/resources/scenarios/product-replay/
```

It includes mixed USNS/weather traffic, a CARF/ALTRV reservation, and an expected summary. `ProductReplayCorpusTest` verifies the local feed adapter, transaction inspection, operational decision evaluation, audit/replay payloads, and metrics without external files or live FAA/NWS/SWIM feeds.

### Spatial / Geometry / Visualization

- Core spatial model: points, lines, polygons, circles, arcs, volumes, trajectories, time intervals, wind fields, and 3D volume primitives.
- Engine geometry service for overlap, route-corridor, distance, segment-impact, and bbox prefilter behavior.
- Pluggable topology extensions:
  - native geometry fallback,
  - JTS for precise planar topology,
  - H3 for discrete cell indexing,
  - S2 for discrete cell indexing.
- Constraint spatial index with time buckets, altitude buckets, bbox filtering, product-validity filtering, and optional discrete topology cells.
- Rendering-neutral visualization model and GeoJSON exporter. Output is generic GeoJSON-compatible feature collections that can be consumed by Leaflet, OpenLayers, Mapbox, or another browser map layer.

Key entry points:

```java
OperationalGeometryService geometry =
    new PluggableOperationalGeometryService();

AirspaceVisualizationService visualization =
    new AirspaceVisualizationService();
```

### Reservation Workflow

- Engine/domain workflow support for reservation drafts, validation, submission, approval, rejection, cancellation, completion, locks, stale locks, conflict review, and audit events.
- In-memory and JSON-file repository implementations.
- Quarkus REST resources exist for local testing and integration, but the core framework does not require HTTP or persistence to use the engine.

## Current Technology Stack

- Java 17
- Maven
- JUnit 5
- JaCoCo coverage gate
- Quarkus REST/Jackson for optional local HTTP resources
- PostgreSQL-shaped Flyway migrations and JPA entity/repository seams
- JGraphT for graph support
- JTS, H3, and S2 as pluggable spatial/topology extensions
- Jackson JSR-310 for time-aware JSON handling
- Lombok
- React, TypeScript, Vite, TanStack Query/Table, React Hook Form, Zustand, and OpenLayers for the browser console

The README previously mentioned Spring Boot, React, Kafka, RabbitMQ, Oracle, DynamoDB, MongoDB, Docker, and Kubernetes as if they were implemented. They are not part of the current checked-in runtime. Live FAA/NWS/SWIM feeds, certified cockpit UI behavior, production databases, and deployment infrastructure are future adapter/deployment concerns.

## Test Coverage And Verification

Run the full test and coverage gate:

```bash
JAVA_HOME=$(/usr/libexec/java_home -v 17) mvn verify -q
```

Current status from the latest verification run:

- Tests: 195
- Failures: 0
- Errors: 0
- Enforced JaCoCo line coverage: 77.5%
- Active framework coverage:
  - `org.tash.extensions.engine`: 85.3%
  - `org.tash.extensions.weather`: 91.6%
  - `org.tash.extensions.messaging`: 85.7%
  - `org.tash.extensions.carf`: 88.1%
  - `org.tash.extensions.notam`: 81.9%

Coverage report:

```text
target/site/jacoco/index.html
```

The coverage gate excludes walkthrough and experimental prototype surfaces that are not part of the current engine commitment, including executable walkthrough classes, experimental routing search/replan packages, the old RRT planner prototype, and large Kalman/probabilistic estimator prototypes.

## Test Fixtures

The project no longer depends on files in `~/Downloads`. Legacy evidence that remains relevant has been copied into test resources:

```text
src/test/resources/legacy/carf/
src/test/resources/legacy/grammar-parity/
src/test/resources/scenarios/weather-engine/
```

Examples include CARF reservation text, head-on and lateral-separation regressions, navaid/fix fixtures, PTR-style examples, METAR/SPECI corpora, TAF corpora, SIGMET/AIRMET corpora, CWAP-style/CWAF-like examples, PIREP examples, and mixed USNS/CARF/NOTAM/weather scenarios.

The `legacy/grammar-parity` corpus pins the useful 2010 grammar behavior without importing old generated parser runtimes or `.llr` binaries. It covers ALTRV route-family phrases from `ALTRV.g`, stationary line-corridor geometry, radius/area/timing-triangle metadata, DOM1 domestic NOTAM record shapes, DOM2 semantic reducer cases, ICAO/Canadian NOTAM field extraction, and USNS/request/table/GENOT transaction families. These fixtures are behavior oracles only: Airspace keeps modern typed parser outputs, typed diagnostics, retained raw text, restart-safe feed/search metadata, GeoJSON source-family separation, and no fake geometry for non-geometric NOTAMs.

## Useful Commands

Run all tests without coverage report generation:

```bash
JAVA_HOME=$(/usr/libexec/java_home -v 17) mvn test -q
```

Run verification with coverage:

```bash
JAVA_HOME=$(/usr/libexec/java_home -v 17) mvn verify -q
```

Build the browser console:

```bash
cd frontend && npm run build
```

Run one test class:

```bash
JAVA_HOME=$(/usr/libexec/java_home -v 17) mvn -q -Dtest=OperationalDecisionEngineTest test
```

## Project Boundaries

Implemented as engine/framework code:

- CARF/ALTRV parsing, mapping, and conflict checks.
- USNS-style message parsing and transaction routing.
- Domestic NOTAM parsing and structured NOTAM restrictions.
- Aviation weather product modeling and pragmatic decoding.
- PIREP validation and quality diagnostics.
- Route blockage and operational decision support.
- Structured decision traces, rule catalog references, audit/replay envelopes, and deterministic local signing.
- Visualization-neutral GeoJSON feature generation.
- Restart-safe product decision replay verification and persisted decision feature export.
- Local product workflow APIs for reservation parse/deconflict, message reply/forward, feed transaction inspection, and RBAC-gated operator actions.

Available only as configuration-gated prototype integrations:

- Live NOAA/AWC aviation weather polling through `LiveAviationWeatherAdapter`.
- Deterministic weather-pattern event grouping and route/corridor sampling.

Not implemented as production integrations:

- Live FAA/NWS/SWIM operational deployment adapters beyond the AWC prototype seam.
- Certified FAA/NWS decoders.
- Certified cockpit UI.
- Live database/KVM connectivity.
- Operational alert dissemination.
- Production deployment, monitoring, or infrastructure automation.
- Any authority to issue clearances, dispatch aircraft, replace official weather/NOTAM sources, or mutate operational records without human review.

## Public Review Path

If you want to help without turning this into proprietary contractor software:

1. Read [SAFETY.md](SAFETY.md) and [docs/SAFETY_WHITEPAPER.md](docs/SAFETY_WHITEPAPER.md).
2. Run the local demo from [docs/EVALUATION_GUIDE.md](docs/EVALUATION_GUIDE.md).
3. Walk through the video outline in [docs/DEMO_SCRIPT.md](docs/DEMO_SCRIPT.md).
4. File issues with concrete fixtures, screenshots, trace output, or operational concerns.
5. Contribute under [CONTRIBUTING.md](CONTRIBUTING.md), preserving source refs, diagnostics, replayability, and public availability.

Most useful review topics:

- source artifacts missing from guidance,
- NOTAM/weather/PIREP/CARF source-family confusion,
- route-impact or reroute assumptions that would mislead an operator,
- PIREP aging/relevance cases,
- weather parser edge cases,
- decision trace or pilot brief wording that overstates authority,
- realistic scenarios that should be added to `src/test/resources`.

## Repository Notes From Recent Work

The last 24 hours of commits moved the repo substantially:

- Completed the modern CARF/ALTRV, USNS/NOTAM, weather, PIREP, workflow, visualization, and operational decision engine layers.
- Removed final-product legacy evaluation utilities that referenced external artifact locations.
- Renamed the old polymorphic model runner to `AirspaceModelWalkthrough`.
- Added pluggable spatial backends for JTS/H3/S2.
- Added JaCoCo coverage enforcement and raised enforced line coverage above 75%.
- Added regression fixtures under `src/test/resources` so the project is testable from the repository alone.

See [Weather Pattern Mapping](docs/weather-pattern-mapping.md) for the live AWC adapter, deterministic pattern model, time/altitude/movement map semantics, event grouping, route sampling, and certification limits.

## Author

Tashdid Khan
