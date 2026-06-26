# Airspace Simulation Guide

Airspace now includes a local operational decision simulator. It is designed to show how aviation inputs become source-cited guidance over time; it is not a flight dynamics simulator, certified dispatch tool, cockpit avionics display, or live NAS traffic simulator.

## What Is Simulated

- Scheduled USNS/feed/weather/PIREP/NOTAM/CARF/operator events.
- A true minute-by-minute simulation clock between scheduled source events.
- Mission and reservation context.
- Aircraft state approximations: position along the route, altitude, speed, climb/cruise phase, fuel burn, runway required/available, and takeoff-performance review.
- Airport surface state approximations: RVR, runway, braking action, SMGCS/LVO/LVP ambiguity, departure queue, and surface delay.
- Sector/ATC workload approximations: active aircraft, baseline capacity, staffed controller positions, handoff queue, handoff delay, and frequency congestion.
- Pilot/controller/operator behavior approximations: accepts reroute, requests clarification, rejects blocked release, asks for ride/altitude reports, or continues monitoring.
- Deterministic ensemble-style weather evolution fields: movement, growth/decay, storm phase, blocked probability mean, probability spread, and ensemble member count.
- Local SWIM/TFMS-like traffic replay bundles: flight plans, ETAs/ETDs, aircraft position snapshots, filed routes, airport demand/capacity, sector workload, and traffic management initiatives, all with explicit `liveSwimNasDataUsed=false` unless a future authorized provider changes the source mode.
- National-scale synthetic demand/capacity simulation: hundreds or thousands of generated flights across airport and sector resources, with per-tick overload snapshots and advisory TFM recommendations.
- Per-timestep mission verdicts.
- Route-impact and reroute candidate snapshots.
- Source refs and affected-mission deltas.
- Coordination draft generation with human-review semantics.
- Pilot brief generation.
- Map playback using the same CARF/ALTRV, NOTAM, weather, PIREP, route-impact, conflict, and reference layer semantics as the workbench.
- KPI reports for time-to-guidance, false clear/block checks, source-ref preservation, reroute availability, stale-data downgrade, coordination latency, pilot brief availability, and replay/audit ID retention.
- Large campaign controls for scenario repetitions and simulated-day count. These create deterministic sensitivity sweeps across forecast confidence, movement speed, and communication delay.
- Scenario bundle authoring, validation, import, and export-ready JSON at `/simulation/author`.
- Replay bundle/world-state inspection at `/simulation/runs/{runId}/replay`, including aircraft, airport, sector, behavior, source refs, map playback, and replay hashes.
- Safety dossier export for campaign runs with assumptions, KPI summaries, replay hashes, known gaps, and non-certification warnings.
- Advisory autonomous workloads that generate draft scenario bundles and red-team unsafe guidance patterns. Drafts require human import/review and cannot mutate official workflow state or send external messages.
- Swappable map renderer seam in the frontend. OpenLayers is the active renderer; Leaflet, Cesium, deck.gl, and generic GeoJSON are selectable adapter seams that receive the same filtered GeoJSON contract.

## What Is Not Simulated

- Certified aircraft flight dynamics.
- Certified aircraft-specific takeoff, landing, climb, descent, or performance minima.
- Certified cockpit weather display behavior.
- Official live FAA/SWIM/FNS/NMS/NADIN/WMSCR/KVM traffic.
- Real sector workload, staffing, controller frequency congestion, or operational demand.
- Historical CWAP-style/CWAF-like calibration or official route blockage validation.
- Automatic official message delivery or workflow state mutation.

## Running A Scenario

1. Start the backend and frontend using the README run steps.
2. Open `/simulation`.
3. Select a scenario from the library or capability presets.
4. Click **Run Scenario**.
5. Scrub the minute-by-minute timeline and inspect:
   - injected event,
   - aircraft performance state,
   - airport surface/RVR state,
   - sector workload and handoff delay,
   - pilot/controller/operator behavior,
   - weather evolution and ensemble spread,
   - traffic replay source mode,
   - map playback,
   - mission verdict,
   - route impact,
   - coordination draft,
   - pilot brief,
   - KPI report.

The REST entrypoint is:

```http
POST /api/simulations/run
Content-Type: application/json

{
  "scenarioId": "low-vis-rvr-smgcs",
  "actor": "simulation-operator",
  "includeSensitivity": true,
  "sensitivityOverrides": {
    "weatherMovementSpeedKt": 35,
    "forecastConfidence": 0.74,
    "communicationDelaySeconds": 45,
    "ensembleMembers": 24
  }
}
```

Optional run controls include:

- `tickIntervalSeconds`
- `durationMinutes`
- `randomSeed`
- `aircraftFleet`
- `trafficReplay`
- `nationalDemandCapacityConfig`
- `weatherEnsembleConfig`
- `sectorWorkloadModel`
- `airportOpsTimeline`

## Recorded Traffic Replay Format

Airspace can now run local, recorded traffic-flow replay bundles that mimic the operational shape of SWIM/TFMS/FMDS-style traffic data without connecting to SWIM. Public FAA material describes SWIM as the NAS exchange layer for near-real-time aeronautical, flight, weather, and surveillance data; TFMS/FMDS-style traffic-flow systems balance demand and capacity using schedules, flight plans, position updates, weather, airspace constraints, and airport/sector capacity. Airspace uses that public shape only: checked-in replay is local fixture data, not an authorized operational feed.

Traffic replay bundles can be supplied in a `POST /api/simulations/run` body as `trafficReplay`, included in a scenario bundle, or inspected with:

```http
POST /api/simulations/traffic-replay/validate
```

The core JSON shape is:

```json
{
  "id": "tfms-like-jfk-lowvis",
  "sourceId": "recorded-swim-fixture:jfk-lowvis",
  "sourceMode": "LOCAL_FIXTURE_REPLAY",
  "providerFamily": "TFMS_LIKE_RECORDED_REPLAY",
  "timeBasis": "OFFSET_MINUTES",
  "authorizationMode": "LOCAL_FIXTURE_ONLY",
  "flightPlans": [
    {
      "flightId": "BAW1",
      "callsign": "SPEEDBIRD1",
      "aircraftClass": "HEAVY_JET",
      "origin": "KJFK",
      "destination": "EGLL",
      "filedRouteText": "KJFK DCT ACK NAT",
      "filedRoutePoints": [[40.64, -73.78, 0], [41.0, -72.7, 15000]],
      "scheduledDepartureTime": "2026-06-20T12:00:00Z",
      "estimatedDepartureTime": "2026-06-20T12:08:00Z",
      "estimatedArrivalTime": "2026-06-20T19:00:00Z",
      "requestedAltitudeBlock": "FL330-FL370",
      "filedSpeedKnots": 470,
      "sourceRefs": ["TFMS_FLIGHT:BAW1"]
    }
  ],
  "positions": [
    {
      "flightId": "BAW1",
      "offsetMinutes": 5,
      "latitude": 40.70,
      "longitude": -73.60,
      "altitudeFeet": 6000,
      "groundSpeedKnots": 260,
      "routeProgress": 0.2,
      "phase": "DEPARTURE",
      "sourceRefs": ["TRACK:BAW1:T5"]
    }
  ],
  "airportDemand": [
    {
      "airportId": "KJFK",
      "offsetMinutes": 5,
      "departureDemandPerHour": 38,
      "arrivalDemandPerHour": 26,
      "departureCapacityPerHour": 10,
      "arrivalCapacityPerHour": 16,
      "departureQueueDepth": 11,
      "averageDelaySeconds": 480,
      "runwayConfiguration": "04R/04L"
    }
  ],
  "sectorDemand": [
    {
      "sectorId": "ZNY-N90",
      "offsetMinutes": 5,
      "activeAircraft": 35,
      "baselineCapacity": 28,
      "handoffQueueDepth": 7,
      "frequencyUtilization": 0.93,
      "estimatedHandoffDelaySeconds": 155
    }
  ],
  "trafficManagementInitiatives": [
    {
      "id": "TMI-LOWVIS",
      "type": "GDP",
      "reason": "Low visibility departure compression",
      "startOffsetMinutes": 2,
      "endOffsetMinutes": 20,
      "affectedFlightIds": ["BAW1"],
      "sourceRefs": ["TMI:TMI-LOWVIS"]
    }
  ]
}
```

During each tick, the simulator prefers request-provided `trafficReplay`, then scenario-bundle replay, then generated local replay. Replay positions drive `SimulationAircraftState` and tick-level `SimulatedAircraft`; airport demand drives queue and surface delay; sector demand drives workload ratio, handoff queue, frequency congestion, and capacity state; active TMIs mark affected aircraft as needing review. The `SimulationTrafficReplayState` records source mode, provider family, replay counts, active TMI count/types, recommendation count, fixture-backed status, and the live-SWIM flag for audit/readout. The typed TFM primitive catalog is documented in [TRAFFIC_FLOW_MANAGEMENT_PRIMITIVES.md](TRAFFIC_FLOW_MANAGEMENT_PRIMITIVES.md).

Use `LOCAL_FIXTURE_REPLAY` for public demos and tests. Future authorized providers can target the same `TrafficReplayBundle` contract, but the adapter must retain provider receipts, authorization mode, egress/consent policy, and source freshness before it can be treated as operational evidence.

Replay and world-state endpoints are:

```http
GET /api/simulations/runs/{runId}/world-state
GET /api/simulations/runs/{runId}/replay
```

National demand/capacity preview:

```http
POST /api/simulations/national-demand/preview
Content-Type: application/json

{
  "id": "nas-scale-demo",
  "flightCount": 1000,
  "airportCount": 12,
  "sectorCount": 24,
  "durationMinutes": 180,
  "tickIntervalMinutes": 15,
  "randomSeed": 20260623
}
```

See [NATIONAL_DEMAND_CAPACITY_SIMULATION.md](NATIONAL_DEMAND_CAPACITY_SIMULATION.md) for the local NAS-scale model and its non-goals.

## Running A Campaign

Campaigns run multiple local scenarios and aggregate KPI results.

```http
POST /api/simulations/campaign
Content-Type: application/json

{
  "scenarioIds": [
    "low-vis-rvr-smgcs",
    "pirep-safety-override",
    "blocked-no-viable-reroute"
  ],
  "actor": "simulation-campaign",
  "includeSensitivity": true,
  "iterationsPerScenario": 10,
  "simulatedDayCount": 30
}
```

Campaign dossier export:

```http
GET /api/simulations/campaigns/{campaignId}/dossier
```

## Authoring And Agentic Review

Scenario bundles can be inspected, validated, and imported without requiring a live feed:

```http
GET /api/simulations/scenarios/{scenarioId}/bundle
POST /api/simulations/scenarios/validate
POST /api/simulations/scenarios/import
```

Autonomous workloads are advisory:

```http
POST /api/simulations/agents/generate-scenarios
POST /api/simulations/agents/red-team
GET /api/agents/workloads
POST /api/agents/safety-lab
POST /api/agents/airspace/run
GET /api/agents/airspace/runs
GET /api/agents/airspace/runs/{runId}
POST /api/agents/airspace/tasks/{taskId}/acknowledge
POST /api/agents/airspace/tasks/{taskId}/resolve
```

Generated scenario bundles are drafts only. A human operator must review and import them before they become part of the scenario library or regression corpus.

The broader Airspace Safety Lab agent catalog covers unsafe-guidance red-team review, outcome-metrics auditing, draft scenario generation, TMI recommendation review, brief-delta comparison, replay-integrity checks, calibration curation, national-demand stress review, collaborative-decision facilitation, provider-freshness monitoring, and coordination-draft review. These runs are local/replay-first, evidence-cited, cost-bounded, and guarded by `NO_EXTERNAL_SEND`, `NO_OFFICIAL_MUTATION`, and `HUMAN_APPROVAL_REQUIRED` policy metadata. See [AIRSPACE_SAFETY_LAB_AGENTS.md](AIRSPACE_SAFETY_LAB_AGENTS.md).

The frontend surfaces this evidence in two places:

- The global Agentic Ops drawer can run the catalog and inspect findings, policy guards, replay references, MCP receipts, and human-review tasks from any workbench route.
- Page-level Safety Lab context panels appear on Simulation, TFM, Outcomes, Decision, Pilot Brief, and Config so operators can see the agent runs and review tasks that are specific to the artifact they are inspecting.

These panels are evidence/readout surfaces only. They do not authorize external delivery, mutate official workflow state, or make certification claims.

## Scenario Corpus

Scenario fixtures live under:

```text
src/test/resources/scenarios/simulation/
```

Each `scenario.json` includes the scenario identity, expected final guidance, event timeline, expected source families, KPI bounds, and notes about what the case is meant to demonstrate.

Canonical scenarios:

- `low-vis-rvr-smgcs`
- `oceanic-altrv-convection`
- `refuel-icing-altitude-separated`
- `pirep-safety-override`
- `volcanic-ash-oceanic`
- `runway-surface-contamination`
- `sector-capacity-compression`
- `blocked-no-viable-reroute`
- `viable-reroute-residual-risk`

## Historical Replay And Calibration Corpus

Historical replay is modeled as a named day wrapper around `TrafficReplayBundle`.
The current corpus includes synthetic days and public-historical-like days that
match public FAA/BTS/NOAA/AWC data shapes without claiming authorized
operational evidence.

```http
GET /api/simulations/historical-replay/days
GET /api/simulations/historical-replay/days/{dayId}
POST /api/simulations/historical-replay/load
POST /api/simulations/historical-replay/calibrate
```

Use `load` with `runSimulation=true` to turn a named replay day into a normal
simulation run. The resulting run has the same timeline, world state, map
features, traffic replay state, route impact, coordination draft, pilot brief,
KPI, and replay-hash surfaces as hand-authored scenarios.

The calibration report is a readiness artifact. It counts outcome labels,
source-ref preservation, route/weather/PIREP/NOTAM/capacity labels, and
uncalibrated coefficient gaps. It must not be represented as historical model
validation until authorized datasets are loaded and independently reviewed.

For the full format and source-mode contract, see
[HISTORICAL_REPLAY_CORPUS.md](HISTORICAL_REPLAY_CORPUS.md).

## KPI Interpretation

- **Time to guidance:** first non-clear/non-monitor advisory in the local timeline.
- **False clear / false block:** deterministic comparison against the scenario’s expected final action.
- **Source-ref preservation:** share of timesteps retaining at least one source reference.
- **Reroute found:** share of timesteps with an avoidance candidate.
- **Stale-data downgrade:** count of stale-product steps downgraded to monitor/review.
- **Coordination draft latency:** synthetic operator review latency from the first guidance point.
- **Pilot brief availability:** whether a handoff brief is generated during the run.
- **Replay pass rate:** whether replay/audit IDs are retained in each timestep.
- **Minute steps:** number of clock ticks returned by the run.
- **Aircraft updates:** number of timesteps with aircraft performance state.
- **Peak sector workload:** maximum simulated active-aircraft-to-capacity ratio.
- **Max handoff delay:** largest simulated controller/sector handoff delay.
- **Max surface delay:** largest simulated airport surface/RVR delay.
- **Ensemble members:** maximum stochastic weather ensemble size used by a run.
- **Traffic replay aircraft:** maximum local replay aircraft count in a run.

These KPIs are useful for local regression and product review. They are not operational safety assurance without authorized data, calibration, and independent evaluation.
