# Recorded Traffic Replay Format

Airspace supports a local, recorded traffic replay format for simulations. It is designed to mimic the information shape of SWIM/TFMS/FMDS-style traffic-flow data while remaining fixture-backed and safe for public demos.

This is not a live SWIM connection, not authorized NAS traffic, and not operational evidence by itself.

## Public Research Basis

- FAA SWIM is the exchange layer for near-real-time aeronautical, flight, weather, and surveillance information across the NAS and aviation community: <https://www.faa.gov/air_traffic/technology/swim>
- FAA SWIM access is managed through the SWIFT portal and SWIM/NEMS connectivity; some services require request/review before access: <https://www.faa.gov/air_traffic/technology/swim/products/get_connected>
- FAA FMDS describes the traffic-flow modernization direction: replacing TFMS, balancing demand and capacity, using schedules, flight plans, position updates, weather, airspace constraints, airport capacity, and collaborative reroute/flow tools: <https://www.faa.gov/air_traffic/technology/fmds>
- FAA TFDM describes surface/tower flow modernization and collaborative departure/surface operations: <https://www.faa.gov/air_traffic/technology/tfdm>
- FAA ATFM/CDM training describes demand as planned/in-operation flights, capacity as flights safely handled per time window, and constraints such as weather, runway configuration, facility/equipment, and staffing: <https://tfmlearning.faa.gov/tfm-training/atfm-basics/cdm-t1-lesson1b.html>

Airspace uses those public concepts to define a local replay contract. It does not copy a proprietary operational schema and does not claim parity with an authorized SWIM service.

## Core Contract

`TrafficReplayBundle` can be provided in:

- `SimulationRunRequest.trafficReplay`
- `ScenarioBundle.trafficReplay`

It can be validated through:

```http
POST /api/simulations/traffic-replay/validate
```

The bundle includes:

- replay identity: `id`, `sourceId`, `sourceMode`, `providerFamily`, `providerReceiptId`, `authorizationMode`
- schedule/plan data: `TrafficReplayFlightPlan`
- position data: `TrafficReplayPosition`
- airport demand/capacity snapshots: `TrafficReplayAirportDemand`
- sector demand/workload snapshots: `TrafficReplaySectorDemand`
- traffic management initiatives: `TrafficManagementInitiative`
- retained assumptions and diagnostics

Use `sourceMode=LOCAL_FIXTURE_REPLAY` for checked-in fixtures. Future provider adapters may use different modes only when credentials, consent, egress policy, provider receipts, and reviewer approval are present.

## Simulation Mapping

Per tick, `OperationalSimulationService` resolves replay in this order:

1. request-level `trafficReplay`
2. imported scenario bundle `trafficReplay`
3. generated local fixture replay from the scenario route

Replay fields drive simulator state as follows:

| Replay field | Simulation output |
|---|---|
| `flightPlans` | `TrafficFlowScenario`, tick-level `SimulatedAircraft.flightPlan` |
| `positions` | `SimulationAircraftState`, `AircraftTrajectoryState`, map/replay aircraft state |
| `airportDemand` | departure queue, surface delay, runway configuration context |
| `sectorDemand` | active aircraft, baseline capacity, handoff queue, frequency congestion, workload state |
| `trafficManagementInitiatives` | typed TFM primitives, affected aircraft review markers, active TMI counts, and recommendation output |
| `sourceRefs` | source-cited simulation trace and replay diagnostics |

`SimulationTrafficReplayState` records replay counts, active TMI count/types, TMI recommendation count, source mode, provider family, authorization mode, fixture-backed status, and `liveSwimNasDataUsed`.

Flights are considered active at a simulation tick only after the first recorded `TrafficReplayPosition` at or before that tick. Future scheduled flight plans stay in the replay bundle, but they do not inflate active aircraft, sector workload, or TMI review counts before position evidence exists.

See [TRAFFIC_FLOW_MANAGEMENT_PRIMITIVES.md](TRAFFIC_FLOW_MANAGEMENT_PRIMITIVES.md) for the GDP, AFP, FEA/FCA, miles-in-trail, reroute advisory, ground stop, departure metering, arrival rate, sector capacity, and TMI recommendation model catalog.

## Non-Goals

- No live SWIM/NEMS transport.
- No FAA credential, agreement, or operational source activation.
- No certified traffic-flow, controller workload, or capacity model.
- No guarantee that local replay fields match a specific authorized FAA message schema.
- No automatic official workflow mutation or external send.

The intended next step for authorized integration is an adapter that converts provider-specific traffic messages into `TrafficReplayBundle`, retaining provider receipts and policy metadata before simulation or decision replay uses the data.
