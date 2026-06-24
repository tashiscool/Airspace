# National Demand / Capacity Simulation

Airspace now includes a local national-scale demand/capacity simulation layer. It moves beyond a few affected missions by generating hundreds or thousands of synthetic flights across airports and sectors, projecting demand against representative capacity, and producing advisory TFM recommendations when demand exceeds capacity.

This is a production-prototype simulation seam, not live NAS traffic and not an FAA operational flow-management tool.

## Public Research Basis

- FAA SWIM is the near-real-time NAS data exchange layer for aeronautical, flight, weather, and surveillance information: <https://www.faa.gov/air_traffic/technology/swim>
- FAA FMDS describes the modernization direction for traffic-flow management: balancing demand and capacity using schedules, flight plans, position updates, weather, airspace constraints, airport capacity, and collaborative reroute/flow tools: <https://www.faa.gov/air_traffic/technology/fmds>
- FAA TFM learning material defines ATFM as balancing traffic demand with system capacity, where demand is flights planned or in operation and capacity is the number of flights safely handled per time interval. It also identifies constraints such as weather, runway configuration, equipment, and staffing: <https://tfmlearning.faa.gov/tfm-training/atfm-basics/cdm-t1-lesson1b.html>
- FAA TFM training covers common traffic-management initiatives such as GDPs, AFPs, ground stops, reroutes, miles-in-trail, and departure metering: <https://tfmlearning.faa.gov/tfm-training/atfm-basics/cdm-t1-lesson3b.html>

## Implemented Concepts

`NationalDemandCapacityConfig` controls a deterministic synthetic run:

- `flightCount`
- `airportCount`
- `sectorCount`
- `durationMinutes`
- `tickIntervalMinutes`
- `randomSeed`
- `demandSpikeFactor`
- `capacityReductionFactor`
- `includeWeatherCapacityReduction`
- optional airport/sector ID lists

`NationalDemandCapacitySimulator` produces:

- `NationalDemandCapacityReport`
- `NationalDemandCapacitySnapshot` per tick
- a generated `TrafficReplayBundle`
- synthetic flight plans and aircraft position tracks
- airport departure/arrival demand and capacity snapshots
- sector active-aircraft/capacity/frequency snapshots
- generated TFM primitives and recommendations when demand exceeds capacity

The simulation run path can consume this report through:

```json
{
  "scenarioId": "sector-capacity-compression",
  "durationMinutes": 60,
  "tickIntervalSeconds": 300,
  "nationalDemandCapacityConfig": {
    "id": "nas-scale-demo",
    "flightCount": 1000,
    "airportCount": 12,
    "sectorCount": 24,
    "durationMinutes": 180,
    "tickIntervalMinutes": 15,
    "randomSeed": 20260623
  }
}
```

Preview-only endpoint:

```http
POST /api/simulations/national-demand/preview
```

Full run endpoint:

```http
POST /api/simulations/run
```

## Runtime Integration

When `nationalDemandCapacityConfig` is supplied:

1. The simulator generates a national report.
2. The report's `trafficReplay` becomes the run's traffic replay source.
3. Per-tick aircraft, airport demand, sector demand, and TFM recommendations are computed from the synthetic national replay.
4. `SimulationDynamicsSnapshot.nationalDemandCapacity` exposes the current aggregate snapshot.
5. `SimulationTick.nationalDemandCapacity` stores the same snapshot for replay.
6. `SimulationKpiSummary` includes national flight count, overloaded airport/sector counts, peak airport and sector demand/capacity ratios, and total national TFM recommendation count.

## Non-Goals

- No live SWIM/FMDS/TFMS/NAS traffic is used.
- No official airport acceptance rates or sector capacities are claimed.
- No calibrated sector workload model is claimed.
- No automatic GDP/AFP/ground-stop/reroute issuance occurs.
- No certified operational safety assurance is implied.

Future authorized provider adapters should map real schedule/position/demand/capacity data into `TrafficReplayBundle` and preserve provider receipts, source freshness, consent scope, and egress policy before the simulation output is used as evidence.
