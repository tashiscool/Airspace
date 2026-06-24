# Historical Replay And Calibration Corpus

Airspace now treats historical replay as a first-class simulation input, while
staying honest about data authority. The current corpus is local and
fixture-backed; it is shaped so synthetic days, public historical-like
reconstructions, and future authorized operational data can use the same
contract.

## Public Data Shape

The replay format is designed around public and authorized aviation data
families:

- FAA OPSNET is the official FAA source for NAS operations and delay data. Public
  users can access finalized monthly operations and delay data after the public
  release window.
- BTS TranStats provides historical scheduled/actual domestic flight performance
  fields that can act as public schedule and delay proxies.
- NOAA/NCEI ASOS/AWOS archives provide historical airport weather observations,
  including visibility, sky condition, present weather, pressure, temperature,
  and wind.
- Aviation Weather Center archive tooling supports recent product-time
  inspection and points users to NCEI for official archived weather records.
- FAA SWIM and TFMS are the future authorized live/near-real-time flight,
  weather, surveillance, and traffic-flow data sources. They remain
  credential/authorization gated.

## Core Types

- `TrafficReplayBundle`: recorded flight plans, positions, airport demand,
  sector demand, traffic management initiatives, provider metadata, and source
  refs.
- `HistoricalReplayDay`: wraps a replay bundle with operating date, scenario
  link, source mode, authorization mode, airports/sectors, public source refs,
  data-quality warnings, assumptions, and expected outcomes.
- `HistoricalReplayOutcome`: labeled calibration target such as route-weather
  impact, PIREP override, NOTAM/procedure ambiguity, or sector capacity
  compression.
- `HistoricalReplayCalibrationReport`: aggregate readiness report with outcome
  counts, deterministic agreement labels, source-ref preservation, source modes,
  and uncalibrated coefficient gaps.

## Source Modes

- `SYNTHETIC`: fully local deterministic fixture.
- `PUBLIC_HISTORICAL_LIKE`: structured to resemble public historical sources,
  but not a downloaded authoritative operational day.
- `AUTHORIZED_OPERATIONAL`: reserved for future data loaded through approved
  SWIM/TFMS/ASPM/NCEI/provider channels with receipts, consent scope, and review.

No current corpus day is treated as authorized operational evidence.

## API Surface

- `GET /api/simulations/historical-replay/days`
- `GET /api/simulations/historical-replay/days/{id}`
- `POST /api/simulations/historical-replay/load`
- `POST /api/simulations/historical-replay/calibrate`

`load` accepts a named day or ad hoc `TrafficReplayBundle`. When
`runSimulation` is true, the selected replay becomes the traffic source for the
normal simulation engine and produces the same timeline, world state, map
features, KPIs, replay hashes, coordination drafts, and pilot briefs as any
other scenario run.

## Current Corpus

- `public-like-jfk-lowvis-opsnet-bts-awc`: public-historical-like low visibility
  reconstruction using OPSNET/BTS/NCEI/AWC-shaped references and labeled
  RVR/SMGCS/NOTAM/procedure outcomes.
- `synthetic-oceanic-convection-route-blockage`: synthetic ALTRV plus moving
  convection replay with route blockage and sector capacity labels.
- `synthetic-pirep-override-weather-day`: synthetic severe PIREP override replay
  with PIREP aging/relevance labels.

## Future Authorized Swap Path

1. Load authorized data through provider adapters.
2. Preserve provider receipt IDs, authorization mode, source refs, freshness,
   consent scope, egress policy, and data quality diagnostics.
3. Normalize provider-specific payloads into `TrafficReplayBundle`.
4. Attach expected outcome labels in `HistoricalReplayDay`.
5. Run local simulation/campaigns.
6. Generate `HistoricalReplayCalibrationReport` and safety dossier evidence.
7. Keep all calibration claims disabled until authoritative datasets and
   independent review support them.

## Non-Claims

This corpus does not certify route blockage scoring, CWAP-style/CWAF-like calibration,
PIREP aging coefficients, sector capacity models, or operational traffic-flow
actions. It is a production-shaped prototype interface for replay, regression,
and future authorized calibration.
