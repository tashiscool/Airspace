# FAA Weather Gap Analysis

This maps the FAA safety problem to the current Airspace product:

> The system still does not reliably turn live weather, aircraft reports,
> forecasts, ATC constraints, NOTAMs, and reservations into clear operational
> guidance fast enough.

## Coverage Matrix

| Gap | Status | Airspace evidence | Still missing |
|---|---|---|---|
| Real-time cockpit weather | Mostly covered for prototype | `frontend/src/pages/ExplorerPage.tsx` and `frontend/src/pages/WeatherPage.tsx` show weather/PIREP guidance, latest deltas, affected missions, freshness, time-to-guidance metrics, source-family drilldown chips, and map features extracted from coordinate-bearing SIGMET/AIRMET/CWAP-style text with validity timing and movement vectors when present. Weather table/guidance selection now drives map selection for matching weather features. `frontend/src/pages/MissionPage.tsx` surfaces per-mission weather verdicts. `frontend/src/pages/PilotBriefPage.tsx` provides a printable/read-only handoff brief. | No certified live weather feed adapter, no cockpit-certified display, and no per-tail avionics integration. |
| Turbulence and icing prediction | Partial | `frontend/src/lib/viewModels.ts` classifies PIREP `/TB` and `/IC`, AIRMET turbulence/icing, routes them into caution/altitude-change guidance, and converts coordinate-bearing weather text into map-ready weather features with TOP/FL/SFC/ABV/BLW altitude metadata. `OperationsMap` turns confidence into readable labels, visually softens/dashes low-confidence overlays, thickens severe/extreme overlays, and shows a combined severity/confidence/freshness risk readout for selected features. Backend route-impact APIs expose source refs, confidence, affected segments, and PIREP relevance. | Needs operationally calibrated turbulence/icing models and richer altitude-band exposure scoring. |
| Route avoidance | Mostly covered for prototype | Engine route-impact and avoidance APIs exist. `frontend/src/pages/MissionPage.tsx` and `frontend/src/pages/ReservationPage.tsx` expose route impact, impacted segments, source-family chips for NOTAM/weather/PIREP/CARF refs, and avoidance candidates. `frontend/src/pages/DecisionPage.tsx` exposes blocking constraints, route predictions, avoidance candidates, source refs, and replay. | Avoidance candidates are deterministic prototype suggestions, not a certified routing solver tied to live NAS sector demand. |
| PIREPs | Mostly covered for prototype | `WeatherPage` separates PIREPs from forecast products. `frontend/src/components/WeatherVerdict.tsx` links PIREP-like sources into mission verdicts. `MissionPage` exposes relevant PIREPs with operator-adjustable reservation route, altitude band, altitude tolerance, recency, and corridor buffer controls. Backend relevance results include age, aging category, stale state, and relevance score; excluded reports remain visible as decayed context. `OperationsMap` fades/dashes stale reports and can filter weather/PIREP overlays by max age or stale state. | Needs historical calibration for the deterministic aging/decay coefficients. |
| ATC/weather coordination | Mostly covered for prototype | Weather verdicts provide coordination recommendations. `MissionPage` and `WeatherPage` expose coordinate actions that prefill USNS/message drafts with mission, action, impact, and source refs. | Needs real recipient directory/position mapping and live messaging adapter delivery confirmation. |
| Pilot decision support | Mostly covered for prototype | `DecisionPage` exposes operational action, trace, blocking constraints, typed/clickable source refs, audit envelope, replay, map, route predictions, and avoidance candidates. Explorer, Mission, Weather, and Pilot Brief pages now show verdicts, deltas, affected missions, and guidance freshness. The pilot brief groups source drivers by family, prints them in the handoff brief, and shows route-impact source chips so a pilot/controller can see exactly whether guidance came from NOTAM, PIREP, weather, CARF/ALTRV, or USNS traffic. `ProductReplayCorpusTest` verifies the local product replay path across mission verdict, route impact, PIREP relevance, NOTAM-as-constraint source handling, coordination draft, pilot brief, affected missions, metrics, persisted feed artifacts, and auditable decisions. | Needs larger real-world corpus, calibrated confidence models, and operational replay validation against historical outcomes. |

## Acceptance Signals

1. **Time to guidance:** a new SIGMET/PIREP/feed artifact should highlight affected missions/routes in under 5 seconds in local product mode.
2. **Per-mission verdict:** each active mission should show CLEAR / MONITOR / CAUTION / DELAY / ALTITUDE CHANGE / AVOID / REROUTE / BLOCKED with confidence and source artifacts.
3. **Route avoidance:** selected route + altitude band + time window should list intersecting hazards and at least one deviation/avoidance recommendation when possible.
4. **What changed:** Weather and Mission Explorer should expose recent weather/PIREP/NOTAM deltas since the last brief.
5. **PIREP relevance:** operators should be able to scope PIREPs to selected route corridor, altitude band, and recency.
6. **Explainability:** every AVOID / REROUTE / BLOCKED decision should list exact weather, PIREP, NOTAM, CARF/ALTRV, and USNS source IDs in trace/audit surfaces.

## Copied Or Mimicked From `airspace-ops-console`

- Per-mission weather verdict UX, backed by Airspace product APIs instead of mock hash-based verdicts.
- A verdict strip on mission detail with action, confidence, recommended coordination, and contributing source artifacts.
- A weather delta feed, affected-mission queue, time-to-guidance metrics, and weather guidance table on the Weather page.
- Source-family drilldown chips in affected-mission guidance on Weather and Mission Explorer so operators can jump from mission verdict to the NOTAM, PIREP, weather, USNS, or CARF/ALTRV source.
- Clickable route-impact source chips on Mission, Reservation, and Pilot Brief surfaces so route guidance can be traced directly back to its driving artifact.
- Weather-map extraction for common compact aviation coordinates, decimal lat/lon pairs, bounded-by/from-to polygons, validity, altitude, radius, line-corridor, and movement tokens in SIGMET/AIRMET/CWAP-style text, with stable geometry intent metadata and selected map features linking back to their source artifact; non-geometric METAR/TAF products remain table/guidance artifacts instead of fake shapes.
- Selected map feature summaries now expose geometry intent such as point radius or line-corridor width, so operators can distinguish a polygon, route corridor, and radius hazard without reading raw properties.
- The map now includes an active-layer risk readout for visible blocking, severe, low-confidence, and stale overlays, making filter state and current operational risk easier to scan.
- The active-layer risk readout is driven by the same tested frontend helper used for map severity, confidence, and freshness semantics, so counts reflect only currently visible/filter-passing overlays.
- Map feature detail also exposes backend-emitted `sourceRefs` as clickable chips, so route-impact/intersection overlays can trace to weather, PIREP, NOTAM, USNS, or CARF/ALTRV sources even when the source was not a frontend-derived weather message.
- Decision source-ref visibility so operators can answer “why this action?” without reading raw JSON first.
- Coordinate-from-hazard deep links into the real Airspace messaging page.
- Printable pilot brief route for mission handoff, including grouped source-driver summaries, route-impact source-family chips, and clickable source artifacts back into the workbench.

## Highest-Value Backlog

1. Calibrate route blockage, turbulence, icing, confidence, and capacity-impact scoring against historical outcomes.
2. Expand product replay corpus with more real-world mixed USNS + CARF/ALTRV + NOTAM + METAR/TAF + SIGMET/AIRMET + PIREP cases; keep NOTAM constraints separate from reservations.
3. Calibrate PIREP aging/decay coefficients and route-risk effect against operational outcomes.
4. Add certified-feed adapter implementations when authoritative FAA/NWS/SWIM/KVM access is available.
5. Harden live recipient/role mapping for coordinate-from-hazard delivery confirmations.
6. Continue validating backend source refs in `Decision.trace` against larger mixed operational corpora.
