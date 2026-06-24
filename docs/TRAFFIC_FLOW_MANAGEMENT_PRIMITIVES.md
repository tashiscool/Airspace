# Traffic Flow Management Primitives

Airspace models Traffic Flow Management (TFM) primitives as local/replay-first concepts. They are meant to support simulation, scenario authoring, recommendation review, and eventual authorized-provider adapters. They are not operational FAA directives.

## Public Terminology Basis

- FAA facility guidance lists TMIs such as minutes-in-trail, fix balancing, airborne holding, departure sequencing, GDP, AFP, CTOP, reroutes, and ground stops, and notes that shared FCA/FEA data improves coordination: <https://www.faa.gov/air_traffic/publications/atpubs/foa_html/chap18_section_7.html>
- FAA TFM learning material describes miles-in-trail as required spacing between aircraft over a fix/sector/route, MDI/DSP as departure-time assignment to create a constant flow, GDP as holding aircraft at departure due to destination constraints, and ground stops as highly restrictive capacity-reduction measures: <https://tfmlearning.faa.gov/tfm-training/atfm-basics/cdm-t1-lesson3b.html>
- FAA AFP training describes AFPs as en route demand/capacity programs built around FCA/FEA analysis, aggregate demand lists, acceptance rates, and EDCTs. It distinguishes GDP control at an airport from AFP control at a geographic/NAS element: <https://tfmlearning.faa.gov/tfm-training/atfm-basics/files/pdf/Appendix_C_Airspace%20Flow%20Program.pdf>
- FAA TFM background material defines FEA/FCA as three-dimensional airspace volumes plus filters/time intervals, reroutes as routings other than the filed flight plan, EDCT as the controlled runway release time, and MIT as required spacing above minimum separation: <https://faa-tfm-aid.nianet.org/wp-content/uploads/Traffic-Flow-Management-Background_v3.9-1.pdf>

## Implemented Model Catalog

| Concept | Airspace model |
|---|---|
| GDP | `TrafficManagementInitiativeType.GDP` plus `TrafficFlowProgramModel` |
| AFP | `TrafficManagementInitiativeType.AFP` plus `TrafficFlowProgramModel` and `FlowEvaluationAreaModel` |
| FEA/FCA | `FlowEvaluationAreaModel` with `areaType=FEA/FCA`, geometry, filters, altitude, time window, and acceptance rate |
| Miles-in-trail / minutes-in-trail | `MilesInTrailRestrictionModel` |
| Reroute advisory / required reroute | `RerouteAdvisoryModel` |
| Ground stop | `GroundStopModel` |
| Departure metering / MDI / DSP | `DepartureMeteringModel` |
| Arrival rate / airport acceptance rate | `ArrivalRateModel` |
| Sector capacity | `SectorCapacityModel` |
| Recommendation output | `TmiRecommendationModel` |

`TrafficManagementInitiative` remains the envelope that can carry one or more of these details plus source refs, affected flight IDs, status, scope, start/end offsets, expected delay, and confidence.

## Simulation Integration

`TrafficReplayBundle.trafficManagementInitiatives` is evaluated at each simulation tick:

1. Active initiatives are filtered by the tick offset.
2. The adapter resolves a canonical `TrafficManagementInitiativeType`.
3. Affected aircraft receive a review assignment:
   - `GROUND_STOP_REVIEW`
   - `EDCT_OR_METERING_REVIEW`
   - `SPACING_RESTRICTION_REVIEW`
   - `REROUTE_ADVISORY_REVIEW`
   - `TMI_REVIEW`
4. `TrafficReplayAdapter.recommendationsAtMinute(...)` produces `TmiRecommendationModel` records from active TMIs plus replay demand/capacity snapshots.
5. `SimulationDynamicsSnapshot.trafficManagementRecommendations` and `SimulationTrafficReplayState.activeTmiTypes` expose the result to API consumers and the Simulation Workbench.

Generated recommendations are advisory and human-reviewed. They do not create, cancel, transmit, or approve operational TMIs.

## Adapter Guidance

Future SWIM/FMDS/TFMS/CDM adapters should map provider-specific fields into these models rather than leaking provider schemas into public DTOs:

- Keep provider receipts and source refs.
- Preserve original provider type code in `TrafficManagementInitiative.type`.
- Set `primitiveType` when the adapter can confidently normalize the code.
- Use `FlowEvaluationAreaModel` for FEA/FCA geometry/filter/time windows even when no control program is active.
- Use `TrafficFlowProgramModel` for GDP/AFP/CTOP-like metering programs with EDCT/acceptance-rate semantics.
- Use specialized detail models for MIT, ground stops, departure metering, arrival rates, sector capacity, and reroute advisories.

Unknown provider codes must remain retained/auditable with diagnostics rather than being silently treated as an operational command.
