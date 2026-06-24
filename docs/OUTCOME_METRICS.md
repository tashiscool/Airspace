# Outcome Metrics

Airspace now exposes outcome metrics for local simulation, replay, and TFM-review runs through:

- `GET /api/outcomes/metrics`
- `POST /api/outcomes/metrics`
- `/outcomes` in the frontend workbench

These metrics are designed to answer the operator and evaluator question: **did the decision-support loop reduce delay, fuel burn, overload risk, unsafe guidance, missing evidence, or operator latency in this scenario?**

## Public Reference Shape

The metric categories follow public FAA traffic-flow and performance concepts, without claiming live NAS authority:

- FAA ASPM and ATCSCC reporting expose operations, delays, airport efficiency, city-pair time/distance, weather delay factors, advisories, and TMI context.
- FAA NextGen benefit reporting discusses delay savings, fuel savings, route efficiency, and normalization for weather, runway configuration, and demand.
- FAA TFM learning material frames traffic-management actions as least-restrictive demand/capacity tools: reroutes, miles-in-trail, minutes-in-trail, metering, GDPs, AFPs, and ground stops.
- FAA weather/TFM confidence material emphasizes constraint prediction, confidence metrics, shared situational awareness, and forecast uncertainty.

References:

- <https://www.aspm.faa.gov/aspmhelp/index/ASPM_Efficiency__ATCSCC_Metrics_Report.html>
- <https://www.aspm.faa.gov/aspmhelp/index/Aviation_System_Performance_Metrics_%28ASPM%29.html>
- <https://www.aspm.faa.gov/aspmhelp/index/ASPM_AERO__Definitions_of_Variables.html>
- <https://www.faa.gov/nextgen/reporting-benefits>
- <https://tfmlearning.faa.gov/tfm-training/atfm-basics/cdm-t1-lesson3b.html>
- <https://www.faa.gov/nextgen/programs/weather/tfm_support/confidence>

## Metrics Implemented

| Metric | Meaning | Current implementation |
|---|---|---|
| Delay minutes saved | Local baseline delay minus mitigated delay after reroute/TMI review | Simulation action penalties, airport surface delay, sector handoff delay, plus modeled TMI delay-savings factors |
| Fuel impact | Added reroute fuel minus avoided holding/delay fuel | Uses route candidate fuel deltas plus a local holding-fuel proxy per delay minute saved |
| Reroute miles | Added nautical miles from selected local alternate route candidates | Uses route candidate comparison cost estimates |
| Sector overload avoided | Overloaded airport/sector resources with matching TMI or reroute mitigation | Counts simulation workload overloads and TFM-board overloads with relevant proposed TMIs |
| False clear | Scenario-labeled case where final `CLEAR` contradicts expected guidance | Requires expected outcome labels in local scenarios |
| False block | Scenario-labeled case where final `BLOCKED` contradicts expected guidance | Requires expected outcome labels in local scenarios |
| Source-ref completeness | Share of evaluated steps or board rows retaining evidence references | Counts source refs on simulation steps, route impacts, and board summaries |
| Operator time-to-decision | Time from scenario start to actionable guidance plus communication/review latency | Uses first actionable guidance tick plus modeled communication and review time |

## Non-Claims

These are **not** calibrated FAA post-event performance measures. They are local, fixture-backed outcome indicators for product review, safety-case preparation, regression testing, and demo campaigns.

Operational/evaluation use still needs:

- authorized SWIM/TFMS/FNS/NMS/ASPM or equivalent data receipts,
- historical calibration datasets,
- independent aviation expert review,
- documented false-clear/false-block labels,
- human-approved coordination and workflow decisions.

## API Shape

`OutcomeMetricsRequest` can select:

- an existing simulation `runId`,
- an existing `campaignId`,
- a `scenarioId` with `runSimulation=true`,
- a local `demandCapacityConfig` for TFM-board evidence,
- `includeTfmBoard=false` for pure run/campaign-only reporting.

`OutcomeMetricsReport` returns:

- top-level totals for delay, fuel, reroute, overload, safety, source-ref, and operator latency,
- normalized metric cards with status and rationale,
- source references,
- assumptions,
- diagnostics.

The frontend `/outcomes` page exposes the same report with editable local scenario and demand/capacity controls.
