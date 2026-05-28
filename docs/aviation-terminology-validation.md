# Aviation Terminology Validation Notes

Last reviewed: 2026-05-28

Purpose: collect public-source terminology checks for Airspace so we can later validate implementation behavior without overcorrecting. This is not an operational procedure manual and not a certification claim.

## Source Priority

Use this order when implementation wording or parser behavior conflicts:

1. FAA orders, AIM, Pilot/Controller Glossary, and FAA program pages.
2. NOAA/NWS/Aviation Weather Center product documentation.
3. ICAO terminology only where FAA publications explicitly reference it or where the product is international/non-U.S.
4. Legacy CARF/USNS source workspaces only as behavior oracles, not as modern authority.
5. Community commentary only as scenario discovery, never as source of truth.

## Public Sources Checked

- FAA Pilot/Controller Glossary, `Visibility` and `Runway Visual Range (RVR)`: https://www.faa.gov/air_traffic/publications/atpubs/pcg_html/glossary-v.html
- FAA JO 7110.65, runway visibility reporting and RVR/RVV phraseology: https://www.faa.gov/air_traffic/publications/atpubs/atc/atc0208.html
- FAA LVO/SMGCS Program page: https://www.faa.gov/about/office_org/headquarters_offices/avs/offices/afx/afs/afs400/afs420/lvo_smgcs
- FAA Order 7930.2U, NOTAM: https://www.faa.gov/documentLibrary/media/Order/7930.2U_Bsc_dtd_6_9_25_FINAL.pdf
- FAA NOTAM scope and keywords: https://www.faa.gov/air_traffic/publications/atpubs/notam_html/chap1_section_2.html
- FAA domestic NOTAM contractions: https://www.faa.gov/air_traffic/publications/domesticnotices/contractions.html
- FAA Order 7610.14 ALTRV procedures: https://www.faa.gov/air_traffic/publications/atpubs/so_html/chap4_section_1.html
- DOT CARF privacy impact summary: https://www.transportation.gov/resources/individuals/privacy/central-altitude-reservation-function-carf
- NOAA/NWS Aviation Weather Center data help: https://aviationweather.gov/help/data/
- FAA Flight Services Center Weather Advisory description: https://www.faa.gov/air_traffic/publications/atpubs/fs_html/chap8_section_8.html
- FAA NextGen Weather Processor products: https://www.faa.gov/nextgen/programs/weather/nwp/nwpproducts
- FAA NWP confidence metrics: https://www.faa.gov/nextgen/programs/weather/tfm_support/confidence
- FAA NWP translation products: https://www.faa.gov/nextgen/programs/weather/tfm_support/translation_products

## Low Visibility, RVR, LVO, LVP, SMGCS

### Confirmed Terminology

| Term | Public-source meaning | Airspace implementation implication |
|---|---|---|
| RVR | Runway Visual Range is an instrument-derived horizontal range a pilot can expect to see down the runway, reported in hundreds of feet in U.S. usage when available. | Safe to parse and display as runway visibility context. Do not treat it as a direct go/no-go decision by itself. |
| Touchdown / Mid / Rollout RVR | FAA glossary distinguishes touchdown, mid, and rollout RVR readouts. | `RVRT`, `RVRM`, and `RVRR` should remain separate semantic conditions where possible. |
| RVR phraseology | JO 7110.65 gives examples such as runway, RVR, value; multiple systems may be reported as touchdown/mid/rollout. | UI copy should say “reported/issued RVR,” not “declared visibility procedure.” |
| LVO/SMGCS | FAA maintains an LVO/SMGCS program and points users to accepted airport lists, Order 8000.94, and AC 120-57. FAA page explicitly says users should confirm latest airport information, such as ATIS. | Correct posture is “confirm procedure state with authorized/local sources.” Airspace should not declare that an airport is or is not in LVO/SMGCS. |
| LVP | Common ICAO/operator phrase for low-visibility procedures, but not always the phrase used in FAA/local tower exchange. | Treat as terminology that may need reconciliation with local FAA/airport language. Avoid claiming it is always the controlling U.S. term. |

### Implementation Review Checks

- Keep the current advisory wording: `DELAY / CONFIRM PROCEDURE STATE` is a workbench guidance posture, not an official operational declaration.
- Do not hardcode a global “RVR below X means cannot depart” rule. Public FAA material shows multiple procedure contexts, equipment states, airport-specific programs, and operator minima.
- Do not assume `LVO`, `LVP`, and `SMGCS` are interchangeable. They can point to related low-visibility concerns, but the active procedure name and threshold must be confirmed from local/authorized sources.
- Review DOM2 RVR examples: FAA Order 7930.2U shows single-runway RVR NOTAM examples as `RWY 10 RVRT U/S` and all-RVR outage as `AD AP RVR ALL U/S`. Our parser should accept legacy `SVC` variants as retained traffic if encountered, but should not prefer `SVC` as the canonical single-runway RVR keyword without evidence.

## NOTAM, Domestic, ICAO, FDC, And “Not All NOTAMs Are ALTRVs”

### Confirmed Terminology

| Term | Public-source meaning | Airspace implementation implication |
|---|---|---|
| NOTAM | FAA Order 7930.2U prescribes formatting and distribution for temporary/unanticipated NAS changes; the 2025 order uses “Notice to Airmen.” | Documentation can use NOTAM without expanding every time; if expanded, use current FAA order terminology. |
| NOTAM keywords | FAA lists keywords including `RWY`, `TWY`, `APRON`, `AD`, `OBST`, `NAV`, `COM`, `SVC`, `AIRSPACE`, `IAP`, `ROUTE`, `SPECIAL`, `SECURITY`, etc. | Parser classification should keep keyword family and semantic reducer separate. |
| RWY | FAA keyword for temporary changes/hazards associated with landing/takeoff surfaces, lighting, markings, signage, and runway-specific services or attributes. | RVR, runway lighting, runway closure, braking/friction, and markings often belong under runway-specific semantics. |
| AD | Aerodrome-level keyword for airport/heliport/maneuvering-area hazards not tied to a specific movement surface. | All-RVR outage and airport-level procedure/visibility context can be AD-level constraints. |
| NAV/IAP/FDC | Instrument aids and procedures can affect approach/minima capability. | Treat as capability review constraints, not automatic blocked-route decisions. |
| ALTRV NOTAM | FAA Order 7610.14 includes altitude reservation NOTAM procedures, but ALTRV NOTAMs are a subset of NOTAMs. | Keep NOTAM constraints distinct from CARF/ALTRV reservations unless the source explicitly represents an altitude reservation. |

### Implementation Review Checks

- Keep “NOTAM constraint” and “CARF/ALTRV reservation” as separate source families in the frontend, API, map layers, and decision trace.
- Parser should not turn non-geometric NOTAMs into fake route polygons. Retain raw text and diagnostics when geometry is not present.
- RVR/RVRT/RVRM/RVRR, approach lighting, ILS/CAT/minima, braking action, friction/MU, snow/ice/slush should be operational review drivers, not automatic legal/unsafe declarations.
- Consider adding canonical examples from FAA Order 7930.2U to parser fixtures:
  - `RWY 10 RVRT U/S`
  - `RWY 10L RVRM U/S`
  - `RWY 10R RVRR U/S`
  - `AD AP RVR ALL U/S`

## CARF, ALTRV, APREQ, APVL

### Confirmed Terminology

| Term | Public-source meaning | Airspace implementation implication |
|---|---|---|
| CARF | DOT describes FAA ATO CARF as coordinating ALTRV requests and special military operations nationally and internationally. | `CARF` is a coordination function/source family, not a generic synonym for all airspace constraints. |
| ALTRV | FAA Order 7610.14 says an altitude reservation is authorization for airspace utilization under prescribed conditions and must receive special handling. | Reservations should retain time, altitude, route/area, moving/stationary, approval/request state, and source refs. |
| Moving / stationary ALTRV | FAA ALTRV policy classifies ALTRVs as moving or stationary. | Preserve moving vs stationary geometry intent. |
| APREQ / APVL | FAA Order 7610.14 covers ALTRV Approval Request and Approval. | Workflow labels should distinguish request, approval, cancellation, supplement, and NOTAM publication. |

### Implementation Review Checks

- Do not treat every NOTAM as an ALTRV.
- Do not treat every ALTRV as a NOTAM. ALTRV coordination may result in NOTAMs, but the reservation workflow and the published notice are separate artifacts.
- Keep “CARF/ALTRV” as one input family in operator UI, with NOTAM, weather, PIREP, and route-impact families separate.

## Weather Product Terms

### Confirmed Terminology

| Term | Public-source meaning | Airspace implementation implication |
|---|---|---|
| METAR/SPECI | AWC documentation describes SPECI as an unscheduled report triggered by criteria including visibility, RVR, ceiling, sky condition, tornado, volcano, or mishap. METAR/SPECI contain station, time, wind, visibility, RVR, weather, sky, temperature/dewpoint, etc. | METAR/SPECI parser should keep station/time/weather/visibility/RVR structured. Without coordinates beyond station reference, it should not invent polygons. |
| TAF | AWC says TAFs are concise expected aviation weather conditions for a specified period within 5 SM of the airport runway complex; U.S. TAFs are usually 24 or 30 hours and amended TAFs supersede prior TAFs. | Forecast slicing is appropriate. Scope is terminal/station-centered, not route-wide unless associated with route/reservation context. |
| PIREP | AWC describes PIREP as pilot-reported hazardous weather such as icing or turbulence; PIREPs over CONUS commonly use location relative to NAVAID/airport, while AIREPs often use lat/lon and are common outside CONUS. | PIREP relevance must consider location confidence, altitude, recency, duplicate/stale status, and route corridor. |
| SIGMET | AWC says U.S. SIGMETs advise weather other than convection that is potentially hazardous to all aircraft: severe icing, severe/extreme turbulence, dust/sand storms below 3 SM visibility, volcanic ash. | Severe/extreme SIGMETs can be high-priority hazards, but route impact still needs geometry/time/altitude overlap. |
| Convective SIGMET | AWC lists criteria for thunderstorm-related hazards and says any convective SIGMET implies severe or greater turbulence, severe icing, and low-level wind shear. | Convective SIGMETs are valid strong route-impact drivers, especially with geometry/forecast movement. |
| G-AIRMET/AIRMET | AWC says G-AIRMETs are graphical advisories for weather hazardous to aircraft but less severe than SIGMETs, valid at time snapshots, issued up to 12 hours. | AIRMET/G-AIRMET should usually produce monitor/caution/altitude-change guidance unless overlap/severity/PIREP corroboration raises risk. |
| CWA | FAA Flight Services and AWC describe Center Weather Advisory as an unscheduled advisory for conditions meeting or approaching national in-flight advisory criteria, primarily for aircrew avoidance in en route/terminal environments; valid up to 2 hours. AWC notes CWA is not a flight-planning product because of short lead/duration. | Treat CWA as high-freshness tactical context, not long-range flight-planning basis. Include staleness handling. |

### Implementation Review Checks

- Keep METAR/TAF station products without route geometry as table/guidance artifacts unless a station-to-route association is explicitly computed.
- TAF forecast groups should produce time slices with confidence and source spans.
- PIREP route relevance should default to corridor + altitude tolerance + recency, and clearly show when a report is off-route/off-altitude/stale.
- Convective SIGMET/CWA/CWAP-like products should not automatically block unless route/time/altitude geometry intersects and scoring thresholds are met.

## NWP, CWAF, CWAP, Route Blockage, Capacity

### Confirmed Terminology

| Term | Public-source meaning | Airspace implementation implication |
|---|---|---|
| NWP | FAA says NWP identifies aviation safety hazards and translates weather information needed to predict route blockage and airspace capacity constraints up to 8 hours in advance. | Our engine language about 0-8 hour route impact is aligned as a prototype claim, but not calibrated operational equivalence. |
| CWAF | FAA says Convective Weather Avoidance Fields provide a quantitative probability assessment of pilot deviation likelihood and are altitude-dependent. | `deviationLikelihood` and altitude-dependent scoring are appropriate terms. Mark model as prototype unless calibrated. |
| CWAP | FAA says Convective Weather Avoidance Polygons provide fly/no-fly display zones and predictions of flow-constrained areas pilots will avoid. | Use `CWAP-style` unless ingesting authoritative CWAP. Keep polygon intent and source refs. |
| Confidence metrics | FAA NWP confidence metrics convey uncertainty in predicted airspace constraints, not simply weather forecast uncertainty; they support strategic traffic-flow decisions over an 8-hour horizon. | Our confidence fields should be described as constraint/decision confidence, not certified meteorological confidence. |
| Capacity impact | FAA NWP pages tie predicted weather constraints to route blockage, flow-constrained areas, airspace permeability, and capacity impact. | `capacityImpact` is a valid prototype field, but real capacity model needs sector demand/calibration before operational use. |

### Implementation Review Checks

- UI and README should say “CWAP-style” or “CWAF-like” unless an authoritative source product is actually ingested.
- Route impact scoring should expose altitude, echo tops, growth/decay, lead time, source age, product type, confidence, and capacity assumptions.
- Avoid claiming historical calibration unless backed by a documented calibration dataset.

## Decision Action Vocabulary

Airspace currently uses: `CLEAR`, `MONITOR`, `CAUTION`, `DELAY`, `ALTITUDE CHANGE`, `AVOID`, `REROUTE`, `BLOCKED`.

### Recommended Semantics

| Action | Recommended meaning |
|---|---|
| CLEAR | No current fused constraint above review threshold. Continue monitoring. |
| MONITOR | Source exists or hazard may become relevant, but no current route/reservation constraint is above caution threshold. |
| CAUTION | Constraint may affect mission/reservation/route; operator review and briefing advised. |
| DELAY | Advisory posture for release/timing hold or coordination until procedure state, runway condition, capability, or weather confidence is clarified. Not an official ATC/airport declaration. |
| ALTITUDE CHANGE | Route lateral path may remain usable if altitude exposure is changed and authorized. |
| AVOID | Hazard/constraint should be avoided laterally/temporally/vertically if possible. |
| REROUTE | A viable alternate candidate exists or should be coordinated. |
| BLOCKED | Prototype prediction that current route/candidate is unusable under current fused constraints. Must include source refs and confidence. |

### Implementation Review Checks

- Public UI should avoid “unsafe,” “illegal,” “cannot depart,” or “airport not compliant” unless quoting an authoritative source.
- `DELAY` should often render with explanatory text like `CONFIRM PROCEDURE STATE` or `CONFIRM CAPABILITY`, especially for low-visibility/RVR, approach/minima, and runway-surface cases.
- Every `AVOID`, `REROUTE`, or `BLOCKED` display must show source refs and confidence/threshold rationale.

## Known Implementation Follow-Ups

### Implemented In The Terminology Alignment Pass

- FAA Order 7930.2U-style RVR fixture examples were added to domestic NOTAM regression coverage.
- Canonical runway and aerodrome RVR reducers were added:
  - `DOM2.RWY.RVR` for runway-level `RVRT`, `RVRM`, `RVRR`, and `RVR` records.
  - `DOM2.AD.RVR_ALL` for aerodrome-level all-RVR records.
- Legacy `SVC` RVR text remains accepted as compatibility/retained traffic when appropriate, but runway-bearing RVR text now reduces to the runway family.
- Frontend low-visibility notices recognize both canonical and compatibility reducer IDs.
- A terminology lint test now guards public operator-facing text against authoritative overclaim phrases such as `cannot depart`, `unsafe to depart`, `illegal`, `Airspace declares`, `airport is in LVO`, and `LVO declared`.

### Still Worth Reviewing With Operational Samples

1. Keep FAA Order 7930.2U RVR examples in parser fixtures and expand when operational samples show additional variants:
   - `RWY 10 RVRT U/S`
   - `RWY 10L RVRM U/S`
   - `RWY 10R RVRR U/S`
   - `AD AP RVR ALL U/S`
2. Confirm frontend labels continue to say “RVR equipment/service NOTAM” or “RVR operational constraint,” not “LVO declared.”
3. Keep source-family chips visually and semantically distinct:
   - CARF/ALTRV reservation
   - NOTAM constraint
   - Weather product
   - PIREP/AIREP observation
   - USNS/feed envelope
   - Decision/replay/audit

## Bottom Line

The current advisory posture is directionally right: Airspace should make low-visibility, RVR, NOTAM, weather, PIREP, route, and CARF/ALTRV conflicts visible quickly, cite source artifacts, and prompt coordination. It should not claim to replace FAA procedures, local airport procedure state, company minima, ATC instructions, pilot authority, or certified weather/routing systems.
