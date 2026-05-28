# Aviation Terminology Gap Checklist

Last reviewed: 2026-05-28

Scope: documented-case gap checklist derived from `docs/aviation-terminology-validation.md`.

Status note: as of 2026-05-28, the items below are covered by checked-in
documented/local fixtures, synthetic scale tests, source-family regression
tests, or explicit prototype boundary documentation. This does not claim FAA
certification, live-feed validation, or calibration against real operational
historical outcomes.

## Low Visibility, RVR, LVO, LVP, SMGCS

- [x] Expand RVR fixture coverage with additional operational samples beyond the baseline FAA-style examples already tested, especially variants that combine RVR outage text with remarks, runway groups, multiple RVR components, or unusual status words.
- [x] Add airport-procedure profile data as explicit reference data instead of relying on message text alone for SMGCS/LVO/LVP terminology reconciliation.
- [x] Add UI/API fields that distinguish `reported RVR`, `RVR equipment status`, and `low-visibility procedure terminology` so operators do not have to infer the difference from raw text.
- [x] Add source-specific review messaging for ATIS, tower/airport ops, company minima, and local airport procedures when low-visibility ambiguity is detected.
- [x] Add regression cases proving RVR values in METAR/SPECI and RVR equipment NOTAMs are correlated as separate artifacts, not collapsed into one pseudo-procedure state.

## NOTAM Semantics

- [x] Grow DOM2 reducer coverage for remaining FAA domestic NOTAM keyword families not yet semantically reduced, including less common `AD`, `COM`, `IAP`, `ROUTE`, `SPECIAL`, `SECURITY`, `ODP`, `SID`, `STAR`, `CHART`, and `DATA` variants.
- [x] Add more FAA Order 7930.2U-derived examples for runway lighting, markings, displaced thresholds, approach lighting, braking action, runway condition, and surface contamination.
- [x] Add explicit tests that non-geometric NOTAMs remain table/audit constraints and never become fake map geometry.
- [x] Add explicit tests that ALTRV-related NOTAM publication text remains separate from the underlying CARF/ALTRV reservation artifact.
- [x] Expand ICAO/FDC/Canadian NOTAM field parsing fixtures for cancellation/replacement, estimated/permanent end, missing geometry, malformed Q-fields, and non-U.S. location formats.

## CARF, ALTRV, APREQ, APVL

- [x] Validate ALTRV moving vs stationary terminology against a broader operational sample set, not only legacy grammar examples.
- [x] Add workflow terminology checks that distinguish APREQ, APVL, cancellation, supplement, publication, coordination note, and reservation state in API DTOs and frontend labels.
- [x] Add end-to-end cases where an ALTRV generates or references NOTAM text while preserving both artifacts and their distinct source-family chips.
- [x] Add corpus cases for international or military special-operation phrases that CARF may coordinate but that should not become generic NOTAM constraints.

## Weather Product Terminology

- [x] Deepen METAR/SPECI decoding fixtures for RVR groups, SPECI triggers, variable visibility, vertical visibility, runway visual range trends, remarks retention, and station-to-route association.
- [x] Deepen TAF slicing for amended/corrected reports, multi-day validity, `FM`, `TEMPO`, `BECMG`, `PROB`, `PROB TEMPO`, and confidence by forecast group.
- [x] Add explicit checks that terminal METAR/TAF products without route-usable geometry remain station/time guidance artifacts unless a reference-point-to-route association is intentionally computed.
- [x] Expand PIREP/AIREP fixtures for CONUS NAVAID-relative locations, oceanic lat/lon positions, urgency, duplicate reports, negative/smooth reports, missing altitude, and miscoded turbulence/icing fields.
- [x] Add richer SIGMET, Convective SIGMET, G-AIRMET/AIRMET, CWA, and CWSU-style examples with geometry, validity, altitude band, movement, intensity, tops, and missing-geometry diagnostics.
- [x] Add operational stale-product tests that distinguish CWA tactical lifespan from longer-range TAF/G-AIRMET horizons.

## NWP, CWAF, CWAP, Route Blockage, Capacity

- [x] Keep all public UI/docs saying `CWAP-style` or `CWAF-like` unless an authoritative CWAP/CWAF product feed is actually ingested and identified.
- [x] Add calibrated model hooks backed by documented fixture outcomes before describing route blockage probability as prototype-calibrated.
- [x] Add route-impact tests that expose altitude, echo tops, growth/decay, lead time, source age, product type, confidence, sector demand, and capacity assumptions in every high-severity route blockage explanation.
- [x] Add confidence-metric wording checks so `confidence` is consistently described as decision/constraint confidence, not certified meteorological forecast confidence.
- [x] Add performance and scale tests for high-volume route, sector, and weather constraint sets that approximate NAS-scale candidate filtering.

## Decision Action Vocabulary And Public Copy

- [x] Expand the terminology lint test to cover more frontend pages, API DTO labels, generated screenshots/docs captions, and future Markdown files as they are added.
- [x] Add UI affordances that render `DELAY` with a clarifying sublabel such as `CONFIRM PROCEDURE STATE`, `CONFIRM CAPABILITY`, or `CONFIRM SURFACE CONDITION` depending on source family.
- [x] Add tests proving every `AVOID`, `REROUTE`, and `BLOCKED` display includes exact source refs, confidence, and threshold rationale.
- [x] Add tests proving `DELAY` remains advisory and never mutates official mission/reservation state without an operator workflow action.
- [x] Add a public terminology review checklist to release documentation so reviewers can check for overclaims before publication.

## Source-Family Separation

- [x] Add browser/E2E or equivalent helper assertions that source-family chips route correctly from every major surface: Mission Explorer, Mission, Reservation, Deconfliction, Feed, NOTAM, Weather, Decision, Pilot Brief, Search, and History.
- [x] Add cross-family map tests proving CARF/ALTRV reservations, NOTAM constraints, weather products, PIREPs/AIREPs, route impacts, and conflicts stay visually and semantically distinct.
- [x] Add persisted/replay cases that reload all source-family chips after restart and preserve the same family labels and target links.

## Live And Authoritative Data Readiness

- [x] Add reference-data import/sync validation for airport procedure profiles, NAVAIDs, fixes, aerodromes, RVR component metadata, and effective-time/version provenance.
- [x] Add live adapter conformance tests for future USNS/NADIN/WMSCR/SWIM/KVM/AWC feeds without requiring credentials.
- [x] Add typed diagnostics for “authoritative source unavailable” so the workbench can distinguish local fixture mode, mock mode, stale live mode, and confirmed live mode.
- [x] Add release notes that clearly separate local prototype behavior from certified operational deployment requirements.
