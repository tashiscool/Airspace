# Airspace Safety Whitepaper

## Summary

Airspace is a public-interest aviation safety prototype for one narrow but important operational problem:

> Aviation teams receive weather products, PIREPs, NOTAMs, reservations, route constraints, and ATC coordination signals through many channels, but those inputs are not always fused into clear, explainable operational guidance fast enough.

The project demonstrates a local, replayable engine and browser workbench that turns mixed aviation inputs into:

- affected mission alerts,
- route-impact summaries,
- weather/PIREP/NOTAM/CARF source refs,
- reroute or avoidance suggestions,
- coordination drafts,
- pilot handoff briefs,
- decision traces,
- audit/replay bundles.

This is not a certified operational system. It is a transparent prototype meant for public review, safety research, and collaboration.

## Problem

The safety gap is not simply “pilots need another weather app.” The harder problem is operational fusion:

- Weather products may describe convection, turbulence, icing, low ceilings, visibility, ash, or wind shear.
- PIREPs may provide fresher aircraft observations but can be stale, incomplete, duplicated, miscoded, delayed, or hard to route to the right operator.
- NOTAMs may create operational constraints without being CARF/ALTRV reservations.
- CARF/ALTRV reservations and protected volumes must be parsed, mapped, deconflicted, and kept separate from other constraint families.
- ATC/weather coordination requires source traceability, affected route/mission context, and human-readable rationale.
- Pilot handoff needs concise guidance, not raw message archaeology.

Airspace explores how these inputs can become clear guidance while preserving provenance.

## Prototype Approach

Airspace uses a deterministic, replayable decision loop:

1. **Observe:** ingest USNS/NADIN/WMSCR-shaped messages, CARF/ALTRV reservations, NOTAMs, METAR/TAF/SIGMET/AIRMET/CWAP-style weather, PIREPs, reference points, and route candidates.
2. **Normalize:** convert inputs into typed artifacts with raw text, source refs, diagnostics, time windows, altitude bands, confidence, freshness, and geometry.
3. **Fuse:** create common operational constraints from weather, PIREPs, NOTAMs, CARF/ALTRV reservations, route geometry, and workflow state.
4. **Predict:** evaluate affected missions, route segments, blockage probability, route alternatives, capacity-impact seams, stale-data warnings, and residual risk.
5. **Guide:** return CLEAR, MONITOR, CAUTION, DELAY, ALTITUDE CHANGE, AVOID, REROUTE, or BLOCKED with confidence and rationale.
6. **Coordinate:** draft source-linked coordination messages for weather desk, traffic manager, mission owner, or pilot handoff.
7. **Explain:** expose source refs, trace rules, confidence math, diagnostics, and map overlays.
8. **Audit:** retain request/result hashes, replay bundles, rule-catalog versions, source refs, and operator actions.

## What The Prototype Demonstrates

- A CARF/ALTRV parser and reservation mapper with route/event/area metadata.
- USNS-style message classification for CARF, NOTAM, service, weather, and PIREP traffic.
- NOTAM parsing and retention as constraints distinct from ALTRV reservations.
- Weather product modeling for turbulence, icing, convection, ceiling, visibility, SIGMET/AIRMET, METAR/TAF, CWAP-style/CWAF-like products, PIREPs, confidence, validity, and movement.
- Route-impact and route-avoidance summaries with affected segments, source refs, confidence, and deterministic cost/route comparison seams.
- Mission-scoped PIREP relevance by route, altitude, recency, and staleness.
- ATC/weather coordination drafts generated from exact source artifacts.
- Pilot brief generation from the same fused decision state.
- Browser workbench surfaces for mission explorer, weather/PIREP review, deconfliction, messaging, feed inspection, decision trace, map overlays, config, search, and audit.
- Deterministic audit/replay for local evaluation.

## What The Prototype Does Not Claim

- No FAA certification.
- No cockpit certification.
- No official weather or NOTAM authority.
- No live FAA/NWS/SWIM/KVM credentials.
- No operational dispatch, clearance, or ATC authority.
- No calibrated NAS-scale route or sector-capacity model.
- No replacement for existing FAA/NWS/ATC procedures.

## Evaluation Questions

Reviewers can help by answering:

1. Does the workflow show the right source artifacts for each recommendation?
2. Are NOTAMs, weather products, PIREPs, CARF/ALTRV reservations, and route impacts kept distinct enough?
3. Are the affected-mission and pilot-brief views useful for fast operational review?
4. Are diagnostics strong enough when a parser cannot confidently map text to geometry?
5. Are the route-impact and reroute summaries understandable to controllers, dispatchers, and pilots?
6. What additional scenario fixtures would make the prototype more realistic?
7. Which confidence/scoring assumptions would need calibration before operational use?

## Public-Interest Release Model

Airspace is released as donation-only public-interest software under AGPL-3.0-or-later. The project governance rejects quiet proprietary capture of core safety logic. Adapter integrations can be added, but the engine, tests, scenarios, traces, and safety-relevant rules should remain publicly inspectable.

