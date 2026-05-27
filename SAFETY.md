# Safety Notice

Airspace is a public-interest aviation safety prototype. It is designed for review, research, local demonstration, and collaboration.

It is **not**:

- FAA-certified,
- cockpit-certified,
- an official weather source,
- an official NOTAM source,
- an operational ATC system,
- a dispatch release system,
- a replacement for certified avionics, dispatchers, controllers, meteorologists, or official FAA/NWS/SWIM/KVM systems.

## Intended Use

Airspace is intended to help reviewers evaluate how live-style aviation inputs could be fused into clearer guidance:

- weather products,
- PIREPs,
- NOTAM constraints,
- CARF/ALTRV reservations,
- USNS/NADIN/WMSCR-style traffic,
- route geometry,
- deconfliction and workflow state,
- audit/replay traces.

The project demonstrates a local, replayable path from raw inputs to:

- affected mission detection,
- route-impact summaries,
- reroute/avoidance suggestions,
- coordination drafts,
- pilot handoff briefs,
- source-ref drilldown,
- decision trace and audit/replay.

## Human Review Requirement

All operational guidance must be treated as advisory prototype output.

The system must not be used to:

- issue clearances,
- approve missions,
- dispatch aircraft,
- cancel or modify official reservations,
- transmit official operational messages,
- replace current FAA/NWS/ATC procedures.

Human operators remain responsible for reviewing all source artifacts, current official data, local procedures, and applicable regulations.

## Known Safety Boundaries

- Weather decoders are pragmatic and test-backed, not certified FAA/NWS decoders.
- Route avoidance is deterministic prototype guidance, not a certified NAS-scale routing solver.
- Confidence and capacity-impact models are calibration-ready but not operationally calibrated against authoritative historical outcomes.
- Local file/in-memory adapters are provided; live FAA/NWS/SWIM/KVM integrations require separate authorized implementation.
- Screenshots and demo scenarios are illustrative and not operational evidence.

## Reporting Safety Issues

If you find behavior that could mislead an operator, hide a source artifact, collapse distinct source families, overstate confidence, or blur prototype output with official authority, please open an issue with:

- input data or fixture,
- expected behavior,
- actual behavior,
- screenshots or trace output,
- why the behavior is safety-relevant.

Do not publish private credentials, protected operational data, or sensitive live traffic in public issues.

