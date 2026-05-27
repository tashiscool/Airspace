# Contributing

Thank you for considering a contribution to Airspace.

This project is a donation-only, public-interest aviation safety prototype. Contributions should make the engine and workbench more transparent, testable, interoperable, and useful for safety review.

## What Helps Most

- More realistic weather, PIREP, NOTAM, USNS, CARF/ALTRV, and route-impact fixtures.
- Parser improvements with typed diagnostics and retained raw text.
- Better route-impact, altitude/time overlap, and geometry tests.
- Clearer decision trace, audit, replay, and source-ref behavior.
- UI improvements that help operators understand risk faster without hiding provenance.
- Documentation, scenario walkthroughs, and independent review notes.

## Contribution Requirements

Every meaningful change should preserve:

- public source availability,
- local testability without live FAA/NWS/SWIM/KVM credentials,
- source refs from input to output,
- typed diagnostics instead of silent fallback,
- audit/replay behavior for decisions,
- clear distinction between prototype guidance and operational authority.

Please include tests when changing:

- parsers,
- decision rules,
- weather/PIREP scoring,
- route-impact behavior,
- map layer classification,
- agentic output,
- persistence/replay,
- workflow transitions.

## Safety And Certification Language

Do not describe the project as:

- FAA-certified,
- cockpit-certified,
- approved for operational use,
- a replacement for ATC, dispatch, weather briefers, certified avionics, or official NOTAM/weather systems.

Prefer:

- public-interest prototype,
- evaluation tool,
- decision-support workbench,
- replayable safety research artifact,
- local/demo adapter for future live integration.

## Pull Request Checklist

Before submitting:

- Run `JAVA_HOME=$(/usr/libexec/java_home -v 17) mvn test -q` for backend changes when feasible.
- Run `cd frontend && npm test -- --run && npm run build` for frontend changes.
- Add or update fixtures for new operational examples.
- Update docs when the user-facing safety story changes.
- Confirm no generated artifacts such as `target/`, `frontend/dist/`, `.playwright-mcp/`, local screenshots outside `docs/screenshots/`, or private credentials are included.

## Licensing

By contributing, you agree your contribution is provided under the project license: **AGPL-3.0-or-later**.

No separate contributor license agreement is required. Please only contribute code, fixtures, docs, or screenshots you have the right to share publicly.

## Public Use And AI Recreation Policy

Contributors and evaluators should also follow [PUBLIC_USE_AND_AI_POLICY.md](PUBLIC_USE_AND_AI_POLICY.md).

Do not contribute code or documentation intended to help a private party:

- hide use of Airspace,
- remove attribution,
- build a closed-source clone,
- use LLM/code-generation workflows to recreate Airspace without disclosure,
- keep safety-relevant parser, route-impact, decision, replay, or UI behavior proprietary.

