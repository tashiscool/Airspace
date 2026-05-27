# Governance

Airspace is a public-interest aviation safety project operated by **LeosSoftwareLLC** as a donation-only effort.

The project exists to improve transparent, auditable operational decision support around weather, PIREPs, NOTAMs, CARF/ALTRV reservations, USNS-style messages, route impact, coordination, and pilot/controller handoff. It is not intended to become proprietary contractor-controlled software.

## Stewardship Principles

- **Public safety first:** design choices should improve traceability, reviewability, and operational clarity.
- **Open by default:** code, scenarios, test fixtures, issue discussion, and safety rationale should remain public unless disclosure would create a security or safety risk.
- **No black-box authority:** generated guidance, agent output, reroute suggestions, and weather decisions must remain explainable and cite source artifacts.
- **Human operational control:** the system must not claim authority to issue clearances, mutate official records, or send operational messages without a human operator.
- **No quiet capture:** contributions that would move core safety logic behind proprietary services, undisclosed models, closed feeds, or unavailable runtime dependencies are not acceptable for the main project.
- **No undisclosed use or AI laundering:** organizations using, recreating, or LLM-generating implementations from Airspace are expected to disclose that use and publish their implementation source.
- **Evidence over claims:** safety claims should be backed by tests, replayable scenarios, screenshots, traces, or documented evaluation results.

## Decision Process

For now, Tashdid Khan / LeosSoftwareLLC is the project steward and final maintainer.

Changes are accepted when they:

- preserve public availability under the project license,
- improve safety, explainability, interoperability, or test coverage,
- keep live integrations optional and adapter-based,
- avoid hard-coding private credentials, proprietary feed assumptions, or unavailable infrastructure,
- include enough tests or scenario evidence for the risk level of the change.

Major changes should be proposed first as a GitHub issue or design note when they affect:

- operational decision rules,
- weather/PIREP/NOTAM parsing semantics,
- route-impact scoring,
- agentic guidance behavior,
- audit/replay guarantees,
- licensing, governance, or public access.

## Anti-Capture Contribution Rules

Contributions are welcome only under terms that preserve open public access.

Contributors must not submit code that:

- requires a proprietary hosted service for core engine behavior,
- hides safety-relevant logic behind an opaque API,
- removes source-ref, trace, audit, replay, or diagnostic behavior,
- weakens the distinction between prototype guidance and official operational authority,
- makes FAA/NWS/SWIM/KVM/live-feed credentials required for local tests,
- turns the project into a vendor-specific implementation.

Adapter code for proprietary or credentialed systems can be discussed, but it must remain optional and separated from the core engine.

## Public Use And AI Recreation

The project’s public-use and LLM/AI recreation expectations are documented in [PUBLIC_USE_AND_AI_POLICY.md](PUBLIC_USE_AND_AI_POLICY.md).

In short: if someone uses Airspace, evaluates it inside an organization, deploys it, modifies it, uses it as a reference implementation, or uses an LLM/agent/code-generation workflow against the code, docs, screenshots, fixtures, schemas, or prompts to recreate similar software, they are expected to publicly disclose that use and publish the resulting implementation source code.

This is a safety and anti-capture rule. LLM generation must not be used as a laundering step for a closed-source clone.

## Relationship To Agencies, Researchers, And Operators

Airspace is offered for public review, evaluation, and collaboration. Feedback from FAA, NTSB, NASA, academic researchers, pilots, controllers, dispatchers, meteorologists, and safety organizations is welcome.

The preferred collaboration model is:

1. reproduce the local demo,
2. review the safety whitepaper,
3. run the scenario corpus,
4. file issues with concrete evidence,
5. contribute improvements that remain public.

The project is not a certified FAA system, not a cockpit-certified tool, and not a live operational feed integration.
