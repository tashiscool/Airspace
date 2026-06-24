# Public Use And AI Recreation Policy

Airspace is a donation-only public-interest aviation safety project. The project is published so aviation safety researchers, operators, pilots, controllers, dispatchers, meteorologists, and public-interest technologists can inspect, evaluate, reproduce, and improve it in the open.

The project is **not** published so a private party can quietly use, clone, train on, or recreate the work behind closed doors.

## Plain-English Requirement

If you use Airspace, deploy Airspace, modify Airspace, evaluate Airspace inside an organization, use Airspace as a reference implementation, or use Airspace/code/docs/screenshots/test fixtures/prompts/scenarios as input to an LLM or other automated system to recreate a similar implementation, you are expected to:

1. **Publicly disclose that use.**
2. **Link back to the Airspace repository.**
3. **Publish your modifications or implementation source code.**
4. **Preserve the same public-access expectation for downstream users.**
5. **Keep safety-relevant logic, decision rules, traces, fixtures, and evaluation results inspectable.**

## What Counts As Use

This policy applies to:

- running Airspace as a service,
- deploying Airspace internally,
- adapting Airspace for another aviation workflow,
- copying or translating code,
- using the repository as a reference architecture,
- using the docs, screenshots, tests, fixtures, or code in prompts,
- asking an LLM or code-generation system to recreate, port, summarize into implementation steps, or build a substantially similar tool,
- building a private implementation based on Airspace behavior, workflows, schemas, screenshots, test fixtures, or documentation.

## Required Public Disclosure

Public disclosure should include:

- organization or individual using it,
- public contact or project page,
- purpose of use,
- whether it is evaluation, deployment, derivative work, LLM-assisted recreation, or integration,
- link to the Airspace repository,
- link to your published source code or modifications,
- link to your published non-secret operational configuration, including redacted deployment, adapter, source-mode, rule, threshold, calibration, audit/replay, and agent-policy settings needed for public safety review.

Recommended disclosure text:

> This project uses or is derived from Airspace, a donation-only public-interest aviation safety prototype by LeosSoftwareLLC: https://github.com/tashiscool/Airspace

## Required Source Publication

If you modify, extend, port, translate, or recreate Airspace behavior, publish the corresponding source code needed to inspect, build, test, and run your implementation.

That includes:

- parser changes,
- route-impact logic,
- weather/PIREP/NOTAM handling,
- decision rules,
- UI/workbench flows,
- scenarios/fixtures,
- replay/audit logic,
- agentic prompts or policy logic,
- LLM-generated implementation derived from this project.

## Operational Configuration Transparency

Safety-relevant behavior often lives in configuration, not just source code. If you deploy, integrate, or recreate Airspace, publish the non-secret operational configuration needed to inspect how the implementation behaves.

Examples include:

- enabled adapters and provider modes,
- source-freshness policy,
- rule catalog version,
- decision thresholds,
- calibration model version,
- route-impact scoring settings,
- audit/replay settings,
- agent policy version,
- redacted environment variable names and templates.

Do **not** publish secrets, credentials, tokens, API keys, private certificates, passwords, personal data, private operational data, or sensitive aviation-security details. Use redacted templates.

## Compliance Disclosure Tools

Airspace does not use hidden telemetry or clone-time collection. Compliance is based on public, explicit disclosure:

- `COMPLIANCE.md`,
- `docs/COMPLIANCE_COLLECTOR.md`,
- `.github/ISSUE_TEMPLATE/airspace-use-disclosure.yml`,
- `scripts/airspace-compliance-manifest.sh`,
- `/api/compliance/policy`,
- `/api/compliance/manifest`,
- `/api/compliance/attestations`.

The local manifest script prints a disclosure manifest by default and submits only when explicitly configured with `AIRSPACE_COMPLIANCE_SUBMIT=1` and `AIRSPACE_COMPLIANCE_COLLECTOR_URL`.

## LLM And Model-Generated Code

Using this repository as LLM context does not erase provenance.

If an LLM, agent, code-generation system, fine-tuning workflow, retrieval system, or prompt pack uses Airspace to produce a new implementation, that implementation should be treated as derived from Airspace for public-interest purposes and should be published under terms that keep the implementation open and inspectable.

Do not use LLM generation as a laundering step to create a closed-source clone.

## Safety Rationale

Airspace deals with safety-relevant aviation decision support. Closed clones are dangerous because reviewers cannot inspect:

- parser behavior,
- source-ref handling,
- stale-data logic,
- route-impact assumptions,
- confidence math,
- NOTAM/weather/PIREP separation,
- decision trace and audit behavior,
- agentic prompt or policy constraints,
- failure modes.

Public source availability is part of the safety model.

## Legal Note

The repository license is currently AGPL-3.0-or-later. AGPL provides strong copyleft obligations for modified versions and network-service interaction, but some requirements in this policy, especially broad public-use disclosure and LLM-assisted independent recreation, may require custom license language for full legal enforceability.

This document states the project’s public-interest policy and contribution condition. If strict enforceability is required, this policy should be reviewed and converted into a lawyer-drafted custom public-benefit source license.
