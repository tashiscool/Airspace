# Airspace Compliance And Public Disclosure

Airspace is licensed under `AGPL-3.0-or-later` and carries an additional public-use and AI-recreation policy for safety transparency. The project expects organizations and individuals who use, deploy, modify, port, integrate, or recreate Airspace to publicly disclose that use and publish the source and non-secret operational configuration needed for review.

This is not legal advice. The AGPL is the legal license; some broader transparency expectations, especially independent LLM-assisted recreation and operational configuration publication, may require lawyer-drafted custom terms for strict enforceability.

## Why There Is No Clone-Time Collector

Airspace does **not** include a hidden clone-time collector.

Reasons:

- `git clone` should not execute network code.
- Hidden telemetry would be inappropriate for a public-interest safety project.
- License compliance evidence should be public, inspectable, and consent-based.
- No one should accidentally submit secrets, credentials, private operational data, personal data, or sensitive aviation-security details.

Instead, Airspace provides transparent collection paths:

- public disclosure issue template: `.github/ISSUE_TEMPLATE/airspace-use-disclosure.yml`,
- local manifest script: `scripts/airspace-compliance-manifest.sh`,
- product endpoints:
  - `GET /api/compliance/policy`,
  - `GET /api/compliance/manifest`,
  - `POST /api/compliance/attestations`,
  - `GET /api/compliance/attestations`.

## What Users Should Publish

Publish the corresponding source and non-secret operational configuration for any Airspace use, deployment, modification, integration, port, reference implementation, or LLM-assisted recreation.

Source publication should include:

- parser changes,
- weather/PIREP/NOTAM/CARF/ALTRV handling,
- route-impact and route-avoidance logic,
- decision rules and thresholds,
- replay/audit logic,
- scenario fixtures and evaluation corpus,
- agent prompts, policies, tool definitions, and safety gates,
- UI/workbench flows derived from Airspace.

Operational configuration publication should include non-secret values needed for safety review:

- deployment topology,
- enabled adapters and provider modes,
- rule catalog version,
- calibration model version,
- decision thresholds,
- agent policy version,
- audit/replay settings,
- source-freshness policy,
- redacted environment variable names without values.

Never publish:

- credentials,
- API keys,
- access tokens,
- private certificates,
- passwords,
- live operational data,
- personal data,
- sensitive aviation-security details.

## Generate A Local Manifest

Print a local disclosure manifest:

```bash
./scripts/airspace-compliance-manifest.sh
```

Fill fields through environment variables:

```bash
AIRSPACE_COMPLIANCE_ORGANIZATION="Example Ops Lab" \
AIRSPACE_COMPLIANCE_PURPOSE="Internal evaluation" \
AIRSPACE_COMPLIANCE_USE_TYPE="EVALUATION" \
AIRSPACE_COMPLIANCE_SOURCE_CODE_URL="https://example.invalid/airspace-fork" \
AIRSPACE_COMPLIANCE_OPERATIONAL_CONFIG_URL="https://example.invalid/airspace-config" \
AIRSPACE_COMPLIANCE_PUBLIC_DISCLOSURE_URL="https://example.invalid/disclosure" \
AIRSPACE_COMPLIANCE_ACK_AGPL=true \
AIRSPACE_COMPLIANCE_ACK_PUBLIC_USE_POLICY=true \
AIRSPACE_COMPLIANCE_SOURCE_PUBLISHED=true \
AIRSPACE_COMPLIANCE_OPERATIONAL_CONFIG_PUBLISHED=true \
AIRSPACE_COMPLIANCE_SECRETS_REDACTED=true \
./scripts/airspace-compliance-manifest.sh
```

The script does not submit anything by default.

Explicit submission requires both:

```bash
AIRSPACE_COMPLIANCE_SUBMIT=1
AIRSPACE_COMPLIANCE_COLLECTOR_URL="https://your-airspace-instance.example/api/compliance/attestations"
```

## Product API Attestation

Example attestation:

```http
POST /api/compliance/attestations
Content-Type: application/json

{
  "organization": "Example Ops Lab",
  "purpose": "Internal evaluation",
  "useType": "EVALUATION",
  "sourceCodeUrl": "https://example.invalid/airspace-fork",
  "operationalConfigUrl": "https://example.invalid/airspace-config",
  "publicDisclosureUrl": "https://example.invalid/disclosure",
  "acknowledgesAgpl": true,
  "acknowledgesPublicUseAndAiPolicy": true,
  "sourcePublished": true,
  "operationalConfigPublished": true,
  "secretsRedacted": true,
  "actor": "compliance-officer"
}
```

Incomplete attestations are retained with diagnostics so reviewers can see what still needs disclosure.

## Public Registry

The preferred public registry is a GitHub issue using the Airspace use-disclosure template:

```text
https://github.com/tashiscool/Airspace/issues/new?template=airspace-use-disclosure.yml
```

This keeps compliance public and reviewable without creating a secret telemetry database.

## Source References

- GNU AGPL-3.0: https://www.gnu.org/licenses/agpl-3.0.html
- OSI AGPL-3.0 page: https://opensource.org/license/agpl-3-0
- SPDX `AGPL-3.0-or-later`: https://spdx.org/licenses/AGPL-3.0-or-later.html
