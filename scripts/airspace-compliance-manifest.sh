#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

repo_url="$(git config --get remote.origin.url 2>/dev/null || true)"
commit_sha="$(git rev-parse HEAD 2>/dev/null || true)"
branch_name="$(git rev-parse --abbrev-ref HEAD 2>/dev/null || true)"
generated_at="$(date -u +"%Y-%m-%dT%H:%M:%SZ")"

organization="${AIRSPACE_COMPLIANCE_ORGANIZATION:-}"
purpose="${AIRSPACE_COMPLIANCE_PURPOSE:-}"
use_type="${AIRSPACE_COMPLIANCE_USE_TYPE:-EVALUATION}"
project_url="${AIRSPACE_COMPLIANCE_PROJECT_URL:-}"
source_code_url="${AIRSPACE_COMPLIANCE_SOURCE_CODE_URL:-}"
operational_config_url="${AIRSPACE_COMPLIANCE_OPERATIONAL_CONFIG_URL:-}"
public_disclosure_url="${AIRSPACE_COMPLIANCE_PUBLIC_DISCLOSURE_URL:-}"
collector_url="${AIRSPACE_COMPLIANCE_COLLECTOR_URL:-}"
submit="${AIRSPACE_COMPLIANCE_SUBMIT:-0}"

escape_json() {
  local value="${1:-}"
  value="${value//\\/\\\\}"
  value="${value//\"/\\\"}"
  value="${value//$'\n'/\\n}"
  value="${value//$'\r'/\\r}"
  printf '%s' "$value"
}

manifest="$(cat <<JSON
{
  "projectName": "Airspace",
  "policyVersion": "airspace-public-use-ai-transparency-v1",
  "licenseSpdx": "AGPL-3.0-or-later",
  "generatedAt": "$(escape_json "$generated_at")",
  "repositoryUrl": "$(escape_json "$repo_url")",
  "commitSha": "$(escape_json "$commit_sha")",
  "branch": "$(escape_json "$branch_name")",
  "organization": "$(escape_json "$organization")",
  "purpose": "$(escape_json "$purpose")",
  "useType": "$(escape_json "$use_type")",
  "projectUrl": "$(escape_json "$project_url")",
  "sourceCodeUrl": "$(escape_json "$source_code_url")",
  "operationalConfigUrl": "$(escape_json "$operational_config_url")",
  "publicDisclosureUrl": "$(escape_json "$public_disclosure_url")",
  "acknowledgesAgpl": ${AIRSPACE_COMPLIANCE_ACK_AGPL:-false},
  "acknowledgesPublicUseAndAiPolicy": ${AIRSPACE_COMPLIANCE_ACK_PUBLIC_USE_POLICY:-false},
  "sourcePublished": ${AIRSPACE_COMPLIANCE_SOURCE_PUBLISHED:-false},
  "operationalConfigPublished": ${AIRSPACE_COMPLIANCE_OPERATIONAL_CONFIG_PUBLISHED:-false},
  "secretsRedacted": ${AIRSPACE_COMPLIANCE_SECRETS_REDACTED:-false},
  "hiddenTelemetryEnabled": false,
  "cloneTimeCollectionEnabled": false,
  "notes": "Generated locally by scripts/airspace-compliance-manifest.sh. No network request is made unless AIRSPACE_COMPLIANCE_SUBMIT=1 and AIRSPACE_COMPLIANCE_COLLECTOR_URL are set."
}
JSON
)"

printf '%s\n' "$manifest"

if [[ "$submit" == "1" ]]; then
  if [[ -z "$collector_url" ]]; then
    echo "AIRSPACE_COMPLIANCE_SUBMIT=1 was set, but AIRSPACE_COMPLIANCE_COLLECTOR_URL is empty." >&2
    exit 2
  fi
  if ! command -v curl >/dev/null 2>&1; then
    echo "curl is required for explicit attestation submission." >&2
    exit 2
  fi
  printf '%s\n' "$manifest" | curl --fail-with-body \
    -H "Content-Type: application/json" \
    --data-binary @- \
    "$collector_url"
fi
