# Agentic MCP Tool Plane

Airspace includes a governed MCP-style tool plane for the agentic operations layer. It is intentionally conservative: tools provide cited evidence and drafts, while official message sends and workflow mutations remain human-approved operator actions.

## Default Servers

- `airspace-first-party`: enabled by default, local, direct Java execution, no credentials, no live external dependency.
- `external-mcp`: disabled/setup-required by default. It is a placeholder for future stdio/SSE/HTTP MCP servers after credentials, command allowlists, and operator consent are configured.

## First-Party Tools

| Tool | Side effect | Purpose |
|---|---:|---|
| `airspace.weather.affected_missions` | `READ_ONLY` | List missions affected by weather, PIREP, NOTAM, or route-impact sources. |
| `airspace.mission.weather_verdict` | `READ_ONLY` | Return a mission weather/NOTAM/PIREP verdict. |
| `airspace.mission.route_impact` | `READ_ONLY` | Return route-impact and reroute summary for a mission/reservation. |
| `airspace.pireps.relevant` | `READ_ONLY` | Filter PIREPs by mission corridor, altitude tolerance, and recency. |
| `airspace.decision.summary` | `READ_ONLY` | Fetch a persisted operational decision summary. |
| `airspace.decision.replay_audit` | `READ_ONLY` | Replay and verify a persisted decision bundle when available. |
| `airspace.feed.artifact_transactions` | `READ_ONLY` | List parsed transactions for a retained feed artifact. |
| `airspace.reference.lookup` | `READ_ONLY` | Lookup navaids, fixes, and reference points. |
| `airspace.coordination.draft` | `DRAFT_ONLY` | Draft weather/ATC coordination text. No message is sent. |

## Evidence Receipts

Every MCP invocation returns a redacted `McpEvidenceReceipt` with:

- server id, tool id, side-effect level, status, policy decision,
- redacted input/output summaries,
- deterministic input/output hashes,
- timing and duration,
- source refs/citations,
- diagnostics for denied, setup-required, skipped, or failed calls.

The agent run can attach `AgentToolCall` records that reference receipt ids. The reasoning envelope also lists available tools, blocked/setup-required tools, and receipt ids so replay/audit can answer why a tool was or was not used.

## Risk And Review Metadata

Every curated MCP tool now carries an `AgenticRiskProfile` with autonomy scope, tool surface, worst-case blast radius, permission scope, data-egress behavior, poisoned-data exposure, audit attribution, tool/model provenance, rollback path, cost/SLA class, and required human-review mode.

`GET /api/agents/risk-assessments` returns the same checklist for operator and reviewer display. First-party tools are local/read-only or draft-only. The external MCP placeholder is setup-required and requires push approval before any future configured use.

Agent output uses explicit human-review modes:

- `REVIEW_ONLY` for advisory evidence.
- `PULL_CLARIFICATION` for missing, ambiguous, malformed, contradictory, or unsupported evidence.
- `PUSH_APPROVAL` for drafts and official-action-adjacent recommendations.

Agent runs also include fixed-shape `AgentAssessment` envelopes with claim, verdict, evidence, counter-evidence, uncertainty, required human action, review mode, and citations.

## Stability Evaluation

`POST /api/agents/evaluate/stability` runs the same agent request repeatedly and compares non-volatile behavior. Metrics include verdict agreement, source-ref Jaccard, finding/recommendation/task count wobble, action/category agreement, task route/priority agreement, and MCP receipt hash agreement.

The default deterministic thresholds are strict. Stability results are evaluation evidence only; they do not authorize operational action.

## Queue Consumer

`AgentJobQueueService` and `AgentJobConsumer` provide an in-process first-party job flow:

1. enqueue an `AgentJobRequest`,
2. plan policy-allowed curated tools,
3. execute local first-party MCP tools,
4. attach tool calls and receipts,
5. run the existing deterministic agent,
6. retain the agent run and review tasks.

The default queue is in-memory and bounded. It is designed for local/demo and testable product workflows, not as a distributed worker replacement.

## Public Endpoints

- `GET /api/agents/mcp/servers`
- `GET /api/agents/mcp/servers/{serverId}/tools`
- `POST /api/agents/mcp/tools/call`
- `GET /api/agents/mcp/receipts`
- `POST /api/agents/jobs`
- `GET /api/agents/jobs`
- `GET /api/agents/jobs/{id}`

## Safety Defaults

- `airspace.agentic.mcp.enabled=true`
- `airspace.agentic.mcp.external.enabled=false`
- `airspace.agentic.queue.max-size=100`
- `airspace.agentic.queue.auto-consume=true`

External MCP remains setup-required until explicitly configured. The default `AgentPolicy` rejects mutating and external-send behavior. This follows the project’s agent rule: collect, normalize, correlate, draft, explain, and escalate, but do not autonomously clear, approve, transmit, or mutate official operational state.
