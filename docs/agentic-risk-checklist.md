# Agentic Risk Checklist

Airspace exposes a risk checklist for curated MCP tools and agentic agent surfaces. The goal is to make autonomy scope and worst-case blast radius visible before any operator relies on an agent result.

## Checklist Fields

- Autonomy scope.
- Tool surface.
- Worst-case blast radius.
- Permission scope.
- Data egress.
- Poisoned-data exposure.
- Audit attribution.
- Tool provenance.
- Model provenance.
- Rollback path.
- Cost class.
- SLA class.
- Required human-review mode.

## API

`GET /api/agents/risk-assessments`

Optional filter:

- `?subjectType=AGENT`
- `?subjectType=MCP_TOOL`
- `?subjectType=ALL`

Each assessment is tied to a tool or agentic surface and includes diagnostics when a tool is external, setup-required, or missing explicit metadata.

## Defaults

Every built-in agent type has an explicit risk profile:

- Weather Impact Watch,
- Mission Risk Analyst,
- Reroute Analyst,
- Coordination Draft,
- Pilot Brief,
- Data Integrity,
- Replay Audit,
- aggregate `ALL`.

First-party Airspace tools are local and either `READ_ONLY` or `DRAFT_ONLY`. Local mock-live MCP tools are enabled for dev/test validation and clearly marked as mock outputs. Real external MCP remains setup-required by default. Draft tools require `PUSH_APPROVAL`; read-only advisory tools require at least `REVIEW_ONLY`.

## Human Review Modes

- `NONE`: no human action required.
- `REVIEW_ONLY`: advisory output should be reviewed with citations.
- `PULL_CLARIFICATION`: missing, ambiguous, stale, malformed, or conflicting evidence needs clarification.
- `PUSH_APPROVAL`: draft or official-action-adjacent output requires human approval before use.

No agentic risk profile changes the core Airspace boundary: agents can cite, explain, draft, and open review tasks, but they cannot autonomously send official messages or mutate operational workflow state.
