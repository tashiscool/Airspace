# Agentic Stability Harness

Airspace includes a local stability harness for agent runs. It is designed to measure repeatability, not prove correctness.

Normal agent runs now apply a stability gate automatically. Deterministic draft-only runs use `WARN_ONLY` mode by default, while model-backed runs use `ENFORCE` mode by default. Manual stability evaluation remains available through the API for deeper review.

## Purpose

The harness runs the same `AgentRunRequest` multiple times and compares non-volatile outputs. IDs, timestamps, and audit times are ignored. The comparison focuses on whether the agent reaches the same advisory shape from the same local evidence.

## Metrics

- Accepted/verdict agreement.
- Finding, recommendation, and task count wobble.
- Cited-source Jaccard similarity.
- Finding category agreement.
- Recommendation action agreement.
- Task route/priority agreement.
- Tool receipt hash agreement.

Default deterministic thresholds are intentionally strict: agreement must be `1.0`, citation Jaccard must be at least `0.95`, and count wobble must be `0.0`.

## API

`POST /api/agents/evaluate/stability`

```json
{
  "iterations": 3,
  "agentRunRequest": {
    "agentType": "MISSION_RISK",
    "missionId": "mission-1"
  }
}
```

The response includes accepted state, run ids, metrics, thresholds, and diagnostics.

Automatic run results also embed the stability result in `AgentEvaluationSummary.stabilityResult` with:

- `stabilityAppliedAutomatically`,
- `stabilityMode`,
- `stabilityAccepted`,
- metric details and diagnostics.

Automatic probe runs use the same agent execution path but skip their own stability gate and are not stored as normal run-history entries. This prevents recursive checks and keeps audit history focused on operator-visible runs.

## Safety Boundary

The harness does not authorize operational action. It is an evaluation gate for deterministic/local and future model-backed agent output. Any unstable result should be treated as review-blocking until the cause is understood. In `ENFORCE` mode, unstable model-backed output is rejected by evaluation before it can be treated as accepted guidance.
