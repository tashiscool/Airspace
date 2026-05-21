export type JsonRecord = Record<string, unknown>;

export type DecisionTraceGroup = {
  stage: string;
  count: number;
  warningCount: number;
  ruleIds: string[];
  steps: JsonRecord[];
};

export function normalizeDecisionTrace(result?: JsonRecord): JsonRecord[] {
  const trace = asRecord(result?.trace);
  const decisionTrace = asRecord(result?.decisionTrace);
  return arrayValue(trace?.steps) ?? arrayValue(decisionTrace?.steps) ?? arrayValue(result?.trace) ?? [];
}

export function groupDecisionTrace(steps: JsonRecord[]): DecisionTraceGroup[] {
  const groups = new Map<string, DecisionTraceGroup>();
  for (const step of steps) {
    const stage = traceStage(step);
    const group = groups.get(stage) ?? { stage, count: 0, warningCount: 0, ruleIds: [], steps: [] };
    group.count += 1;
    group.steps.push(step);
    if (isWarningStep(step)) group.warningCount += 1;
    const ruleId = recordLabel(step, ['ruleId', 'rule', 'decisionRuleId'], '');
    if (ruleId && !group.ruleIds.includes(ruleId)) group.ruleIds.push(ruleId);
    groups.set(stage, group);
  }
  return [...groups.values()].sort((a, b) => stageOrder(a.stage) - stageOrder(b.stage) || a.stage.localeCompare(b.stage));
}

export function traceStage(step: unknown) {
  return recordLabel(step, ['stage', 'category', 'ruleId'], 'TRACE').toUpperCase();
}

export function recordLabel(value: unknown, keys: string[], fallback: string) {
  if (typeof value === 'string') return value;
  const record = asRecord(value);
  if (!record) return fallback;
  for (const key of keys) {
    const candidate = record[key];
    if (candidate != null) return String(candidate);
  }
  return fallback;
}

export function asRecord(value: unknown): JsonRecord | undefined {
  return value && typeof value === 'object' && !Array.isArray(value) ? value as JsonRecord : undefined;
}

export function arrayValue(value: unknown): JsonRecord[] | undefined {
  return Array.isArray(value) ? value.map((item) => asRecord(item) ?? { message: String(item) }) : undefined;
}

function isWarningStep(step: JsonRecord) {
  const text = JSON.stringify(step).toUpperCase();
  return text.includes('WARN') || text.includes('STALE') || text.includes('LOW_CONFIDENCE') || text.includes('REJECT');
}

function stageOrder(stage: string) {
  const order = ['PARSE', 'CLASSIFY', 'MAP', 'FUSE', 'INDEX', 'ROUTE_IMPACT', 'CONFLICT', 'BLOCKAGE', 'RULE', 'ACTION', 'WARNING', 'TRACE'];
  const index = order.findIndex((item) => stage.includes(item));
  return index === -1 ? order.length : index;
}
