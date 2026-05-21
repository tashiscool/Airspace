export type JsonRecord = Record<string, unknown>;

export type DecisionTraceGroup = {
  stage: string;
  count: number;
  warningCount: number;
  ruleIds: string[];
  steps: JsonRecord[];
};

export type DecisionSourceLink = {
  type: string;
  id: string;
  label: string;
  route?: string;
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

export function decisionRoutePredictions(result?: JsonRecord): JsonRecord[] {
  return arrayValue(result?.routeImpacts)
    ?? arrayValue(result?.routeBlockages)
    ?? arrayValue(result?.routeBlockagePredictions)
    ?? arrayValue(result?.routeWeatherImpacts)
    ?? [];
}

export function decisionAvoidanceCandidates(result?: JsonRecord): JsonRecord[] {
  const direct = arrayValue(result?.avoidanceCandidates);
  if (direct?.length) return direct;
  const routePlan = asRecord(result?.routePlanResult);
  return arrayValue(routePlan?.candidates) ?? [];
}

export function decisionSourceReferences(trace: JsonRecord[], result?: JsonRecord) {
  const refs = new Set<string>();
  const visit = (value: unknown) => {
    if (value == null) return;
    if (typeof value === 'string') {
      if (/\b(PIREP|SIGMET|AIRMET|METAR|TAF|NOTAM|USNS|CARF|ALTRV|WX|WEATHER)\b/i.test(value) || /^[a-f0-9-]{20,}$/i.test(value)) {
        refs.add(value.length > 80 ? `${value.slice(0, 77)}...` : value);
      }
      return;
    }
    if (Array.isArray(value)) {
      value.forEach(visit);
      return;
    }
    const record = asRecord(value);
    if (!record) return;
    for (const key of ['sourceId', 'sourceIds', 'sourceRef', 'sourceRefs', 'messageId', 'messageIds', 'productId', 'productIds', 'notamId', 'pirepId', 'reservationId']) {
      if (record[key] != null) visit(record[key]);
    }
  };
  trace.forEach(visit);
  visit(result?.sourceRefs);
  visit(result?.blockingConstraints);
  visit(result?.routeImpacts);
  visit(result?.routeBlockages);
  visit(result?.weatherProducts);
  visit(result?.pirepResults);
  return [...refs].slice(0, 16);
}

export function decisionSourceLinks(trace: JsonRecord[], result?: JsonRecord): DecisionSourceLink[] {
  const links = new Map<string, DecisionSourceLink>();
  const add = (type: string | undefined, id: string | undefined, description?: string) => {
    const safeId = id?.trim();
    if (!safeId) return;
    const safeType = sourceType(type, safeId);
    const key = `${safeType}:${safeId}`;
    if (!links.has(key)) {
      links.set(key, {
        type: safeType,
        id: safeId,
        label: description || `${safeType}:${safeId}`,
        route: sourceRoute(safeType, safeId)
      });
    }
  };
  const visit = (value: unknown, fallbackType?: string) => {
    if (value == null) return;
    if (typeof value === 'string') {
      const parsed = parseSourceRef(value, fallbackType);
      add(parsed.type, parsed.id, parsed.label);
      return;
    }
    if (Array.isArray(value)) {
      value.forEach((item) => visit(item, fallbackType));
      return;
    }
    const record = asRecord(value);
    if (!record) return;
    const type = stringValue(record.type) ?? stringValue(record.sourceType) ?? fallbackType;
    const id = stringValue(record.id) ?? stringValue(record.sourceId) ?? stringValue(record.productId)
      ?? stringValue(record.messageId) ?? stringValue(record.pirepId) ?? stringValue(record.notamId)
      ?? stringValue(record.reservationId);
    add(type, id, stringValue(record.description) ?? stringValue(record.label) ?? stringValue(record.name));
    for (const key of ['sources', 'sourceRefs', 'sourceRef', 'sourceIds', 'messageIds', 'productIds', 'blockingConstraints', 'routeImpacts']) {
      if (record[key] != null) visit(record[key], type);
    }
  };
  trace.forEach((step) => visit(step));
  visit(result?.sourceRefs);
  visit(result?.blockingConstraints);
  visit(result?.routeImpacts);
  visit(result?.routeBlockages);
  visit(result?.weatherProducts, 'WEATHER');
  visit(result?.pirepResults, 'PIREP');
  return [...links.values()].slice(0, 24);
}

function parseSourceRef(value: string, fallbackType?: string) {
  const trimmed = value.trim();
  const typed = /^([A-Z_ -]+):(.+)$/i.exec(trimmed);
  if (typed) {
    const type = sourceType(typed[1], typed[2]);
    const id = typed[2].trim();
    return { type, id, label: `${type}:${id}` };
  }
  const type = sourceType(fallbackType, trimmed);
  return { type, id: trimmed, label: `${type}:${trimmed}` };
}

function sourceType(type: string | undefined, id: string) {
  const text = `${type ?? ''} ${id}`.toUpperCase();
  if (text.includes('PIREP')) return 'PIREP';
  if (text.includes('SIGMET') || text.includes('AIRMET') || text.includes('METAR') || text.includes('TAF') || text.includes('WEATHER') || text.includes('WX')) return 'WEATHER';
  if (text.includes('NOTAM')) return 'NOTAM';
  if (text.includes('USNS') || text.includes('MESSAGE')) return 'MESSAGE';
  if (text.includes('CARF') || text.includes('ALTRV') || text.includes('RESERVATION')) return 'RESERVATION';
  if (text.includes('SECTOR')) return 'SECTOR';
  return (type ?? 'SOURCE').toUpperCase().replace(/\s+/g, '_');
}

function sourceRoute(type: string, id: string) {
  if (type === 'MESSAGE' || type === 'WEATHER' || type === 'PIREP' || type === 'NOTAM') return `/messages/${encodeURIComponent(id)}`;
  if (type === 'RESERVATION') return `/deconfliction/${encodeURIComponent(id)}`;
  return undefined;
}

function stringValue(value: unknown) {
  return value == null ? undefined : String(value);
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
