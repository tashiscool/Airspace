import { describe, expect, it } from 'vitest';
import { groupDecisionTrace, normalizeDecisionTrace, traceStage } from './decisionView';

describe('decision view helpers', () => {
  it('normalizes trace steps from persisted result shapes', () => {
    expect(normalizeDecisionTrace({ trace: { steps: [{ stage: 'parse', ruleId: 'P1' }] } })).toEqual([{ stage: 'parse', ruleId: 'P1' }]);
    expect(normalizeDecisionTrace({ decisionTrace: { steps: ['classified'] } })).toEqual([{ message: 'classified' }]);
  });

  it('groups trace stages with rule ids and warnings', () => {
    const groups = groupDecisionTrace([
      { stage: 'route_impact', ruleId: 'WX-ROUTE-001', message: 'blocked' },
      { category: 'warning', ruleId: 'WX-STALE-001', message: 'stale product warning' },
      { stage: 'parse', ruleId: 'USNS-PARSE-001' }
    ]);

    expect(traceStage({ category: 'warning' })).toBe('WARNING');
    expect(groups.map((group) => group.stage)).toEqual(['PARSE', 'ROUTE_IMPACT', 'WARNING']);
    expect(groups.find((group) => group.stage === 'WARNING')).toMatchObject({
      warningCount: 1,
      ruleIds: ['WX-STALE-001']
    });
  });
});
