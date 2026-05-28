import { describe, expect, it } from 'vitest';
import { decisionAvoidanceCandidates, decisionRoutePredictions, decisionSourceLinks, decisionSourceReferences, groupDecisionTrace, normalizeDecisionTrace, traceStage } from './decisionView';

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

  it('normalizes route impacts, avoidance candidates, and source references', () => {
    const result = {
      routeImpacts: [{ primaryHazardId: 'WX-1', sourceRefs: ['WEATHER:WX-1'], blockedProbability: 0.9 }],
      routePlanResult: { candidates: [{ id: 'north-east', avoidedConstraintIds: ['WX-1'] }] },
      sourceRefs: [{ type: 'WEATHER', id: 'WX-1' }]
    };

    expect(decisionRoutePredictions(result)).toHaveLength(1);
    expect(decisionAvoidanceCandidates(result)).toEqual([{ id: 'north-east', avoidedConstraintIds: ['WX-1'] }]);
    expect(decisionSourceReferences([], result)).toEqual(expect.arrayContaining(['WEATHER:WX-1']));
  });

  it('builds typed source links from trace and result source references', () => {
    const trace = [
      {
        stage: 'route-impact',
        sources: [{ type: 'PIREP', id: 'UA-1', description: 'urgent turbulence report' }]
      },
      {
        stage: 'fuse',
        sourceRefs: ['NOTAM:ABC-123']
      }
    ];
    const result = {
      sourceRefs: [{ type: 'WEATHER', id: 'SIGMET-9', description: 'convective sigmet' }],
      routeImpacts: [{ primaryHazardId: 'WX-1', sourceRefs: ['WEATHER:WX-1'] }]
    };

    expect(decisionSourceLinks(trace, result)).toEqual(expect.arrayContaining([
      expect.objectContaining({ type: 'PIREP', id: 'UA-1', route: '/messages/UA-1' }),
      expect.objectContaining({ type: 'NOTAM', id: 'ABC-123', route: '/messages/ABC-123' }),
      expect.objectContaining({ type: 'WEATHER', id: 'SIGMET-9', route: '/messages/SIGMET-9' }),
      expect.objectContaining({ type: 'WEATHER', id: 'WX-1', route: '/messages/WX-1' })
    ]));
  });

  it('routes every documented decision source family to the correct workspace', () => {
    const result = {
      sourceRefs: [
        'WEATHER:SIGMET-1',
        'PIREP:UA-1',
        'NOTAM:FDC-1',
        'USNS:MSG-1',
        'CARF:RES-1',
        'ALTRV:RES-2'
      ]
    };

    expect(decisionSourceLinks([], result)).toEqual(expect.arrayContaining([
      expect.objectContaining({ type: 'WEATHER', id: 'SIGMET-1', route: '/messages/SIGMET-1' }),
      expect.objectContaining({ type: 'PIREP', id: 'UA-1', route: '/messages/UA-1' }),
      expect.objectContaining({ type: 'NOTAM', id: 'FDC-1', route: '/messages/FDC-1' }),
      expect.objectContaining({ type: 'MESSAGE', id: 'MSG-1', route: '/messages/MSG-1' }),
      expect.objectContaining({ type: 'RESERVATION', id: 'RES-1', route: '/deconfliction/RES-1' }),
      expect.objectContaining({ type: 'RESERVATION', id: 'RES-2', route: '/deconfliction/RES-2' })
    ]));
  });
});
