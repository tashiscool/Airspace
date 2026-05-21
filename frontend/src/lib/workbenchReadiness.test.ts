import { describe, expect, it } from 'vitest';
import { remainingWorkbenchGaps, workbenchReadinessSummary } from './workbenchReadiness';

describe('workbench readiness audit', () => {
  it('computes a weighted readiness summary', () => {
    expect(workbenchReadinessSummary([
      { id: 'a', label: 'A', status: 'complete', note: '' },
      { id: 'b', label: 'B', status: 'partial', note: '' },
      { id: 'c', label: 'C', status: 'missing', note: '' }
    ])).toEqual({
      total: 3,
      complete: 1,
      partial: 1,
      missing: 1,
      percent: 52
    });
  });

  it('lists only remaining non-complete gaps', () => {
    const gaps = remainingWorkbenchGaps([
      { id: 'a', label: 'A', status: 'complete', note: '' },
      { id: 'b', label: 'B', status: 'partial', note: '' }
    ]);

    expect(gaps.map((gap) => gap.id)).toEqual(['b']);
  });
});
