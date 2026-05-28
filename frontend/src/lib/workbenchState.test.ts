import { describe, expect, it, vi, beforeEach } from 'vitest';
import {
  actionsForWorkbenchRow,
  conflictReviewStorageKey,
  DEFAULT_LAYOUT_PREFS,
  isAttentionRow,
  readAcceptedConflictReviews,
  readWorkbenchJson,
  routeForWorkbenchAction,
  routeForWorkbenchRow,
  sourceFamilyForRow,
  sourceFamilyLabel,
  selectionFromRow,
  writeAcceptedConflictReviews
} from './workbenchState';
import type { ExplorerRow } from './viewModels';

const reservation: ExplorerRow = {
  key: 'reservation:r1',
  family: 'RESERVATION',
  id: 'r1',
  title: 'Reservation r1',
  subtitle: 'M1 · 2 conflicts',
  status: 'CONFLICT',
  route: '/missions/m1/reservations/r1',
  preview: 'Needs review'
};

beforeEach(() => {
  const store = new Map<string, string>();
  vi.stubGlobal('localStorage', {
    getItem: vi.fn((key: string) => store.get(key) ?? null),
    setItem: vi.fn((key: string, value: string) => store.set(key, value))
  });
});

describe('workbench state helpers', () => {
  it('derives row routes and workflow actions', () => {
    expect(routeForWorkbenchRow(reservation)).toBe('/missions/m1/reservations/r1');
    expect(actionsForWorkbenchRow(reservation)).toEqual(['open', 'parse', 'deconflict', 'review-conflicts']);
    expect(routeForWorkbenchAction('parse', reservation)).toBe('/missions/m1/reservations/r1');
    expect(routeForWorkbenchAction('deconflict', reservation)).toBe('/deconfliction/r1');
    expect(routeForWorkbenchAction('open-notams', reservation)).toBe('/notams');
  });

  it('keeps source-family labels precise', () => {
    expect(sourceFamilyForRow(reservation)).toBe('CARF_ALTRV');
    expect(sourceFamilyLabel('NOTAM')).toBe('NOTAM constraint');
    expect(sourceFamilyForRow({
      ...reservation,
      family: 'MESSAGE',
      title: 'SIGMET weather advisory',
      subtitle: 'INBOUND · SIGMET',
      preview: 'convective weather'
    })).toBe('WEATHER');
  });

  it('preserves documented source-family selections across major workbench surfaces', () => {
    const rows: ExplorerRow[] = [
      { ...reservation, family: 'MISSION', id: 'm1', missionId: 'm1', title: 'Mission M1', route: '/missions/m1' },
      { ...reservation, family: 'RESERVATION', id: 'r1', reservationId: 'r1', route: '/missions/m1/reservations/r1' },
      { ...reservation, family: 'MESSAGE', id: 'msg-wx', title: 'SIGMET weather', subtitle: 'SIGMET', preview: 'weather route impact', route: '/messages/msg-wx' },
      { ...reservation, family: 'MESSAGE', id: 'msg-pirep', title: 'PIREP UA', subtitle: 'PIREP', preview: 'urgent PIREP', route: '/messages/msg-pirep' },
      { ...reservation, family: 'NOTAM', id: 'notam-1', title: 'FDC NOTAM', subtitle: 'NOTAM constraint', route: '/notams' },
      { ...reservation, family: 'FEED', id: 'feed-1', title: 'USNS feed', subtitle: 'USNS', preview: 'retained traffic', route: '/feed/feed-1' },
      { ...reservation, family: 'DECISION', id: 'decision-1', title: 'Decision BLOCKED', route: '/decisions/decision-1' }
    ];

    expect(rows.map((row) => selectionFromRow(row).sourceFamily)).toEqual([
      'CARF_ALTRV',
      'CARF_ALTRV',
      'WEATHER',
      'PIREP',
      'NOTAM',
      'USNS',
      'DECISION'
    ]);
    expect(selectionFromRow(rows[2]).messageId).toBe('msg-wx');
    expect(selectionFromRow(rows[5]).feedArtifactId).toBe('feed-1');
    expect(selectionFromRow(rows[6]).decisionId).toBe('decision-1');
  });

  it('marks conflict and rejected rows as attention rows', () => {
    expect(isAttentionRow(reservation)).toBe(true);
    expect(readWorkbenchJson('missing', DEFAULT_LAYOUT_PREFS).railCollapsed).toBe(false);
  });

  it('persists accepted conflict review ids per reservation', () => {
    writeAcceptedConflictReviews('r1', new Set(['c2', 'c1']));

    expect(localStorage.setItem).toHaveBeenCalledWith(conflictReviewStorageKey('r1'), JSON.stringify(['c1', 'c2']));
    expect([...readAcceptedConflictReviews('r1')]).toEqual(['c1', 'c2']);
    expect([...readAcceptedConflictReviews('r2')]).toEqual([]);
  });
});
