import type { ExplorerRow } from './viewModels';
import { isNotamFamily } from './viewModels';

export type WorkbenchSourceFamily =
  | 'CARF_ALTRV'
  | 'NOTAM'
  | 'USNS'
  | 'WEATHER'
  | 'PIREP'
  | 'REFERENCE'
  | 'DECISION'
  | 'SIMULATION'
  | 'TFM'
  | 'OUTCOME'
  | 'UNKNOWN';

export type WorkbenchSelection = {
  missionId?: string;
  reservationId?: string;
  messageId?: string;
  feedArtifactId?: string;
  decisionId?: string;
  sourceFamily?: WorkbenchSourceFamily;
  label?: string;
  lockState?: string;
  conflictCount?: number;
};

export type WorkbenchLayoutPrefs = {
  railCollapsed: boolean;
  explorerFilters: {
    families: string[];
    statuses: string[];
    attentionOnly: boolean;
  };
  mapLayers: string[];
  reservationEditorMode: 'sections' | 'raw';
  reservationSupplementTab: string;
};

export type WorkbenchAction =
  | 'open'
  | 'lock'
  | 'new-reservation'
  | 'parse'
  | 'deconflict'
  | 'review-conflicts'
  | 'reply'
  | 'forward'
  | 'archive'
  | 'inspect-transactions'
  | 'open-notams'
  | 'open-replay';

export type WorkbenchCommand = {
  id: string;
  label: string;
  route?: string;
  action?: WorkbenchAction;
  sourceFamily?: WorkbenchSourceFamily;
};

export const DEFAULT_LAYOUT_PREFS: WorkbenchLayoutPrefs = {
  railCollapsed: false,
  explorerFilters: {
    families: [],
    statuses: [],
    attentionOnly: false
  },
  mapLayers: [],
  reservationEditorMode: 'sections',
  reservationSupplementTab: 'CONSOLE'
};

export function readWorkbenchJson<T>(key: string, fallback: T): T {
  if (typeof localStorage === 'undefined') return fallback;
  try {
    const raw = localStorage.getItem(key);
    return raw ? { ...fallback, ...JSON.parse(raw) } as T : fallback;
  } catch {
    return fallback;
  }
}

export function writeWorkbenchJson<T>(key: string, value: T) {
  if (typeof localStorage === 'undefined') return;
  localStorage.setItem(key, JSON.stringify(value));
}

export function conflictReviewStorageKey(reservationId: string) {
  return `airspace.workbench.conflictReviews.${reservationId}`;
}

export function readAcceptedConflictReviews(reservationId: string): Set<string> {
  if (!reservationId || typeof localStorage === 'undefined') return new Set();
  try {
    const raw = localStorage.getItem(conflictReviewStorageKey(reservationId));
    const values = raw ? JSON.parse(raw) : [];
    return new Set(Array.isArray(values) ? values.map(String) : []);
  } catch {
    return new Set();
  }
}

export function writeAcceptedConflictReviews(reservationId: string, conflictIds: Set<string>) {
  if (!reservationId || typeof localStorage === 'undefined') return;
  localStorage.setItem(conflictReviewStorageKey(reservationId), JSON.stringify([...conflictIds].sort()));
}

export function sourceFamilyForRow(row?: ExplorerRow): WorkbenchSourceFamily {
  if (!row) return 'UNKNOWN';
  if (row.family === 'MISSION' || row.family === 'RESERVATION') return 'CARF_ALTRV';
  if (row.family === 'NOTAM' || isNotamFamily(row.title) || isNotamFamily(row.subtitle)) return 'NOTAM';
  if (row.family === 'FEED' || row.family === 'MESSAGE') {
    const text = `${row.title} ${row.subtitle} ${row.preview}`.toUpperCase();
    if (text.includes('PIREP')) return 'PIREP';
    if (text.includes('SIGMET') || text.includes('AIRMET') || text.includes('METAR') || text.includes('TAF') || text.includes('WEATHER')) return 'WEATHER';
    return 'USNS';
  }
  if (row.family === 'DECISION') return 'DECISION';
  return 'UNKNOWN';
}

export function sourceFamilyLabel(family?: WorkbenchSourceFamily) {
  switch (family) {
    case 'CARF_ALTRV': return 'CARF/ALTRV reservation';
    case 'NOTAM': return 'NOTAM constraint';
    case 'USNS': return 'USNS traffic';
    case 'WEATHER': return 'Weather constraint';
    case 'PIREP': return 'PIREP report';
    case 'REFERENCE': return 'Reference data';
    case 'DECISION': return 'Operational decision';
    case 'SIMULATION': return 'Simulation playback';
    case 'TFM': return 'Traffic Flow Management board';
    case 'OUTCOME': return 'Operational outcome metrics';
    default: return 'Unknown source';
  }
}

export function routeForWorkbenchRow(row?: ExplorerRow) {
  return row?.route;
}

export function routeForWorkbenchAction(action: WorkbenchAction, row?: ExplorerRow) {
  if (!row) return undefined;
  if (action === 'open' || action === 'lock') return row.route;
  if (action === 'new-reservation') return row.missionId ? `/missions/${row.missionId}` : row.route;
  if (action === 'parse') return row.route;
  if (action === 'deconflict' || action === 'review-conflicts') return row.reservationId || row.family === 'RESERVATION'
    ? `/deconfliction/${row.reservationId ?? row.id}`
    : undefined;
  if (action === 'reply' || action === 'forward' || action === 'archive') return `/messages/${row.id}`;
  if (action === 'inspect-transactions') return `/feed/${row.id}`;
  if (action === 'open-notams') return '/notams';
  if (action === 'open-replay') return `/decisions/${row.id}`;
  return row.route;
}

export function actionsForWorkbenchRow(row?: ExplorerRow): WorkbenchAction[] {
  if (!row) return [];
  if (row.family === 'MISSION') return ['open', 'lock', 'new-reservation'];
  if (row.family === 'RESERVATION') return ['open', 'parse', 'deconflict', 'review-conflicts'];
  if (row.family === 'MESSAGE') return ['open', 'reply', 'forward', 'archive'];
  if (row.family === 'NOTAM') return ['open-notams', 'reply', 'forward'];
  if (row.family === 'FEED') return ['inspect-transactions'];
  if (row.family === 'DECISION') return ['open', 'open-replay'];
  return ['open'];
}

export function isAttentionRow(row: ExplorerRow) {
  const status = String(row.status ?? '').toUpperCase();
  const text = `${row.title} ${row.subtitle} ${row.preview}`.toUpperCase();
  return status.includes('CONFLICT')
    || status.includes('REJECT')
    || status.includes('PENDING')
    || text.includes('CONFLICT')
    || text.includes('REJECT')
    || text.includes('STALE')
    || text.includes('UNREVIEWED')
    || text.includes('LOCKED BY');
}

export function selectionFromRow(row?: ExplorerRow): WorkbenchSelection {
  return {
    missionId: row?.missionId ?? (row?.family === 'MISSION' ? row.id : undefined),
    reservationId: row?.reservationId,
    messageId: row?.family === 'MESSAGE' || row?.family === 'NOTAM' ? row.id : undefined,
    feedArtifactId: row?.family === 'FEED' ? row.id : undefined,
    decisionId: row?.family === 'DECISION' ? row.id : undefined,
    sourceFamily: sourceFamilyForRow(row),
    label: row?.title,
    lockState: row?.preview?.toLowerCase().includes('locked') ? row.preview : undefined,
    conflictCount: conflictCount(row)
  };
}

function conflictCount(row?: ExplorerRow) {
  const match = row?.subtitle?.match(/(\d+)\s+conflict/i) ?? row?.preview?.match(/(\d+)\s+conflict/i);
  return match ? Number(match[1]) : undefined;
}
