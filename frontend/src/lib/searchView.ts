import type { SearchResultSummary } from '../types';
import type { WorkbenchSelection } from './workbenchState';

export function searchTypeLabel(type?: string): string {
  const normalized = String(type ?? '').toLowerCase();
  if (normalized === 'feed-transaction') return 'Feed transaction';
  if (!normalized) return 'Unknown';
  return normalized
    .split(/[-_\s]+/)
    .filter(Boolean)
    .map((part) => part.charAt(0).toUpperCase() + part.slice(1))
    .join(' ');
}

export function selectionForSearch(row: SearchResultSummary): WorkbenchSelection {
  const type = String(row.type ?? '').toUpperCase();
  const text = `${row.title ?? ''} ${row.snippet ?? ''}`.toUpperCase();
  const isFeed = type.includes('FEED');
  const isFeedTransaction = type === 'FEED-TRANSACTION';
  return {
    missionId: type.includes('MISSION') ? row.id : undefined,
    messageId: type.includes('MESSAGE') || (type.includes('NOTAM') && !isFeed) ? row.id : undefined,
    feedArtifactId: isFeed ? feedArtifactId(row.id) : undefined,
    decisionId: type.includes('DECISION') ? row.id : undefined,
    sourceFamily: sourceFamilyForSearch(type, text),
    label: isFeedTransaction ? `Feed transaction: ${row.title}` : row.title,
    lockState: row.status
  };
}

function sourceFamilyForSearch(type: string, text: string): WorkbenchSelection['sourceFamily'] {
  if (type.includes('NOTAM') || text.includes('NOTAM') || text.includes('DOM2.') || text.includes(' Q')) return 'NOTAM';
  if (type.includes('DECISION')) return 'DECISION';
  if (text.includes('PIREP')) return 'PIREP';
  if (type.includes('WEATHER') || text.includes('SIGMET') || text.includes('AIRMET') || text.includes('METAR') || text.includes('TAF')) return 'WEATHER';
  if (type.includes('CARF') || type.includes('RESERVATION') || type.includes('MISSION')) return 'CARF_ALTRV';
  if (type.includes('FEED')) return 'USNS';
  return 'UNKNOWN';
}

function feedArtifactId(id?: string): string | undefined {
  if (!id) return undefined;
  return id.split('#')[0];
}
