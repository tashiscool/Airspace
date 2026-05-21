import type {
  AirspaceFeature,
  DecisionSummary,
  FeedArtifactSummary,
  HistoryEventSummary,
  MessageSummary,
  MissionDetail,
  MissionSummary,
  ReferencePointSummary,
  ReservationSummary,
  ReservationSupplementSummary,
  SearchResultSummary
} from '../types';

export const MESSAGE_FAMILIES = [
  'DOM',
  'FDC',
  'ICAO_NOTAMN',
  'ICAO_NOTAMR',
  'ICAO_NOTAMC',
  'ICAO_NOTAMJ',
  'CANADIAN_DOMESTIC',
  'SNOWTAM',
  'BIRDTAM',
  'ASHTAM',
  'GENOT',
  'SVC_RQ',
  'SVC_TBL',
  'FDC_ACK',
  'PIREP',
  'SIGMET',
  'AIRMET',
  'METAR',
  'TAF',
  'WEATHER_ADVISORY',
  'USNS',
  'UNKNOWN'
];

export type SectionKey = 'A' | 'B' | 'C' | 'D' | 'E' | 'F' | 'G';

export type AltrvSections = Record<SectionKey, string>;

export type SectionValidationDiagnostic = {
  section: SectionKey;
  severity: 'ERROR' | 'WARN';
  message: string;
};

export type ExplorerRow = {
  key: string;
  family: 'MISSION' | 'RESERVATION' | 'MESSAGE' | 'NOTAM' | 'APREQ' | 'APPROVAL' | 'FEED' | 'DECISION';
  id: string;
  title: string;
  subtitle: string;
  status?: string;
  time?: string;
  missionId?: string;
  reservationId?: string;
  route?: string;
  preview: string;
};

export type NotamLikeRow = {
  id: string;
  source: 'MESSAGE' | 'SUPPLEMENT' | 'SEARCH';
  family: string;
  status?: string;
  facility?: string;
  title: string;
  text: string;
  route?: string;
  updatedAt?: string;
};

export function fmtZ(value?: string) {
  if (!value) return '—';
  const date = new Date(value);
  if (Number.isNaN(date.getTime())) return value;
  return date.toISOString().replace('.000Z', 'Z');
}

export function compactId(value?: string) {
  if (!value) return '—';
  if (value.length <= 16) return value;
  return `${value.slice(0, 8)}…${value.slice(-5)}`;
}

export function parseJson<T = unknown>(value?: string): T | undefined {
  if (!value) return undefined;
  try {
    return JSON.parse(value) as T;
  } catch {
    return undefined;
  }
}

export function extractAltrvSections(rawText?: string): AltrvSections {
  const empty: AltrvSections = { A: '', B: '', C: '', D: '', E: '', F: '', G: '' };
  if (!rawText) return empty;
  const normalized = rawText.replace(/\r\n/g, '\n');
  const matches = [...normalized.matchAll(/(?:^|\n)\s*([A-G])\.\s*/g)];
  if (!matches.length) {
    empty.D = normalized.trim();
    return empty;
  }
  for (let index = 0; index < matches.length; index += 1) {
    const key = matches[index][1] as SectionKey;
    const start = (matches[index].index ?? 0) + matches[index][0].length;
    const end = index + 1 < matches.length ? matches[index + 1].index ?? normalized.length : normalized.length;
    empty[key] = normalized.slice(start, end).trim();
  }
  return empty;
}

export function composeAltrvSections(sections: AltrvSections) {
  return (Object.keys(sections) as SectionKey[])
    .map((key) => `${key}. ${sections[key].trim()}`)
    .join('\n');
}

export function validateAltrvSections(sections: AltrvSections): SectionValidationDiagnostic[] {
  const diagnostics: SectionValidationDiagnostic[] = [];
  const required: SectionKey[] = ['A', 'B', 'C', 'D', 'E', 'F'];
  for (const key of required) {
    if (!sections[key].trim()) {
      diagnostics.push({
        section: key,
        severity: 'ERROR',
        message: `Section ${key} is required for a usable CARF/ALTRV reservation.`
      });
    }
  }
  if (sections.A.trim() && !/[A-Z0-9]{2,7}/i.test(sections.A)) {
    diagnostics.push({
      section: 'A',
      severity: 'WARN',
      message: 'Section A should include at least one callsign or accountability token.'
    });
  }
  if (sections.B.trim() && !/\d/.test(sections.B)) {
    diagnostics.push({
      section: 'B',
      severity: 'WARN',
      message: 'Section B should include aircraft count or type information.'
    });
  }
  if (
    sections.D.trim() &&
    !/(FL|SFC|ABV|BLW|\d{4}[NS]\s*\d{5}[EW]|\d{2,3}\.\d+|RTE|ROUTE|AREA|BNDD|WITHIN)/i.test(sections.D)
  ) {
    diagnostics.push({
      section: 'D',
      severity: 'WARN',
      message: 'Section D should include route, area, fix, coordinate, or altitude information.'
    });
  }
  if (sections.F.trim() && !/(ETD|AVANA|WEF|TIL|\d{6})/i.test(sections.F)) {
    diagnostics.push({
      section: 'F',
      severity: 'WARN',
      message: 'Section F should include timing such as ETD, AVANA, WEF/TIL, or DDHHMM values.'
    });
  }
  if (sections.G.length > 1200) {
    diagnostics.push({
      section: 'G',
      severity: 'WARN',
      message: 'Section G comments are unusually long; verify the retained raw text before sending.'
    });
  }
  return diagnostics;
}

export function rowsFromMissionDetail(detail?: MissionDetail): ExplorerRow[] {
  if (!detail) return [];
  const rows: ExplorerRow[] = [];
  rows.push({
    key: `mission:${detail.mission.id}`,
    family: 'MISSION',
    id: detail.mission.id,
    title: detail.mission.missionNumber,
    subtitle: detail.mission.title ?? 'Mission workspace',
    status: detail.mission.status,
    missionId: detail.mission.id,
    route: `/missions/${detail.mission.id}`,
    time: detail.mission.updatedAt,
    preview: `${detail.reservations.length} reservation(s), ${detail.messages.length} message(s)`
  });
  for (const reservation of detail.reservations) {
    rows.push(rowFromReservation(reservation, detail.mission));
  }
  for (const message of detail.messages) {
    rows.push(rowFromMessage(message));
  }
  return rows;
}

export function rowFromReservation(reservation: ReservationSummary, mission?: MissionSummary): ExplorerRow {
  return {
    key: `reservation:${reservation.id}`,
    family: 'RESERVATION',
    id: reservation.id,
    title: `Reservation ${compactId(reservation.id)}`,
    subtitle: `${mission?.missionNumber ?? reservation.missionId} · ${reservation.conflictCount} conflict(s)`,
    status: reservation.state,
    missionId: reservation.missionId,
    reservationId: reservation.id,
    route: `/missions/${reservation.missionId}/reservations/${reservation.id}`,
    preview: reservation.diagnostics?.[0] ?? reservation.rawText?.slice(0, 180) ?? 'No retained reservation text'
  };
}

export function rowFromMessage(message: MessageSummary): ExplorerRow {
  return {
    key: `message:${message.id}`,
    family: isNotamFamily(message.family) ? 'NOTAM' : 'MESSAGE',
    id: message.id,
    title: message.subject || `${message.family} message`,
    subtitle: `${message.direction} · ${message.family}`,
    status: message.status,
    missionId: message.missionId,
    reservationId: message.reservationId,
    route: `/messages/${message.id}`,
    time: message.createdAt,
    preview: message.rawText?.slice(0, 220) ?? 'No retained message text'
  };
}

export function rowFromFeed(artifact: FeedArtifactSummary): ExplorerRow {
  return {
    key: `feed:${artifact.id}`,
    family: 'FEED',
    id: artifact.id,
    title: `${artifact.type} · ${artifact.sourceId}`,
    subtitle: artifact.rawPayloadHash,
    status: artifact.accepted ? 'ACCEPTED' : 'REJECTED',
    route: `/feed/${artifact.id}`,
    time: artifact.receivedAt,
    preview: artifact.diagnostics.join('; ') || artifact.rawPayload?.slice(0, 180) || 'No diagnostics'
  };
}

export function rowFromDecision(decision: DecisionSummary): ExplorerRow {
  return {
    key: `decision:${decision.id}`,
    family: 'DECISION',
    id: decision.id,
    title: decision.recommendedAction || decision.action,
    subtitle: `confidence ${Math.round(decision.confidence * 100)}%`,
    status: decision.action,
    route: `/decisions/${decision.id}`,
    preview: decision.rationale ?? 'No decision rationale retained'
  };
}

export function notamRowsFromSources(
  messages: MessageSummary[] = [],
  supplements: ReservationSupplementSummary[] = [],
  search: SearchResultSummary[] = []
): NotamLikeRow[] {
  const fromMessages = messages
    .filter((message) => isNotamFamily(message.family) || hasNotamText(message.rawText) || hasNotamText(message.subject))
    .map((message): NotamLikeRow => ({
      id: message.id,
      source: 'MESSAGE',
      family: message.family,
      status: message.status,
      title: message.subject || `${message.family} NOTAM traffic`,
      text: message.rawText || 'No retained NOTAM body.',
      route: `/messages/${message.id}`,
      updatedAt: message.createdAt
    }));
  const fromSupplements = supplements
    .filter((supplement) => supplement.kind.toUpperCase().includes('NOTAM') || hasNotamText(supplement.text))
    .map((supplement): NotamLikeRow => ({
      id: supplement.id,
      source: 'SUPPLEMENT',
      family: supplement.kind,
      status: supplement.status,
      title: supplement.title || 'Reservation NOTAM supplement',
      text: supplement.text || 'No supplement text retained.',
      route: `/missions/${supplement.reservationId}/reservations/${supplement.reservationId}`,
      updatedAt: supplement.updatedAt
    }));
  const fromSearch = search
    .filter((item) => item.type.toUpperCase().includes('NOTAM') || hasNotamText(item.title) || hasNotamText(item.snippet))
    .map((item): NotamLikeRow => ({
      id: item.id,
      source: 'SEARCH',
      family: item.type,
      status: item.status,
      title: item.title,
      text: item.snippet || 'Search result did not include retained NOTAM text.',
      route: item.route,
      updatedAt: item.updatedAt
    }));
  return [...fromMessages, ...fromSupplements, ...fromSearch];
}

export function isNotamFamily(family?: string) {
  const value = String(family ?? '').toUpperCase();
  return value.includes('NOTAM') || ['DOM', 'FDC', 'SNOWTAM', 'BIRDTAM', 'ASHTAM', 'GENOT', 'CANADIAN_DOMESTIC'].includes(value);
}

export function weatherRowsFromMessages(messages: MessageSummary[] = []) {
  return messages.filter((message) =>
    ['PIREP', 'SIGMET', 'AIRMET', 'METAR', 'TAF', 'WEATHER_ADVISORY', 'WX_ADVISORY'].includes(String(message.family).toUpperCase())
  );
}

export function featureLabel(feature: AirspaceFeature) {
  const props = feature.properties ?? {};
  return String(props.label ?? props.name ?? props.id ?? feature.id ?? props.featureKind ?? 'feature');
}

export function referencePointToFeature(point: ReferencePointSummary): AirspaceFeature {
  return {
    id: point.id,
    type: 'Feature',
    geometry: { type: 'Point', coordinates: [point.longitude, point.latitude, point.altitudeFeet ?? 0] },
    properties: {
      featureKind: 'reference-point',
      sourceFamily: 'REFERENCE',
      pointType: point.pointType,
      label: point.identifier,
      source: point.source
    }
  };
}

function hasNotamText(value?: string) {
  return /\b(NOTAM|!FDC|!DOM|SNOWTAM|BIRDTAM|ASHTAM|GENOT)\b/i.test(value ?? '');
}
