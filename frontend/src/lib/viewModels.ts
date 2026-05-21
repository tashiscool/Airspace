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

export type WeatherGuidanceItem = {
  id: string;
  hazard: string;
  action: 'CLEAR' | 'MONITOR' | 'CAUTION' | 'DELAY' | 'ALTITUDE CHANGE' | 'AVOID' | 'REROUTE' | 'BLOCKED';
  priority: 'HIGH' | 'MEDIUM' | 'LOW';
  coordination: string;
  rationale: string;
  sourceFamily: string;
  sourceLabel: string;
  missionId?: string;
  reservationId?: string;
  createdAt?: string;
};

export type MissionWeatherVerdict = {
  action: WeatherGuidanceItem['action'];
  priority: WeatherGuidanceItem['priority'];
  confidence: number;
  count: number;
  summary: string;
  recommendedAction: string;
  sources: WeatherGuidanceItem[];
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

export function weatherGuidanceFromMessage(message: MessageSummary): WeatherGuidanceItem {
  const family = message.family.toUpperCase();
  const raw = `${message.subject ?? ''} ${message.rawText ?? ''}`.toUpperCase();
  if (family.includes('SIGMET') || family.includes('CWAP') || family.includes('CWAF') || raw.includes('EMBD TS') || raw.includes('CONV')) {
    return {
      id: message.id,
      missionId: message.missionId,
      reservationId: message.reservationId,
      createdAt: message.createdAt,
      hazard: family.includes('SIGMET') ? 'Convective SIGMET' : 'Convective weather',
      action: raw.includes('TOP FL') || raw.includes('INTSF') ? 'REROUTE' : 'AVOID',
      priority: 'HIGH',
      coordination: 'Weather desk + traffic manager review',
      sourceFamily: family,
      sourceLabel: message.subject || compactId(message.id),
      rationale: 'Embedded or intensifying convection can block route segments and reduce sector capacity.'
    };
  }
  if (family.includes('PIREP') || raw.includes('/TB') || raw.includes('/IC')) {
    const severe = raw.includes('SEV') || raw.includes('URGENT');
    return {
      id: message.id,
      missionId: message.missionId,
      reservationId: message.reservationId,
      createdAt: message.createdAt,
      hazard: raw.includes('/IC') ? 'Aircraft icing report' : 'Aircraft turbulence report',
      action: severe ? 'ALTITUDE CHANGE' : 'CAUTION',
      priority: severe ? 'HIGH' : 'MEDIUM',
      coordination: 'Solicit/verify PIREP and disseminate',
      sourceFamily: family,
      sourceLabel: message.subject || compactId(message.id),
      rationale: 'Aircraft reports close the gap between forecasts and observed hazards along the active route.'
    };
  }
  if (family.includes('METAR') || family.includes('TAF') || raw.includes('BKN00') || raw.includes('OVC00') || raw.includes(' 1/2SM')) {
    return {
      id: message.id,
      missionId: message.missionId,
      reservationId: message.reservationId,
      createdAt: message.createdAt,
      hazard: family.includes('TAF') ? 'Forecast ceiling/visibility' : 'Observed ceiling/visibility',
      action: raw.includes('1/2SM') || raw.includes('BKN004') ? 'DELAY' : 'MONITOR',
      priority: raw.includes('1/2SM') || raw.includes('BKN004') ? 'MEDIUM' : 'LOW',
      coordination: 'Terminal weather and route-release check',
      sourceFamily: family,
      sourceLabel: message.subject || compactId(message.id),
      rationale: 'Low ceiling or visibility affects departure/arrival decisions more than enroute lateral avoidance.'
    };
  }
  if (family.includes('AIRMET') || raw.includes('TURB') || raw.includes('ICE')) {
    return {
      id: message.id,
      missionId: message.missionId,
      reservationId: message.reservationId,
      createdAt: message.createdAt,
      hazard: 'AIRMET turbulence/icing',
      action: raw.includes('ICE') ? 'ALTITUDE CHANGE' : 'CAUTION',
      priority: 'MEDIUM',
      coordination: 'Monitor forecast confidence and pilot reports',
      sourceFamily: family,
      sourceLabel: message.subject || compactId(message.id),
      rationale: 'AIRMET hazards usually require altitude or route-risk management rather than an automatic block.'
    };
  }
  return {
    id: message.id,
    missionId: message.missionId,
    reservationId: message.reservationId,
    createdAt: message.createdAt,
    hazard: message.family,
    action: 'MONITOR',
    priority: 'LOW',
    coordination: 'Retain and classify',
    sourceFamily: family,
    sourceLabel: message.subject || compactId(message.id),
    rationale: 'Product is preserved for fusion, review, and later replay even when no immediate block is detected.'
  };
}

export function missionWeatherVerdict(missionId: string, messages: MessageSummary[] = []): MissionWeatherVerdict {
  const guidance = weatherRowsFromMessages(messages)
    .map(weatherGuidanceFromMessage)
    .filter((item) => item.missionId === missionId || !item.missionId);
  if (!guidance.length) {
    return {
      action: 'CLEAR',
      priority: 'LOW',
      confidence: 0.95,
      count: 0,
      summary: 'No active weather products linked to this mission.',
      recommendedAction: 'Continue normal monitoring.',
      sources: []
    };
  }
  const ranked = [...guidance].sort((a, b) => actionRank(b.action) - actionRank(a.action) || priorityRank(b.priority) - priorityRank(a.priority));
  const top = ranked[0];
  return {
    action: top.action,
    priority: top.priority,
    confidence: verdictConfidence(top, guidance),
    count: guidance.length,
    summary: `${guidance.length} weather/PIREP product(s); ${top.hazard}: ${top.rationale}`,
    recommendedAction: top.coordination,
    sources: ranked.slice(0, 5)
  };
}

export function weatherChangeFeed(messages: MessageSummary[] = [], limit = 8): WeatherGuidanceItem[] {
  return weatherRowsFromMessages(messages)
    .map(weatherGuidanceFromMessage)
    .sort((a, b) => String(b.createdAt ?? '').localeCompare(String(a.createdAt ?? '')) || priorityRank(b.priority) - priorityRank(a.priority))
    .slice(0, limit);
}

export function weatherFeaturesFromMessages(messages: MessageSummary[] = []): AirspaceFeature[] {
  return weatherRowsFromMessages(messages).flatMap((message) => {
    const raw = `${message.subject ?? ''}\n${message.rawText ?? ''}`;
    const coordinates = extractWeatherCoordinates(raw);
    if (!coordinates.length) return [];
    const guidance = weatherGuidanceFromMessage(message);
    const altitude = extractWeatherAltitudeBand(raw);
    const timing = extractWeatherTiming(raw);
    const movement = extractWeatherMovement(raw);
    const geometry = coordinates.length === 1
      ? { type: 'Point', coordinates: coordinates[0] }
      : coordinates.length === 2
        ? { type: 'LineString', coordinates }
        : { type: 'Polygon', coordinates: [closedRing(coordinates)] };
    return [{
      id: `weather-message-${message.id}`,
      type: 'Feature',
      geometry,
      properties: {
        featureKind: 'weather-product',
        sourceFamily: message.family.toUpperCase().includes('PIREP') ? 'PIREP' : 'WEATHER',
        sourceId: message.id,
        label: message.subject || guidance.hazard,
        hazardType: guidance.hazard,
        recommendedAction: guidance.action,
        rationale: guidance.rationale,
        confidence: guidance.priority === 'HIGH' ? 0.86 : guidance.priority === 'MEDIUM' ? 0.72 : 0.58,
        minAltitudeFeet: altitude.minAltitudeFeet,
        maxAltitudeFeet: altitude.maxAltitudeFeet,
        altitudeLabel: altitude.label,
        validStart: timing.validStart,
        validEnd: timing.validEnd,
        forecastHour: timing.forecastHour,
        validDurationH: timing.validDurationH,
        timingLabel: timing.label,
        movementDirection: movement.direction,
        movementSpeedKt: movement.speedKt,
        movementVector: movement.label,
        observedAt: message.createdAt,
        createdAt: message.createdAt,
        rawText: message.rawText
      }
    }];
  });
}

export function sourceRefFamily(ref: string) {
  const text = String(ref ?? '').toUpperCase();
  if (text.startsWith('FDC:') || text.startsWith('DOM:') || text.includes('NOTAM') || text.includes('SNOWTAM')
    || text.includes('BIRDTAM') || text.includes('ASHTAM') || text.includes('GENOT')) {
    return 'NOTAM';
  }
  if (text.startsWith('PIREP:') || text.includes('PIREP')) return 'PIREP';
  if (text.startsWith('SIGMET:') || text.startsWith('AIRMET:') || text.startsWith('METAR:') || text.startsWith('TAF:')
    || text.startsWith('WEATHER:') || text.includes('WX')) {
    return 'WEATHER';
  }
  if (text.startsWith('CARF:') || text.startsWith('ALTRV:') || text.startsWith('RESERVATION:')) return 'CARF/ALTRV';
  if (text.startsWith('USNS:') || text.startsWith('MESSAGE:')) return 'USNS';
  return 'SOURCE';
}

export function sourceRefLabel(ref: string) {
  const family = sourceRefFamily(ref);
  const value = String(ref ?? '').trim();
  const typed = /^[A-Z_/ -]+:(.+)$/i.exec(value);
  return {
    family,
    id: typed ? typed[1].trim() : value,
    label: `${family}: ${typed ? typed[1].trim() : value}`
  };
}

export function sourceRefRoute(ref: string) {
  const source = sourceRefLabel(ref);
  if (!source.id) return undefined;
  if (['NOTAM', 'PIREP', 'WEATHER', 'USNS'].includes(source.family)) return `/messages/${encodeURIComponent(source.id)}`;
  if (source.family === 'CARF/ALTRV') return `/deconfliction/${encodeURIComponent(source.id)}`;
  return undefined;
}

function actionRank(action: WeatherGuidanceItem['action']) {
  return ['CLEAR', 'MONITOR', 'CAUTION', 'DELAY', 'ALTITUDE CHANGE', 'AVOID', 'REROUTE', 'BLOCKED'].indexOf(action);
}

function priorityRank(priority: WeatherGuidanceItem['priority']) {
  return priority === 'HIGH' ? 3 : priority === 'MEDIUM' ? 2 : 1;
}

function verdictConfidence(top: WeatherGuidanceItem, guidance: WeatherGuidanceItem[]) {
  const base = top.priority === 'HIGH' ? 0.86 : top.priority === 'MEDIUM' ? 0.74 : 0.62;
  const corroboration = Math.min(0.08, Math.max(0, guidance.length - 1) * 0.02);
  const stalePenalty = top.createdAt && Date.now() - new Date(top.createdAt).getTime() > 90 * 60 * 1000 ? 0.08 : 0;
  return Math.max(0.35, Math.min(0.97, base + corroboration - stalePenalty));
}

function extractWeatherCoordinates(raw: string): number[][] {
  const coordinates: number[][] = [];
  const compact = /\b(\d{2})(\d{2})([NS])\s*(\d{3})(\d{2})([EW])\b/g;
  let match: RegExpExecArray | null;
  while ((match = compact.exec(raw)) !== null) {
    const lat = Number(match[1]) + Number(match[2]) / 60;
    const lon = Number(match[4]) + Number(match[5]) / 60;
    coordinates.push([match[6] === 'W' ? -lon : lon, match[3] === 'S' ? -lat : lat]);
  }
  const decimal = /\b([+-]?\d{1,2}\.\d+)\s*,\s*([+-]?\d{1,3}\.\d+)\b/g;
  while ((match = decimal.exec(raw)) !== null) {
    const lat = Number(match[1]);
    const lon = Number(match[2]);
    if (Math.abs(lat) <= 90 && Math.abs(lon) <= 180) coordinates.push([lon, lat]);
  }
  return dedupeCoordinateSequence(coordinates);
}

function dedupeCoordinateSequence(coordinates: number[][]) {
  const output: number[][] = [];
  for (const coordinate of coordinates) {
    const previous = output[output.length - 1];
    if (!previous || previous[0] !== coordinate[0] || previous[1] !== coordinate[1]) output.push(coordinate);
  }
  return output;
}

function closedRing(coordinates: number[][]) {
  const ring = [...coordinates];
  const first = ring[0];
  const last = ring[ring.length - 1];
  if (first && last && (first[0] !== last[0] || first[1] !== last[1])) ring.push(first);
  return ring;
}

function extractWeatherAltitudeBand(raw: string) {
  const text = raw.toUpperCase();
  const top = /\bTOPS?\s*(?:TO\s*)?FL\s?(\d{2,3})\b/.exec(text) ?? /\bTOP\s*FL\s?(\d{2,3})\b/.exec(text);
  const between = /\b(?:BTN|BETWEEN)\s*FL\s?(\d{2,3})\s*(?:AND|-|\/)\s*FL\s?(\d{2,3})\b/.exec(text)
    ?? /\bFL\s?(\d{2,3})\s*(?:-|\/)\s*FL\s?(\d{2,3})\b/.exec(text);
  const above = /\bABV\s*FL\s?(\d{2,3})\b/.exec(text);
  const below = /\bBLW\s*FL\s?(\d{2,3})\b/.exec(text);
  if (between) {
    const first = Number(between[1]) * 100;
    const second = Number(between[2]) * 100;
    return {
      minAltitudeFeet: Math.min(first, second),
      maxAltitudeFeet: Math.max(first, second),
      label: `FL${between[1]}-FL${between[2]}`
    };
  }
  if (top) {
    return {
      minAltitudeFeet: text.includes('SFC') ? 0 : undefined,
      maxAltitudeFeet: Number(top[1]) * 100,
      label: `${text.includes('SFC') ? 'SFC-' : 'TOP '}FL${top[1]}`
    };
  }
  if (above) {
    return {
      minAltitudeFeet: Number(above[1]) * 100,
      maxAltitudeFeet: undefined,
      label: `ABV FL${above[1]}`
    };
  }
  if (below) {
    return {
      minAltitudeFeet: undefined,
      maxAltitudeFeet: Number(below[1]) * 100,
      label: `BLW FL${below[1]}`
    };
  }
  if (text.includes('SFC')) {
    return {
      minAltitudeFeet: 0,
      maxAltitudeFeet: undefined,
      label: 'SFC'
    };
  }
  return {};
}

function extractWeatherTiming(raw: string) {
  const text = raw.toUpperCase();
  const valid = /\bVALID\s+(\d{2})(\d{2})(\d{2})\/(\d{2})(\d{2})(\d{2})\b/.exec(text)
    ?? /\bVALID\s+(\d{2})(\d{2})(\d{2})\s*-\s*(\d{2})(\d{2})(\d{2})\b/.exec(text);
  const forecast = /\b(?:T\+|FCST\s*HR\s*|FORECAST\s*HOUR\s*)(\d{1,2})\b/.exec(text);
  if (valid) {
    const start = `${valid[1]}${valid[2]}${valid[3]}Z`;
    const end = `${valid[4]}${valid[5]}${valid[6]}Z`;
    return {
      validStart: start,
      validEnd: end,
      forecastHour: forecast ? Number(forecast[1]) : undefined,
      validDurationH: durationHours(valid[2], valid[3], valid[5], valid[6]),
      label: `VALID ${start}-${end}`
    };
  }
  if (forecast) {
    return {
      forecastHour: Number(forecast[1]),
      validDurationH: 1,
      label: `T+${Number(forecast[1])}H`
    };
  }
  return {};
}

function extractWeatherMovement(raw: string) {
  const text = raw.toUpperCase();
  const moving = /\b(?:MOV|MOVG|MOVE|MOVING)\s+([A-Z]{1,3})\s+(\d{1,3})\s?KT\b/.exec(text)
    ?? /\b([A-Z]{1,3})\s+(\d{1,3})\s?KT\b/.exec(text);
  if (!moving) return {};
  return {
    direction: moving[1],
    speedKt: Number(moving[2]),
    label: `${moving[1]} ${moving[2]}KT`
  };
}

function durationHours(startHour: string, startMinute: string, endHour: string, endMinute: string) {
  const start = Number(startHour) * 60 + Number(startMinute);
  let end = Number(endHour) * 60 + Number(endMinute);
  if (end < start) end += 24 * 60;
  return Math.max(1, Math.round((end - start) / 60));
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
