import type { AirspaceFeature } from '../types';

export type MapLayerId =
  | 'reservations'
  | 'flight-paths'
  | 'weather'
  | 'wx-convective'
  | 'wx-turbulence'
  | 'wx-icing'
  | 'wx-wshear'
  | 'wx-ash'
  | 'route-impacts'
  | 'notams'
  | 'conflicts'
  | 'pireps'
  | 'navaids'
  | 'fixes'
  | 'aerodromes'
  | 'other';

export type MapLayerDefinition = {
  id: MapLayerId;
  label: string;
  group: 'OPS' | 'WEATHER' | 'REFERENCE';
  stroke: string;
  fill: string;
  width: number;
};

export const MAP_LAYERS: MapLayerDefinition[] = [
  { id: 'reservations', label: 'Reservations', group: 'OPS', stroke: '#35a7ff', fill: 'rgba(53, 167, 255, 0.18)', width: 2 },
  { id: 'flight-paths', label: 'Flight paths', group: 'OPS', stroke: '#3ed6e0', fill: 'rgba(62, 214, 224, 0.10)', width: 3 },
  { id: 'route-impacts', label: 'Route impacts', group: 'OPS', stroke: '#f97316', fill: 'rgba(249, 115, 22, 0.26)', width: 4 },
  { id: 'notams', label: 'NOTAM restrictions', group: 'OPS', stroke: '#f59e0b', fill: 'rgba(245, 158, 11, 0.18)', width: 2 },
  { id: 'conflicts', label: 'Conflicts', group: 'OPS', stroke: '#ef4444', fill: 'rgba(239, 68, 68, 0.28)', width: 4 },
  { id: 'weather', label: 'Weather', group: 'WEATHER', stroke: '#a78bfa', fill: 'rgba(167, 139, 250, 0.18)', width: 2 },
  { id: 'wx-convective', label: 'Convective', group: 'WEATHER', stroke: '#e11d48', fill: 'rgba(225, 29, 72, 0.22)', width: 2 },
  { id: 'wx-turbulence', label: 'Turbulence', group: 'WEATHER', stroke: '#eab308', fill: 'rgba(234, 179, 8, 0.16)', width: 2 },
  { id: 'wx-icing', label: 'Icing', group: 'WEATHER', stroke: '#22d3ee', fill: 'rgba(34, 211, 238, 0.18)', width: 2 },
  { id: 'wx-wshear', label: 'Wind shear', group: 'WEATHER', stroke: '#ec4899', fill: 'rgba(236, 72, 153, 0.18)', width: 2 },
  { id: 'wx-ash', label: 'Volcanic ash', group: 'WEATHER', stroke: '#a16207', fill: 'rgba(161, 98, 7, 0.22)', width: 2 },
  { id: 'pireps', label: 'PIREPs', group: 'WEATHER', stroke: '#d946ef', fill: 'rgba(217, 70, 239, 0.22)', width: 2 },
  { id: 'navaids', label: 'NAVAIDs', group: 'REFERENCE', stroke: '#94a3b8', fill: 'rgba(148, 163, 184, 0.38)', width: 1 },
  { id: 'fixes', label: 'Fixes', group: 'REFERENCE', stroke: '#64748b', fill: 'rgba(100, 116, 139, 0.38)', width: 1 },
  { id: 'aerodromes', label: 'Aerodromes', group: 'REFERENCE', stroke: '#a3e635', fill: 'rgba(163, 230, 53, 0.28)', width: 1 },
  { id: 'other', label: 'Other', group: 'OPS', stroke: '#94a3b8', fill: 'rgba(148, 163, 184, 0.14)', width: 2 }
];

export function featureDisplayLayer(featureOrProperties?: AirspaceFeature | Record<string, unknown>): MapLayerId {
  const properties = (featureOrProperties && 'properties' in featureOrProperties
    ? featureOrProperties.properties
    : featureOrProperties) as Record<string, unknown> | undefined;
  const explicit = String(properties?.displayLayer ?? '').toLowerCase();
  if (isMapLayerId(explicit)) return explicit;

  const kind = String(properties?.featureKind ?? '').toLowerCase();
  const family = String(properties?.sourceFamily ?? '').toUpperCase();
  const constraint = String(properties?.constraintType ?? '').toUpperCase();
  const hazard = String(properties?.hazardType ?? properties?.hazard ?? properties?.weatherType ?? '').toUpperCase();
  const pointType = String(properties?.pointType ?? '').toUpperCase();

  if (kind === 'flight-path' || constraint === 'CARF_ROUTE') return 'flight-paths';
  if (kind === 'reservation' || constraint === 'CARF_RESERVATION') return 'reservations';
  if (kind === 'conflict' || constraint === 'CARF_CONFLICT') return 'conflicts';
  if (kind.includes('intersection') || kind.includes('avoidance') || constraint === 'ROUTE_BLOCKAGE' || constraint === 'ROUTE_AVOIDANCE') return 'route-impacts';
  if (pointType === 'NAVAID') return 'navaids';
  if (pointType === 'AERODROME' || pointType === 'AIRPORT') return 'aerodromes';
  if (kind === 'reference-point' || pointType === 'FIX' || family === 'REFERENCE') return 'fixes';
  if (family === 'PIREP' || constraint === 'PIREP_HAZARD') return 'pireps';
  if (hazard.includes('CONV') || hazard.includes('THUNDER') || hazard.includes('CWA')) return 'wx-convective';
  if (hazard.includes('TURB')) return 'wx-turbulence';
  if (hazard.includes('ICE') || hazard.includes('ICING')) return 'wx-icing';
  if (hazard.includes('SHEAR') || hazard.includes('WSHEAR')) return 'wx-wshear';
  if (hazard.includes('ASH') || hazard.includes('VOLCANIC')) return 'wx-ash';
  if (kind.includes('weather') || family === 'WEATHER' || constraint === 'WEATHER_HAZARD') return 'weather';
  if (kind === 'notam' || family === 'NOTAM' || constraint === 'NOTAM_RESTRICTION') return 'notams';
  return 'other';
}

export function layerDefinition(id: MapLayerId): MapLayerDefinition {
  return MAP_LAYERS.find((layer) => layer.id === id) ?? MAP_LAYERS[MAP_LAYERS.length - 1];
}

export function layersForWorkbenchGroup(group: 'all' | 'ops' | 'weather' | 'reference'): MapLayerId[] {
  if (group === 'all') return MAP_LAYERS.map((layer) => layer.id);
  if (group === 'ops') return ['reservations', 'flight-paths', 'conflicts', 'route-impacts', 'notams'];
  if (group === 'weather') return ['weather', 'wx-convective', 'wx-turbulence', 'wx-icing', 'wx-wshear', 'wx-ash', 'pireps', 'route-impacts'];
  return ['navaids', 'fixes', 'aerodromes'];
}

export type MapFeatureSummary = {
  title: string;
  subtitle: string;
  source: string;
  risk?: string;
  geometry?: string;
  timing?: string;
  altitude?: string;
  confidence?: string;
  action?: string;
  freshness?: string;
  movement?: string;
  cost?: string;
};

export type MapFeatureSourceLink = {
  label: string;
  route: string;
};

export type MapFeatureSourceRef = {
  family: string;
  id: string;
  route?: string;
};

export type MapFeatureFreshness = {
  observedAt?: string;
  ageMinutes?: number;
  stale: boolean;
  category: 'current' | 'aging' | 'stale' | 'unknown';
  label: string;
  opacity: number;
};

export type MapFeatureConfidence = {
  value?: number;
  category: 'high' | 'medium' | 'low' | 'unknown';
  label: string;
  opacity: number;
};

export type MapFeatureSeverity = {
  category: 'extreme' | 'severe' | 'moderate' | 'low' | 'unknown';
  label: string;
  widthBoost: number;
};

export type MapVisibleRiskCounts = {
  blocked: number;
  severe: number;
  lowConfidence: number;
  stale: number;
};

export type AffectedMapContext = {
  featureIds?: string[];
  missionIds?: string[];
  sourceRefs?: string[];
};

export function mapFeatureSummary(feature?: AirspaceFeature): MapFeatureSummary | undefined {
  if (!feature) return undefined;
  const properties = feature.properties ?? {};
  const layer = layerDefinition(featureDisplayLayer(feature));
  const validStart = stringProp(properties, 'validStart') ?? stringProp(properties, 'startTime') ?? stringProp(properties, 'effectiveStart');
  const validEnd = stringProp(properties, 'validEnd') ?? stringProp(properties, 'endTime') ?? stringProp(properties, 'effectiveEnd');
  const minAlt = stringProp(properties, 'minAltitudeFeet') ?? stringProp(properties, 'minimumAltitudeFeet') ?? stringProp(properties, 'floor');
  const maxAlt = stringProp(properties, 'maxAltitudeFeet') ?? stringProp(properties, 'maximumAltitudeFeet') ?? stringProp(properties, 'ceiling');
  const confidence = mapFeatureConfidence(feature);
  const movement = stringProp(properties, 'movementVector') ?? movementLabel(properties);
  const severity = mapFeatureSeverity(feature);
  const freshness = mapFeatureFreshness(feature);
  const cost = routeCostLabel(properties);
  return {
    title: stringProp(properties, 'label') ?? stringProp(properties, 'name') ?? stringProp(properties, 'featureKind') ?? String(feature.id ?? layer.label),
    subtitle: stringProp(properties, 'rationale') ?? stringProp(properties, 'explanation') ?? stringProp(properties, 'hazardType') ?? layer.label,
    source: stringProp(properties, 'sourceFamily') ?? stringProp(properties, 'sourceProduct') ?? layer.label,
    risk: mapFeatureRiskLabel(feature),
    geometry: geometryIntentLabel(properties, feature),
    timing: validStart || validEnd ? `${validStart ?? 'start ?'} - ${validEnd ?? 'end ?'}` : undefined,
    altitude: minAlt || maxAlt ? `${minAlt ?? 'floor ?'} - ${maxAlt ?? 'ceiling ?'}` : undefined,
    confidence: confidence.label,
    action: stringProp(properties, 'recommendedAction') ?? stringProp(properties, 'action'),
    freshness: freshness?.label,
    movement: movement ? `${movement} · ${severity.label}` : severity.label,
    cost
  };
}

export function mapFeatureRiskLabel(feature?: AirspaceFeature) {
  const severity = mapFeatureSeverity(feature);
  const confidence = mapFeatureConfidence(feature);
  const freshness = mapFeatureFreshness(feature);
  const parts = [
    severity.label,
    confidence.label,
    freshness?.label
  ].filter(Boolean);
  return parts.length ? parts.join(' · ') : undefined;
}

export function mapFeatureSeverity(feature?: AirspaceFeature): MapFeatureSeverity {
  const properties = feature?.properties ?? {};
  const text = [
    stringProp(properties, 'severity'),
    stringProp(properties, 'intensity'),
    stringProp(properties, 'recommendedAction'),
    stringProp(properties, 'action'),
    stringProp(properties, 'hazardType'),
    stringProp(properties, 'rationale')
  ].join(' ').toUpperCase();
  if (/\b(EXTREME|EXTRM|BLOCKED|EMERGENCY)\b/.test(text)) {
    return { category: 'extreme', label: 'extreme severity', widthBoost: 2.2 };
  }
  if (/\b(SEV|SEVERE|REROUTE|AVOID|INTSF|URGENT)\b/.test(text)) {
    return { category: 'severe', label: 'severe', widthBoost: 1.5 };
  }
  if (/\b(MOD|MODERATE|CAUTION|DELAY|ALTITUDE CHANGE)\b/.test(text)) {
    return { category: 'moderate', label: 'moderate', widthBoost: 0.8 };
  }
  if (/\b(LOW|CLEAR|MONITOR)\b/.test(text)) {
    return { category: 'low', label: 'low severity', widthBoost: 0 };
  }
  return { category: 'unknown', label: 'severity unknown', widthBoost: 0 };
}

export function mapFeatureConfidence(feature?: AirspaceFeature): MapFeatureConfidence {
  const properties = feature?.properties ?? {};
  const raw = numericProp(properties, 'confidence') ?? numericProp(properties, 'probability') ?? numericProp(properties, 'blockedProbability');
  if (raw == null) {
    return { category: 'unknown', label: 'confidence unknown', opacity: 0.82 };
  }
  const value = raw > 1 ? raw / 100 : raw;
  const category = value >= 0.75 ? 'high' : value >= 0.45 ? 'medium' : 'low';
  return {
    value,
    category,
    label: `${Math.round(value * 100)}% confidence`,
    opacity: category === 'high' ? 1 : category === 'medium' ? 0.74 : 0.46
  };
}

export function mapFeatureSourceLink(feature?: AirspaceFeature): MapFeatureSourceLink | undefined {
  const properties = feature?.properties ?? {};
  const sourceId = stringProp(properties, 'sourceId') ?? stringProp(properties, 'messageId') ?? stringProp(properties, 'productId')
    ?? stringProp(properties, 'pirepId') ?? stringProp(properties, 'notamId') ?? stringProp(properties, 'reservationId');
  if (!sourceId) return undefined;
  const layer = featureDisplayLayer(feature);
  const family = String(properties.sourceFamily ?? properties.family ?? layer).toUpperCase();
  if (layer === 'weather' || layer.startsWith('wx-') || layer === 'pireps' || layer === 'notams' || family.includes('WEATHER') || family.includes('PIREP') || family.includes('NOTAM')) {
    return { label: `Open ${family || 'source'} ${sourceId}`, route: `/messages/${encodeURIComponent(sourceId)}` };
  }
  if (layer === 'reservations' || layer === 'flight-paths' || layer === 'conflicts' || family.includes('CARF') || family.includes('ALTRV')) {
    return { label: `Open reservation ${sourceId}`, route: `/deconfliction/${encodeURIComponent(sourceId)}` };
  }
  return undefined;
}

export function mapFeatureSourceRefs(feature?: AirspaceFeature): MapFeatureSourceRef[] {
  const properties = feature?.properties ?? {};
  const values = [
    properties.sourceRefs,
    properties.sourceRef,
    properties.sourceIds,
    properties.blockingSourceRefs
  ].flatMap((value) => sourceRefValues(value));
  const direct = mapFeatureSourceLink(feature);
  if (direct) values.unshift(direct.route.replace(/^\/messages\//, 'MESSAGE:').replace(/^\/deconfliction\//, 'RESERVATION:'));
  const seen = new Set<string>();
  return values.flatMap((value) => {
    const ref = parseSourceRef(value);
    const key = `${ref.family}:${ref.id}`;
    if (!ref.id || seen.has(key)) return [];
    seen.add(key);
    return [ref];
  }).slice(0, 12);
}

export function mapFeatureFreshness(feature?: AirspaceFeature, now = new Date()): MapFeatureFreshness | undefined {
  const properties = feature?.properties ?? {};
  const layer = featureDisplayLayer(feature);
  const weatherRelevant = layer === 'pireps' || layer === 'weather' || layer.startsWith('wx-') || layer === 'route-impacts';
  if (!weatherRelevant) return undefined;

  const explicitAgeSeconds = numericProp(properties, 'ageSeconds') ?? numericProp(properties, 'guidanceLatencySeconds');
  const observedAt = stringProp(properties, 'observedAt')
    ?? stringProp(properties, 'receivedAt')
    ?? stringProp(properties, 'createdAt')
    ?? stringProp(properties, 'validStart')
    ?? stringProp(properties, 'startTime');
  const parsedAt = observedAt ? Date.parse(observedAt) : Number.NaN;
  const ageMinutes = explicitAgeSeconds != null
    ? Math.max(0, explicitAgeSeconds / 60)
    : Number.isFinite(parsedAt)
      ? Math.max(0, (now.getTime() - parsedAt) / 60000)
      : undefined;
  const explicitStale = booleanProp(properties, 'stale');
  const stale = explicitStale || (ageMinutes != null && ageMinutes > 90);
  const category = stale ? 'stale' : ageMinutes == null ? 'unknown' : ageMinutes > 60 ? 'aging' : 'current';
  return {
    observedAt,
    ageMinutes,
    stale,
    category,
    label: category === 'unknown' ? 'freshness unknown' : `${Math.round(ageMinutes ?? 0)}m old${stale ? ' · stale' : category === 'aging' ? ' · aging' : ''}`,
    opacity: category === 'stale' ? 0.34 : category === 'aging' ? 0.58 : 1
  };
}

export function featurePassesFreshnessFilter(
  feature: AirspaceFeature,
  options: { maxAgeMinutes?: number; hideStale?: boolean; now?: Date } = {}
) {
  const freshness = mapFeatureFreshness(feature, options.now ?? new Date());
  if (!freshness) return true;
  if (options.hideStale && freshness.stale) return false;
  if (options.maxAgeMinutes != null && freshness.ageMinutes != null && freshness.ageMinutes > options.maxAgeMinutes) {
    return false;
  }
  return true;
}

export function mapVisibleRiskCounts(features: AirspaceFeature[] = []): MapVisibleRiskCounts {
  return features.reduce<MapVisibleRiskCounts>((counts, feature) => {
    const severity = mapFeatureSeverity(feature);
    const freshness = mapFeatureFreshness(feature);
    const confidence = mapFeatureConfidence(feature);
    if (isBlockingFeature(feature)) counts.blocked += 1;
    if (severity.category === 'severe' || severity.category === 'extreme') counts.severe += 1;
    if (confidence.category === 'low') counts.lowConfidence += 1;
    if (freshness?.stale) counts.stale += 1;
    return counts;
  }, { blocked: 0, severe: 0, lowConfidence: 0, stale: 0 });
}

export function featureMatchesAffectedContext(feature: AirspaceFeature, context: AffectedMapContext = {}, index = 0) {
  const featureIds = new Set(context.featureIds ?? []);
  const missionIds = new Set((context.missionIds ?? []).map(String));
  const sourceRefs = new Set(context.sourceRefs ?? []);
  if (!featureIds.size && !missionIds.size && !sourceRefs.size) return false;
  const id = String(feature.id ?? index);
  const properties = feature.properties ?? {};
  if (featureIds.has(id)) return true;
  const missionId = stringProp(properties, 'missionId') ?? stringProp(properties, 'affectedMissionId');
  if (missionId && missionIds.has(missionId)) return true;
  return mapFeatureSourceRefs(feature).some((ref) => sourceRefs.has(`${ref.family}:${ref.id}`) || sourceRefs.has(ref.id));
}

function isMapLayerId(value: string): value is MapLayerId {
  return MAP_LAYERS.some((layer) => layer.id === value);
}

function stringProp(properties: Record<string, unknown>, key: string) {
  const value = properties[key];
  if (value == null) return undefined;
  return String(value);
}

function numericProp(properties: Record<string, unknown>, key: string) {
  const value = properties[key];
  if (typeof value === 'number') return value;
  if (typeof value === 'string' && value.trim() !== '' && Number.isFinite(Number(value))) return Number(value);
  return undefined;
}

function routeCostLabel(properties: Record<string, unknown>) {
  const distance = numericProp(properties, 'additionalDistanceNm');
  const delay = numericProp(properties, 'additionalMinutes');
  const fuel = numericProp(properties, 'additionalFuelLb');
  const cost = numericProp(properties, 'additionalCostUsd');
  if (distance == null && delay == null && fuel == null && cost == null) return undefined;
  return [
    distance == null ? undefined : `+${formatNumber(distance)} NM`,
    delay == null ? undefined : `+${formatNumber(delay)} min`,
    fuel == null ? undefined : `+${formatNumber(fuel)} lb fuel`,
    cost == null ? undefined : `+$${formatNumber(cost)}`
  ].filter(Boolean).join(' · ');
}

function formatNumber(value: number) {
  return value >= 100 ? Math.round(value).toLocaleString() : value.toFixed(1);
}

function booleanProp(properties: Record<string, unknown>, key: string) {
  const value = properties[key];
  if (typeof value === 'boolean') return value;
  if (typeof value === 'string') return value.toLowerCase() === 'true';
  return false;
}

function isBlockingFeature(feature: AirspaceFeature) {
  const properties = feature.properties ?? {};
  const text = [
    stringProp(properties, 'recommendedAction'),
    stringProp(properties, 'action'),
    stringProp(properties, 'constraintType'),
    stringProp(properties, 'featureKind'),
    stringProp(properties, 'rationale')
  ].join(' ').toUpperCase();
  return /\b(BLOCKED|REROUTE|AVOID|ROUTE_BLOCKAGE|ROUTE_AVOIDANCE)\b/.test(text);
}

function sourceRefValues(value: unknown): string[] {
  if (typeof value === 'string') {
    return value.split(/[,\n;]/).map((item) => item.trim()).filter(Boolean);
  }
  if (Array.isArray(value)) return value.flatMap(sourceRefValues);
  return [];
}

function parseSourceRef(value: string): MapFeatureSourceRef {
  const typed = /^([A-Z_ -]+):(.+)$/i.exec(value.trim());
  const rawFamily = typed ? typed[1].toUpperCase().replace(/\s+/g, '_') : '';
  const id = (typed ? typed[2] : value).trim();
  const text = `${rawFamily} ${id}`.toUpperCase();
  const family = text.includes('PIREP') ? 'PIREP'
    : text.includes('NOTAM') || text.startsWith('FDC ') || text.startsWith('DOM ') ? 'NOTAM'
      : text.includes('SIGMET') || text.includes('AIRMET') || text.includes('WEATHER') || text.includes('METAR') || text.includes('TAF') || text.includes('WX') ? 'WEATHER'
        : text.includes('RESERVATION') || text.includes('CARF') || text.includes('ALTRV') ? 'CARF/ALTRV'
          : text.includes('MESSAGE') || text.includes('USNS') ? 'USNS'
            : rawFamily || 'SOURCE';
  const route = ['PIREP', 'NOTAM', 'WEATHER', 'USNS'].includes(family)
    ? `/messages/${encodeURIComponent(id)}`
    : family === 'CARF/ALTRV'
      ? `/deconfliction/${encodeURIComponent(id)}`
      : undefined;
  return { family, id, route };
}

function movementLabel(properties: Record<string, unknown>) {
  const direction = stringProp(properties, 'movementDirection');
  const speed = stringProp(properties, 'movementSpeedKt') ?? stringProp(properties, 'speedKt');
  if (!direction && !speed) return undefined;
  return `${direction ?? 'MOV'} ${speed ? `${speed}KT` : ''}`.trim();
}

function geometryIntentLabel(properties: Record<string, unknown>, feature?: AirspaceFeature) {
  const explicit = stringProp(properties, 'geometryIntent');
  const label = stringProp(properties, 'geometryLabel');
  const radius = stringProp(properties, 'radiusNauticalMiles');
  const corridor = stringProp(properties, 'corridorWidthNauticalMiles');
  if (explicit === 'POINT_RADIUS') return `${label ?? 'POINT RADIUS'} · radius ${radius ?? '?'}NM`;
  if (explicit === 'LINE_CORRIDOR') return `${label ?? 'LINE CORRIDOR'}${corridor ? ` · corridor ${corridor}NM` : ''}`;
  if (explicit === 'POLYGON') return label ?? 'POLYGON';
  if (explicit === 'POINT') return label ?? 'POINT';
  if (explicit && radius) return `${explicit} · radius ${radius}NM`;
  if (explicit && corridor) return `${explicit} · corridor ${corridor}NM`;
  if (explicit) return explicit;
  if (radius) return `radius ${radius}NM`;
  if (corridor) return `corridor ${corridor}NM`;
  const type = String(feature?.geometry?.type ?? '').toUpperCase();
  return type || undefined;
}
