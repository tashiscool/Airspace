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
  if (kind.includes('intersection') || constraint === 'ROUTE_BLOCKAGE') return 'route-impacts';
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
  timing?: string;
  altitude?: string;
  confidence?: string;
  action?: string;
};

export function mapFeatureSummary(feature?: AirspaceFeature): MapFeatureSummary | undefined {
  if (!feature) return undefined;
  const properties = feature.properties ?? {};
  const layer = layerDefinition(featureDisplayLayer(feature));
  const validStart = stringProp(properties, 'validStart') ?? stringProp(properties, 'startTime') ?? stringProp(properties, 'effectiveStart');
  const validEnd = stringProp(properties, 'validEnd') ?? stringProp(properties, 'endTime') ?? stringProp(properties, 'effectiveEnd');
  const minAlt = stringProp(properties, 'minAltitudeFeet') ?? stringProp(properties, 'minimumAltitudeFeet') ?? stringProp(properties, 'floor');
  const maxAlt = stringProp(properties, 'maxAltitudeFeet') ?? stringProp(properties, 'maximumAltitudeFeet') ?? stringProp(properties, 'ceiling');
  const confidence = stringProp(properties, 'confidence') ?? stringProp(properties, 'probability') ?? stringProp(properties, 'blockedProbability');
  return {
    title: stringProp(properties, 'label') ?? stringProp(properties, 'name') ?? stringProp(properties, 'featureKind') ?? String(feature.id ?? layer.label),
    subtitle: stringProp(properties, 'rationale') ?? stringProp(properties, 'explanation') ?? stringProp(properties, 'hazardType') ?? layer.label,
    source: stringProp(properties, 'sourceFamily') ?? stringProp(properties, 'sourceProduct') ?? layer.label,
    timing: validStart || validEnd ? `${validStart ?? 'start ?'} - ${validEnd ?? 'end ?'}` : undefined,
    altitude: minAlt || maxAlt ? `${minAlt ?? 'floor ?'} - ${maxAlt ?? 'ceiling ?'}` : undefined,
    confidence: confidence != null ? `${confidence}` : undefined,
    action: stringProp(properties, 'recommendedAction') ?? stringProp(properties, 'action')
  };
}

function isMapLayerId(value: string): value is MapLayerId {
  return MAP_LAYERS.some((layer) => layer.id === value);
}

function stringProp(properties: Record<string, unknown>, key: string) {
  const value = properties[key];
  if (value == null) return undefined;
  return String(value);
}
