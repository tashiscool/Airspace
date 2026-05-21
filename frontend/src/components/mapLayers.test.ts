import { describe, expect, it } from 'vitest';
import { featureDisplayLayer, layersForWorkbenchGroup, mapFeatureSummary } from './mapLayers';

describe('map layer classification', () => {
  it('keeps NOTAM restrictions separate from CARF/ALTRV reservations', () => {
    expect(featureDisplayLayer({ properties: { featureKind: 'notam', sourceFamily: 'NOTAM', isAltrv: false } }))
      .toBe('notams');
    expect(featureDisplayLayer({ properties: { featureKind: 'reservation', sourceFamily: 'CARF_ALTRV', isNotam: false } }))
      .toBe('reservations');
    expect(featureDisplayLayer({ properties: { featureKind: 'flight-path', constraintType: 'CARF_ROUTE' } }))
      .toBe('flight-paths');
  });

  it('separates weather hazards from route impact overlays and conflicts', () => {
    expect(featureDisplayLayer({ properties: { featureKind: 'weather-product', sourceFamily: 'WEATHER' } }))
      .toBe('weather');
    expect(featureDisplayLayer({ properties: { featureKind: 'weather-product', sourceFamily: 'WEATHER', hazardType: 'CONVECTIVE' } }))
      .toBe('wx-convective');
    expect(featureDisplayLayer({ properties: { featureKind: 'weather-product', sourceFamily: 'WEATHER', hazardType: 'ICING' } }))
      .toBe('wx-icing');
    expect(featureDisplayLayer({ properties: { featureKind: 'weather-product', sourceFamily: 'WEATHER', hazardType: 'VOLCANIC_ASH' } }))
      .toBe('wx-ash');
    expect(featureDisplayLayer({ properties: { featureKind: 'weather-product', sourceFamily: 'WEATHER', hazardType: 'WIND_SHEAR' } }))
      .toBe('wx-wshear');
    expect(featureDisplayLayer({ properties: { featureKind: 'weather-intersection', constraintType: 'ROUTE_BLOCKAGE' } }))
      .toBe('route-impacts');
    expect(featureDisplayLayer({ properties: { featureKind: 'conflict', constraintType: 'CARF_CONFLICT' } }))
      .toBe('conflicts');
  });

  it('classifies reference data overlays separately from operational constraints', () => {
    expect(featureDisplayLayer({ properties: { featureKind: 'reference-point', sourceFamily: 'REFERENCE', pointType: 'NAVAID' } }))
      .toBe('navaids');
    expect(featureDisplayLayer({ properties: { featureKind: 'reference-point', sourceFamily: 'REFERENCE', pointType: 'FIX' } }))
      .toBe('fixes');
    expect(featureDisplayLayer({ properties: { featureKind: 'reference-point', sourceFamily: 'REFERENCE', pointType: 'AERODROME' } }))
      .toBe('aerodromes');
  });

  it('returns concrete layer ids for workbench group toggles', () => {
    expect(layersForWorkbenchGroup('ops')).toEqual(['reservations', 'flight-paths', 'conflicts', 'route-impacts', 'notams']);
    expect(layersForWorkbenchGroup('weather')).toContain('wx-convective');
    expect(layersForWorkbenchGroup('reference')).toEqual(['navaids', 'fixes', 'aerodromes']);
  });

  it('summarizes map features into operational labels', () => {
    const summary = mapFeatureSummary({
      id: 'wx1',
      type: 'Feature',
      geometry: { type: 'Polygon', coordinates: [] },
      properties: {
        label: 'Convective line',
        sourceFamily: 'WEATHER',
        hazardType: 'CONVECTIVE',
        validStart: '2026-05-21T12:00:00Z',
        validEnd: '2026-05-21T14:00:00Z',
        minAltitudeFeet: 18000,
        maxAltitudeFeet: 45000,
        confidence: 0.82,
        recommendedAction: 'REROUTE'
      }
    });

    expect(summary).toMatchObject({
      title: 'Convective line',
      source: 'WEATHER',
      timing: '2026-05-21T12:00:00Z - 2026-05-21T14:00:00Z',
      altitude: '18000 - 45000',
      confidence: '0.82',
      action: 'REROUTE'
    });
  });
});
