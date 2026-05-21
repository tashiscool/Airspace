import { describe, expect, it } from 'vitest';
import { featureDisplayLayer, featurePassesFreshnessFilter, layersForWorkbenchGroup, mapFeatureConfidence, mapFeatureFreshness, mapFeatureRiskLabel, mapFeatureSeverity, mapFeatureSourceLink, mapFeatureSourceRefs, mapFeatureSummary } from './mapLayers';

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
    expect(featureDisplayLayer({ properties: { featureKind: 'route-avoidance-candidate', constraintType: 'ROUTE_AVOIDANCE' } }))
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
        recommendedAction: 'REROUTE',
        movementVector: 'NE 25KT'
      }
    });

    expect(summary).toMatchObject({
      title: 'Convective line',
      source: 'WEATHER',
      timing: '2026-05-21T12:00:00Z - 2026-05-21T14:00:00Z',
      altitude: '18000 - 45000',
      confidence: '82% confidence',
      action: 'REROUTE',
      movement: 'NE 25KT · severe'
    });
    expect(summary?.risk).toContain('severe · 82% confidence');
    expect(mapFeatureRiskLabel({
      properties: {
        sourceFamily: 'WEATHER',
        recommendedAction: 'REROUTE',
        confidence: 0.82,
        observedAt: '2026-05-21T12:00:00Z'
      }
    })).toContain('severe · 82% confidence');
    expect(mapFeatureSourceLink({
      id: 'wx1',
      type: 'Feature',
      geometry: { type: 'Polygon', coordinates: [] },
      properties: {
        sourceFamily: 'WEATHER',
        hazardType: 'CONVECTIVE',
        sourceId: 'sigmet-9'
      }
    })).toEqual({ label: 'Open WEATHER sigmet-9', route: '/messages/sigmet-9' });
    expect(mapFeatureSourceLink({
      properties: {
        sourceFamily: 'CARF_ALTRV',
        featureKind: 'reservation',
        reservationId: 'res-1'
      }
    })).toEqual({ label: 'Open reservation res-1', route: '/deconfliction/res-1' });
    expect(mapFeatureSourceRefs({
      properties: {
        featureKind: 'weather-intersection',
        sourceRefs: ['WEATHER:WX-1', 'PIREP:UA-2', 'NOTAM:FDC-3', 'ALTRV:RES-4']
      }
    })).toEqual(expect.arrayContaining([
      expect.objectContaining({ family: 'WEATHER', id: 'WX-1', route: '/messages/WX-1' }),
      expect.objectContaining({ family: 'PIREP', id: 'UA-2', route: '/messages/UA-2' }),
      expect.objectContaining({ family: 'NOTAM', id: 'FDC-3', route: '/messages/FDC-3' }),
      expect.objectContaining({ family: 'CARF/ALTRV', id: 'RES-4', route: '/deconfliction/RES-4' })
    ]));
  });

  it('adds weather and PIREP freshness decay metadata', () => {
    const current = mapFeatureFreshness({
      properties: {
        sourceFamily: 'PIREP',
        observedAt: '2026-05-21T12:30:00Z'
      }
    }, new Date('2026-05-21T12:45:00Z'));
    const stale = mapFeatureFreshness({
      properties: {
        sourceFamily: 'PIREP',
        observedAt: '2026-05-21T10:00:00Z'
      }
    }, new Date('2026-05-21T12:45:00Z'));
    const nonWeather = mapFeatureFreshness({
      properties: {
        featureKind: 'reservation',
        sourceFamily: 'CARF_ALTRV'
      }
    }, new Date('2026-05-21T12:45:00Z'));

    expect(current).toMatchObject({ category: 'current', stale: false, label: '15m old', opacity: 1 });
    expect(stale).toMatchObject({ category: 'stale', stale: true });
    expect(stale?.opacity).toBeLessThan(0.5);
    expect(nonWeather).toBeUndefined();
  });

  it('normalizes map confidence into operational categories', () => {
    expect(mapFeatureConfidence({ properties: { confidence: 0.82 } })).toMatchObject({
      category: 'high',
      label: '82% confidence',
      opacity: 1
    });
    expect(mapFeatureConfidence({ properties: { blockedProbability: 38 } })).toMatchObject({
      category: 'low',
      label: '38% confidence'
    });
    expect(mapFeatureConfidence({ properties: {} })).toMatchObject({
      category: 'unknown',
      label: 'confidence unknown'
    });
  });

  it('normalizes map severity from operational actions and hazard text', () => {
    expect(mapFeatureSeverity({ properties: { recommendedAction: 'BLOCKED' } })).toMatchObject({
      category: 'extreme',
      label: 'extreme severity'
    });
    expect(mapFeatureSeverity({ properties: { rationale: 'SEV TURB urgent pilot report' } })).toMatchObject({
      category: 'severe',
      label: 'severe'
    });
    expect(mapFeatureSeverity({ properties: { action: 'CAUTION' } })).toMatchObject({
      category: 'moderate',
      label: 'moderate'
    });
  });

  it('filters weather and PIREP overlays by freshness without hiding operational reservations', () => {
    const now = new Date('2026-05-21T12:45:00Z');
    const stalePirep = {
      properties: {
        sourceFamily: 'PIREP',
        observedAt: '2026-05-21T10:00:00Z'
      }
    };
    const freshPirep = {
      properties: {
        sourceFamily: 'PIREP',
        observedAt: '2026-05-21T12:30:00Z'
      }
    };
    const reservation = {
      properties: {
        featureKind: 'reservation',
        sourceFamily: 'CARF_ALTRV'
      }
    };

    expect(featurePassesFreshnessFilter(stalePirep, { hideStale: true, now })).toBe(false);
    expect(featurePassesFreshnessFilter(stalePirep, { maxAgeMinutes: 60, now })).toBe(false);
    expect(featurePassesFreshnessFilter(freshPirep, { maxAgeMinutes: 60, now })).toBe(true);
    expect(featurePassesFreshnessFilter(reservation, { hideStale: true, maxAgeMinutes: 1, now })).toBe(true);
  });
});
