import { describe, expect, it } from 'vitest';
import { MAP_RENDERERS, mapRendererById } from './mapRenderers';

describe('map renderer registry', () => {
  it('keeps OpenLayers as default while exposing optional renderer adapters', () => {
    expect(mapRendererById(undefined).id).toBe('openlayers');
    expect(mapRendererById('missing').id).toBe('openlayers');
    expect(MAP_RENDERERS.map((renderer) => renderer.id)).toEqual([
      'openlayers',
      'leaflet',
      'cesium',
      'deckgl',
      'generic-geojson'
    ]);
  });

  it('keeps adapter entries renderer-neutral and GeoJSON based', () => {
    for (const renderer of MAP_RENDERERS) {
      expect(renderer.acceptsGeoJson({ type: 'FeatureCollection', features: [] })).toBe(true);
      expect(renderer.description.toLowerCase()).toMatch(/geojson|adapter|renderer|globe|webgl/);
    }
    expect(mapRendererById('cesium').supports3d).toBe(true);
    expect(mapRendererById('deckgl').renderMode).toBe('preview');
  });
});
