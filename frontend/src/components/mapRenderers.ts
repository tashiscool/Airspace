import type { FeatureCollection } from '../types';

export type MapRendererId = 'openlayers' | 'leaflet' | 'cesium' | 'deckgl' | 'generic-geojson';

export type MapRendererAdapter = {
  id: MapRendererId;
  label: string;
  state: 'active' | 'optional-package' | 'adapter seam';
  description: string;
  supports3d: boolean;
  renderMode: 'native' | 'preview';
  acceptsGeoJson: (features?: FeatureCollection) => boolean;
};

export const MAP_RENDERERS: MapRendererAdapter[] = [
  {
    id: 'openlayers',
    label: 'OpenLayers',
    state: 'active',
    description: 'Default operational 2D renderer using backend-neutral GeoJSON.',
    supports3d: false,
    renderMode: 'native',
    acceptsGeoJson: Boolean
  },
  {
    id: 'leaflet',
    label: 'Leaflet',
    state: 'optional-package',
    description: 'Optional lightweight 2D map adapter. Install Leaflet to render natively.',
    supports3d: false,
    renderMode: 'preview',
    acceptsGeoJson: Boolean
  },
  {
    id: 'cesium',
    label: 'Cesium',
    state: 'optional-package',
    description: 'Optional globe/3D airspace adapter for altitude-aware playback.',
    supports3d: true,
    renderMode: 'preview',
    acceptsGeoJson: Boolean
  },
  {
    id: 'deckgl',
    label: 'deck.gl',
    state: 'optional-package',
    description: 'Optional high-volume WebGL renderer for campaign and NAS-scale overlays.',
    supports3d: true,
    renderMode: 'preview',
    acceptsGeoJson: Boolean
  },
  {
    id: 'generic-geojson',
    label: 'GeoJSON Table',
    state: 'adapter seam',
    description: 'Renderer-neutral inspection mode for environments without a map package.',
    supports3d: false,
    renderMode: 'preview',
    acceptsGeoJson: Boolean
  }
];

export function mapRendererById(id: string | undefined): MapRendererAdapter {
  return MAP_RENDERERS.find((renderer) => renderer.id === id) ?? MAP_RENDERERS[0];
}
