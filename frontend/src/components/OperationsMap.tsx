import 'ol/ol.css';
import { useEffect, useMemo, useRef, useState } from 'react';
import OlMap from 'ol/Map';
import View from 'ol/View';
import GeoJSON from 'ol/format/GeoJSON';
import VectorLayer from 'ol/layer/Vector';
import TileLayer from 'ol/layer/Tile';
import VectorSource from 'ol/source/Vector';
import OSM from 'ol/source/OSM';
import { Fill, Stroke, Style } from 'ol/style';
import type { FeatureCollection } from '../types';
import { DEFAULT_LAYOUT_PREFS, readWorkbenchJson, writeWorkbenchJson } from '../lib/workbenchState';
import { featureDisplayLayer, layerDefinition, layersForWorkbenchGroup, mapFeatureSummary, MAP_LAYERS, type MapLayerId } from './mapLayers';

export function OperationsMap({
  features,
  selectedFeatureId: controlledSelectedFeatureId,
  onSelectedFeatureIdChange
}: {
  features?: FeatureCollection;
  selectedFeatureId?: string;
  onSelectedFeatureIdChange?: (featureId: string | undefined) => void;
}) {
  const ref = useRef<HTMLDivElement | null>(null);
  const [forecastHour, setForecastHour] = useState(0);
  const [internalSelectedFeatureId, setInternalSelectedFeatureId] = useState<string | undefined>();
  const [showSelectedOnly, setShowSelectedOnly] = useState(false);
  const [fitRequest, setFitRequest] = useState<{ scope: 'all' | 'selected'; sequence: number }>({ scope: 'all', sequence: 0 });
  const selectedFeatureId = controlledSelectedFeatureId ?? internalSelectedFeatureId;
  const [enabledLayers, setEnabledLayers] = useState<Set<MapLayerId>>(
    () => {
      const prefs = readWorkbenchJson('airspace.workbench.layout', DEFAULT_LAYOUT_PREFS);
      const ids = prefs.mapLayers.filter((id): id is MapLayerId => MAP_LAYERS.some((layer) => layer.id === id));
      return new Set(ids.length ? ids : MAP_LAYERS.map((layer) => layer.id));
    }
  );
  const forecastRange = useMemo(() => forecastBounds(features), [features]);
  const visibleFeatures = useMemo<FeatureCollection | undefined>(() => {
    if (!features) return undefined;
    return {
      ...features,
      features: features.features.filter((feature, index) => {
        if (!enabledLayers.has(featureDisplayLayer(feature))) return false;
        if (showSelectedOnly && selectedFeatureId && featureKey(feature, index) !== selectedFeatureId) return false;
        const hour = numericProp(feature.properties, 'forecastHour');
        if (hour == null || !forecastRange) return true;
        const duration = numericProp(feature.properties, 'validDurationH') ?? 3;
        return forecastHour >= hour && forecastHour <= hour + duration;
      })
    };
  }, [features, enabledLayers, forecastHour, forecastRange, selectedFeatureId, showSelectedOnly]);

  useEffect(() => {
    const currentPrefs = readWorkbenchJson('airspace.workbench.layout', DEFAULT_LAYOUT_PREFS);
    writeWorkbenchJson('airspace.workbench.layout', { ...currentPrefs, mapLayers: [...enabledLayers] });
  }, [enabledLayers]);

  useEffect(() => {
    if (!ref.current) return;
    const source = new VectorSource();
    if (visibleFeatures) {
      const olFeatures = new GeoJSON().readFeatures(visibleFeatures, {
        featureProjection: 'EPSG:3857',
        dataProjection: 'EPSG:4326'
      });
      olFeatures.forEach((feature, index) => {
        const sourceFeature = visibleFeatures.features[index];
        feature.set('_airspaceFeatureKey', sourceFeature ? featureKey(sourceFeature, index) : String(index), true);
      });
      source.addFeatures(olFeatures);
    }
    const map = new OlMap({
      target: ref.current,
      layers: [
        new TileLayer({ source: new OSM() }),
        new VectorLayer({
          source,
          style: (feature) => {
            const layer = layerDefinition(featureDisplayLayer(feature.getProperties()));
            const selected = String(feature.get('_airspaceFeatureKey') ?? '') === selectedFeatureId;
            return new Style({
              stroke: new Stroke({ color: layer.stroke, width: selected ? layer.width + 2 : layer.width }),
              fill: new Fill({ color: layer.fill })
            });
          }
        })
      ],
      view: new View({ center: [0, 0], zoom: 2 })
    });
    const fitToFeatures = () => {
      map.updateSize();
      const candidates = fitRequest.scope === 'selected' && selectedFeatureId
        ? source.getFeatures().filter((feature) => String(feature.get('_airspaceFeatureKey') ?? '') === selectedFeatureId)
        : source.getFeatures();
      if (candidates.length === 0) return;
      const extent = candidates.reduce((current, feature) => {
        const featureExtent = feature.getGeometry()?.getExtent();
        if (!featureExtent) return current;
        if (!current) return [...featureExtent] as [number, number, number, number];
        return [
          Math.min(current[0], featureExtent[0]),
          Math.min(current[1], featureExtent[1]),
          Math.max(current[2], featureExtent[2]),
          Math.max(current[3], featureExtent[3])
        ] as [number, number, number, number];
      }, undefined as [number, number, number, number] | undefined);
      if (extent) {
        map.getView().fit(extent, { padding: [24, 24, 24, 24], maxZoom: 8 });
      }
    };
    map.on('singleclick', (event) => {
      const hit = map.forEachFeatureAtPixel(event.pixel, (feature) => feature);
      const key = hit?.get('_airspaceFeatureKey');
      if (key != null) selectFeature(String(key));
    });
    requestAnimationFrame(fitToFeatures);
    return () => map.setTarget(undefined);
  }, [visibleFeatures, selectedFeatureId, fitRequest]);

  const counts = useMemo(() => {
    const result = new globalThis.Map<MapLayerId, number>();
    for (const layer of MAP_LAYERS) result.set(layer.id, 0);
    for (const feature of features?.features ?? []) {
      const layer = featureDisplayLayer(feature);
      result.set(layer, (result.get(layer) ?? 0) + 1);
    }
    return result;
  }, [features]);
  const selectedFeature = useMemo(() => {
    const all = visibleFeatures?.features ?? [];
    return all.find((feature, index) => featureKey(feature, index) === selectedFeatureId) ?? all[0];
  }, [visibleFeatures, selectedFeatureId]);
  const groupedLayers = useMemo(() => {
    const groups = new globalThis.Map<string, typeof MAP_LAYERS>();
    for (const layer of MAP_LAYERS) {
      if ((counts.get(layer.id) ?? 0) === 0 && layer.id === 'other') continue;
      const group = groups.get(layer.group) ?? [];
      group.push(layer);
      groups.set(layer.group, group);
    }
    return [...groups.entries()];
  }, [counts]);

  function toggle(layerId: MapLayerId) {
    setEnabledLayers((current) => {
      const next = new Set(current);
      if (next.has(layerId)) {
        next.delete(layerId);
      } else {
        next.add(layerId);
      }
      return next;
    });
  }

  function setLayerGroup(group: 'all' | 'ops' | 'weather' | 'reference') {
    setEnabledLayers(new Set(layersForWorkbenchGroup(group)));
  }

  function selectFeature(featureId: string | undefined) {
    setInternalSelectedFeatureId(featureId);
    onSelectedFeatureIdChange?.(featureId);
  }

  function fitAll() {
    setShowSelectedOnly(false);
    setFitRequest((current) => ({ scope: 'all', sequence: current.sequence + 1 }));
  }

  function fitSelected() {
    if (!selectedFeatureId) return;
    setFitRequest((current) => ({ scope: 'selected', sequence: current.sequence + 1 }));
  }

  return (
    <section className="map-panel">
      <div className="map-toolbar" aria-label="Map layers">
        <div className="map-layer-group">
          <span>Workbench</span>
          <button className="map-layer" onClick={fitAll} type="button">Fit All</button>
          <button className="map-layer" disabled={!selectedFeatureId} onClick={fitSelected} type="button">Fit Selected</button>
          <button className={showSelectedOnly ? 'map-layer active' : 'map-layer'} onClick={() => setShowSelectedOnly((value) => !value)} type="button">Selected Only</button>
          <button className="map-layer" onClick={() => setLayerGroup('ops')} type="button">Ops</button>
          <button className="map-layer" onClick={() => setLayerGroup('weather')} type="button">Weather</button>
          <button className="map-layer" onClick={() => setLayerGroup('reference')} type="button">Reference</button>
        </div>
        {groupedLayers.map(([group, layers]) => (
          <div className="map-layer-group" key={group}>
            <span>{group}</span>
            {layers.map((layer) => (
              <button
                key={layer.id}
                className={enabledLayers.has(layer.id) ? 'map-layer active' : 'map-layer'}
                onClick={() => toggle(layer.id)}
                type="button"
                aria-pressed={enabledLayers.has(layer.id)}
                title={`${layer.label}: ${counts.get(layer.id) ?? 0} features`}
              >
                <span className="map-swatch" style={{ background: layer.stroke }} />
                {layer.label}
                <strong>{counts.get(layer.id) ?? 0}</strong>
              </button>
            ))}
          </div>
        ))}
      </div>
      {forecastRange && (
        <div className="forecast-slider">
          <span>Forecast T+{forecastHour}H</span>
          <input
            type="range"
            min={forecastRange.min}
            max={forecastRange.max}
            value={forecastHour}
            onChange={(event) => setForecastHour(Number(event.target.value))}
          />
          <span>T+{forecastRange.max}H</span>
        </div>
      )}
      <div className="map-canvas">
        <div className="map" ref={ref} />
        <svg className="map-vector-overlay" viewBox="0 0 100 100" preserveAspectRatio="none" aria-hidden="true">
          {(visibleFeatures?.features ?? []).map((feature, index) => {
            const layer = layerDefinition(featureDisplayLayer(feature));
            const path = visibleFeatures ? svgPathForFeature(feature, visibleFeatures) : undefined;
            if (!path) return null;
            const isLine = String(feature.geometry?.type ?? '').toLowerCase() === 'linestring';
            const id = featureKey(feature, index);
            const selected = id === selectedFeatureId;
            return (
              <path
                key={id}
                d={path}
                fill={isLine ? 'none' : layer.fill}
                stroke={layer.stroke}
                strokeWidth={selected ? 2.1 : isLine ? 1.3 : 0.65}
                vectorEffect="non-scaling-stroke"
              />
            );
          })}
        </svg>
      </div>
      <div className="map-semantics">
        <div><strong>CARF/ALTRV</strong><span>Reservations, protected route volumes, timing metadata, conflicts.</span></div>
        <div><strong>NOTAM</strong><span>Restrictions and advisories stay separate from reservations.</span></div>
        <div><strong>Weather/PIREP</strong><span>Hazards, reports, and route blockage overlays from the decision engine.</span></div>
      </div>
      <div className="map-feature-browser">
        <div>
          <strong>Feature Detail</strong>
          <span>{visibleFeatures?.features.length ?? 0} visible / {features?.features.length ?? 0} total</span>
        </div>
        <select value={selectedFeatureId ?? ''} onChange={(event) => selectFeature(event.target.value || undefined)}>
          {(visibleFeatures?.features ?? []).map((feature, index) => {
            const id = featureKey(feature, index);
            return <option key={id} value={id}>{featureLabel(feature, index)}</option>;
          })}
        </select>
        {selectedFeature ? <FeatureProperties feature={selectedFeature} /> : <p className="muted">No visible features.</p>}
      </div>
    </section>
  );
}

function FeatureProperties({ feature }: { feature: FeatureCollection['features'][number] }) {
  const properties = feature.properties ?? {};
  const summary = mapFeatureSummary(feature);
  const rows = Object.entries(properties)
    .filter(([, value]) => value != null && ['string', 'number', 'boolean'].includes(typeof value))
    .slice(0, 10);
  return (
    <>
      {summary && (
        <div className="map-feature-summary">
          <strong>{summary.title}</strong>
          <p>{summary.subtitle}</p>
          <div>
            <span>{summary.source}</span>
            {summary.timing && <span>{summary.timing}</span>}
            {summary.altitude && <span>{summary.altitude}</span>}
            {summary.confidence && <span>{summary.confidence}</span>}
            {summary.action && <span>{summary.action}</span>}
          </div>
        </div>
      )}
      <dl className="map-feature-properties">
        {rows.map(([key, value]) => (
          <div key={key}>
            <dt>{key}</dt>
            <dd>{String(value)}</dd>
          </div>
        ))}
      </dl>
    </>
  );
}

function featureLabel(feature: FeatureCollection['features'][number], index: number) {
  const properties = feature.properties ?? {};
  return String(properties.label ?? properties.name ?? properties.featureKind ?? properties.sourceFamily ?? feature.id ?? `Feature ${index + 1}`);
}

function featureKey(feature: FeatureCollection['features'][number], index = 0) {
  return String(feature.id ?? feature.properties?.id ?? feature.properties?.label ?? feature.properties?.name ?? index);
}

function numericProp(properties: Record<string, unknown> | undefined, key: string) {
  const value = properties?.[key];
  if (typeof value === 'number') return value;
  if (typeof value === 'string' && value.trim() !== '' && Number.isFinite(Number(value))) return Number(value);
  return undefined;
}

function forecastBounds(collection?: FeatureCollection) {
  const hours = (collection?.features ?? [])
    .map((feature) => numericProp(feature.properties, 'forecastHour'))
    .filter((value): value is number => value != null);
  if (!hours.length) return undefined;
  return { min: Math.min(0, ...hours), max: Math.max(...hours.map((hour) => hour + 3)) };
}

function svgPathForFeature(feature: FeatureCollection['features'][number], collection: FeatureCollection): string | undefined {
  const bounds = coordinateBounds(collection);
  if (!bounds) return undefined;
  const geometry = feature.geometry ?? {};
  const type = String(geometry.type ?? '').toLowerCase();
  const coordinates = geometry.coordinates;
  if (type === 'linestring' && Array.isArray(coordinates)) {
    return pathForPositions(coordinates as unknown[], bounds, false);
  }
  if (type === 'polygon' && Array.isArray(coordinates) && Array.isArray(coordinates[0])) {
    return pathForPositions(coordinates[0] as unknown[], bounds, true);
  }
  return undefined;
}

function coordinateBounds(collection: FeatureCollection): { minLon: number; maxLon: number; minLat: number; maxLat: number } | undefined {
  const positions: number[][] = [];
  for (const feature of collection.features) {
    collectPositions(feature.geometry?.coordinates, positions);
  }
  if (!positions.length) return undefined;
  const lons = positions.map((position) => position[0]);
  const lats = positions.map((position) => position[1]);
  const minLon = Math.min(...lons);
  const maxLon = Math.max(...lons);
  const minLat = Math.min(...lats);
  const maxLat = Math.max(...lats);
  return {
    minLon: minLon === maxLon ? minLon - 0.5 : minLon,
    maxLon: minLon === maxLon ? maxLon + 0.5 : maxLon,
    minLat: minLat === maxLat ? minLat - 0.5 : minLat,
    maxLat: minLat === maxLat ? maxLat + 0.5 : maxLat
  };
}

function collectPositions(value: unknown, positions: number[][]) {
  if (!Array.isArray(value)) return;
  if (typeof value[0] === 'number' && typeof value[1] === 'number') {
    positions.push(value as number[]);
    return;
  }
  for (const child of value) collectPositions(child, positions);
}

function pathForPositions(values: unknown[], bounds: { minLon: number; maxLon: number; minLat: number; maxLat: number }, close: boolean) {
  const points = values
    .filter((value): value is number[] => Array.isArray(value) && typeof value[0] === 'number' && typeof value[1] === 'number')
    .map((position) => project(position, bounds));
  if (!points.length) return undefined;
  const [first, ...rest] = points;
  return `M ${first[0].toFixed(2)} ${first[1].toFixed(2)} ${rest.map((point) => `L ${point[0].toFixed(2)} ${point[1].toFixed(2)}`).join(' ')}${close ? ' Z' : ''}`;
}

function project(position: number[], bounds: { minLon: number; maxLon: number; minLat: number; maxLat: number }): [number, number] {
  const pad = 8;
  const x = pad + ((position[0] - bounds.minLon) / (bounds.maxLon - bounds.minLon)) * (100 - pad * 2);
  const y = pad + ((bounds.maxLat - position[1]) / (bounds.maxLat - bounds.minLat)) * (100 - pad * 2);
  return [x, y];
}
