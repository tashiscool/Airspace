import 'ol/ol.css';
import { useEffect, useRef } from 'react';
import Map from 'ol/Map';
import View from 'ol/View';
import GeoJSON from 'ol/format/GeoJSON';
import VectorLayer from 'ol/layer/Vector';
import TileLayer from 'ol/layer/Tile';
import VectorSource from 'ol/source/Vector';
import OSM from 'ol/source/OSM';
import { Fill, Stroke, Style } from 'ol/style';
import type { FeatureCollection } from '../types';

export function OperationsMap({ features }: { features?: FeatureCollection }) {
  const ref = useRef<HTMLDivElement | null>(null);
  useEffect(() => {
    if (!ref.current) return;
    const source = new VectorSource();
    if (features) {
      source.addFeatures(new GeoJSON().readFeatures(features, {
        featureProjection: 'EPSG:3857',
        dataProjection: 'EPSG:4326'
      }));
    }
    const map = new Map({
      target: ref.current,
      layers: [
        new TileLayer({ source: new OSM() }),
        new VectorLayer({
          source,
          style: new Style({
            stroke: new Stroke({ color: '#0f766e', width: 2 }),
            fill: new Fill({ color: 'rgba(15, 118, 110, 0.22)' })
          })
        })
      ],
      view: new View({ center: [0, 0], zoom: 2 })
    });
    if (source.getFeatures().length > 0) {
      const extent = source.getExtent();
      if (extent) {
        map.getView().fit(extent, { padding: [24, 24, 24, 24], maxZoom: 8 });
      }
    }
    return () => map.setTarget(undefined);
  }, [features]);
  return <div className="map" ref={ref} />;
}
