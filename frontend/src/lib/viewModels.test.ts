import { describe, expect, it } from 'vitest';
import {
  composeAltrvSections,
  extractAltrvSections,
  notamRowsFromSources,
  sourceRefFamily,
  sourceRefLabel,
  sourceRefRoute,
  rowFromMessage,
  validateAltrvSections,
  missionWeatherVerdict,
  weatherGuidanceFromMessage,
  weatherFeaturesFromMessages,
  weatherFeatureIdForMessageId,
  weatherRowsFromMessages
} from './viewModels';

describe('frontend view model helpers', () => {
  it('extracts and recomposes CARF/ALTRV Sections A-G without treating raw text as a NOTAM', () => {
    const raw = `A. HAWK01 KZNY
B. 2F22/I
C. KZNY
D. FL240B260 3000N 15000W 0000
E. HAWK01
F. ETD HAWK01 021200 MAR 2010 AVANA 021300
G. TAS: 300 KTAS`;

    const sections = extractAltrvSections(raw);

    expect(sections.A).toBe('HAWK01 KZNY');
    expect(sections.D).toContain('FL240B260');
    expect(composeAltrvSections(sections)).toContain('F. ETD HAWK01');
  });

  it('derives NOTAM rows from NOTAM-like traffic only', () => {
    const rows = notamRowsFromSources([
      {
        id: 'm1',
        family: 'FDC',
        direction: 'INBOUND',
        status: 'ACCEPTED',
        subject: 'FDC airspace',
        rawText: '!FDC TEST NOTAM'
      },
      {
        id: 'm2',
        family: 'CARF_ALTRV',
        direction: 'INBOUND',
        status: 'ACCEPTED',
        subject: 'ALTRV',
        rawText: 'A. HAWK01'
      }
    ]);

    expect(rows).toHaveLength(1);
    expect(rows[0]).toMatchObject({ id: 'm1', family: 'FDC', source: 'MESSAGE' });
  });

  it('classifies route-impact source references without treating NOTAMs as reservations', () => {
    expect(sourceRefFamily('FDC:abc')).toBe('NOTAM');
    expect(sourceRefFamily('DOM:abc')).toBe('NOTAM');
    expect(sourceRefFamily('PIREP:ua1')).toBe('PIREP');
    expect(sourceRefFamily('WEATHER:wx1')).toBe('WEATHER');
    expect(sourceRefFamily('ALTRV:r1')).toBe('CARF/ALTRV');
    expect(sourceRefLabel('NOTAM:abc')).toMatchObject({ family: 'NOTAM', id: 'abc', label: 'NOTAM: abc' });
    expect(sourceRefRoute('PIREP:ua1')).toBe('/messages/ua1');
    expect(sourceRefRoute('ALTRV:r1')).toBe('/deconfliction/r1');
  });

  it('routes message and weather families into distinct UI models', () => {
    const notam = rowFromMessage({
      id: 'n1',
      family: 'SNOWTAM',
      direction: 'INBOUND',
      status: 'ACCEPTED',
      subject: 'Snow',
      rawText: 'SNOWTAM'
    });
    const weather = weatherRowsFromMessages([
      { id: 'w1', family: 'SIGMET', direction: 'INBOUND', status: 'ACCEPTED', rawText: 'SIGMET' },
      { id: 'x1', family: 'DOM', direction: 'INBOUND', status: 'ACCEPTED', rawText: 'NOTAM' }
    ]);

    expect(notam.family).toBe('NOTAM');
    expect(weather.map((row) => row.id)).toEqual(['w1']);
  });

  it('derives mission weather verdict sources and recommended coordination from weather traffic', () => {
    const sigmet = {
      id: 'wx-1',
      missionId: 'm1',
      family: 'SIGMET',
      direction: 'INBOUND',
      status: 'ACCEPTED',
      subject: 'SIGMET ECHO',
      rawText: 'CONVECTIVE SIGMET LINE TOP FL430 INTSF',
      createdAt: new Date().toISOString()
    };

    const guidance = weatherGuidanceFromMessage(sigmet);
    const verdict = missionWeatherVerdict('m1', [sigmet]);

    expect(guidance).toMatchObject({
      action: 'REROUTE',
      priority: 'HIGH',
      sourceFamily: 'SIGMET',
      sourceLabel: 'SIGMET ECHO'
    });
    expect(verdict).toMatchObject({
      action: 'REROUTE',
      priority: 'HIGH',
      count: 1,
      recommendedAction: 'Weather desk + traffic manager review'
    });
    expect(verdict.confidence).toBeGreaterThan(0.8);
    expect(verdict.sources.map((source) => source.id)).toEqual(['wx-1']);
  });

  it('extracts aviation weather coordinates into map-ready weather features', () => {
    const features = weatherFeaturesFromMessages([
      {
        id: 'sigmet-1',
        family: 'SIGMET',
        direction: 'INBOUND',
        status: 'ACCEPTED',
        subject: 'CONVECTIVE SIGMET',
        rawText: 'SIGMET VALID 211200/211600 FROM 4030N07345W TO 4100N07430W TO 4000N07500W TOP FL430 MOV NE 25KT INTSF',
        createdAt: '2026-05-21T12:00:00Z'
      },
      {
        id: 'metar-1',
        family: 'METAR',
        direction: 'INBOUND',
        status: 'ACCEPTED',
        rawText: 'METAR KJFK 200000Z 18012KT 2SM RA BKN010'
      }
    ]);

    expect(features).toHaveLength(1);
    expect(features[0]).toMatchObject({
      id: 'weather-message-sigmet-1',
      geometry: { type: 'Polygon' },
      properties: {
        sourceFamily: 'WEATHER',
        sourceId: 'sigmet-1',
        recommendedAction: 'REROUTE',
        maxAltitudeFeet: 43000,
        altitudeLabel: 'TOP FL430',
        validStart: '211200Z',
        validEnd: '211600Z',
        validDurationH: 4,
        movementDirection: 'NE',
        movementSpeedKt: 25,
        movementVector: 'NE 25KT',
        geometryIntent: 'POLYGON',
        geometryLabel: 'FROM/TO POLYGON'
      }
    });
    expect((features[0].geometry?.coordinates as number[][][])[0][0]).toEqual([-73.75, 40.5]);
    expect(weatherFeatureIdForMessageId('sigmet-1')).toBe('weather-message-sigmet-1');
  });

  it('preserves point-radius and line-corridor weather geometry intent', () => {
    const features = weatherFeaturesFromMessages([
      {
        id: 'airmet-radius',
        family: 'AIRMET',
        direction: 'INBOUND',
        status: 'ACCEPTED',
        subject: 'AIRMET ICE',
        rawText: 'AIRMET ICE WITHIN 40NM OF 4030N07345W BLW FL180'
      },
      {
        id: 'cwap-line',
        family: 'WEATHER_ADVISORY',
        direction: 'INBOUND',
        status: 'ACCEPTED',
        subject: 'CWAP LINE',
        rawText: 'CWAP WI 30 NM EITHER SIDE OF LINE FROM 4030N07345W TO 4100N07430W TOP FL390'
      }
    ]);

    expect(features[0]).toMatchObject({
      id: 'weather-message-airmet-radius',
      geometry: { type: 'Polygon' },
      properties: {
        radiusNauticalMiles: 40,
        geometryIntent: 'POINT_RADIUS',
        geometryLabel: 'WITHIN 40NM',
        maxAltitudeFeet: 18000
      }
    });
    expect((features[0].geometry?.coordinates as number[][][])[0]).toHaveLength(13);
    expect(features[1]).toMatchObject({
      id: 'weather-message-cwap-line',
      geometry: { type: 'LineString' },
      properties: {
        corridorWidthNauticalMiles: 30,
        geometryIntent: 'LINE_CORRIDOR',
        geometryLabel: '30NM EITHER SIDE'
      }
    });
  });

  it('recognizes bounded-by weather polygons as polygon geometry intent', () => {
    const features = weatherFeaturesFromMessages([
      {
        id: 'sigmet-bounded',
        family: 'SIGMET',
        direction: 'INBOUND',
        status: 'ACCEPTED',
        subject: 'BOUNDED SIGMET',
        rawText: 'SIGMET BOUNDED BY 4030N07345W 4100N07430W 4000N07500W TOP FL410'
      }
    ]);

    expect(features[0]).toMatchObject({
      geometry: { type: 'Polygon' },
      properties: {
        geometryIntent: 'POLYGON',
        geometryLabel: 'BOUNDED BY',
        maxAltitudeFeet: 41000
      }
    });
  });

  it('supports decimal latitude/longitude weather geometry and keeps non-geometric METAR/TAF table-only', () => {
    const features = weatherFeaturesFromMessages([
      {
        id: 'decimal-cwap',
        family: 'WEATHER_ADVISORY',
        direction: 'INBOUND',
        status: 'ACCEPTED',
        subject: 'CWAP DECIMAL',
        rawText: 'CWAP FROM 40.50,-73.75 TO 41.00,-74.50 TO 40.00,-75.00 TOP FL390 T+3'
      },
      {
        id: 'taf-table-only',
        family: 'TAF',
        direction: 'INBOUND',
        status: 'ACCEPTED',
        subject: 'TAF KJFK',
        rawText: 'TAF KJFK 211130Z 2112/2218 18012KT P6SM BKN020 TEMPO 2112/2116 2SM RA BKN008'
      }
    ]);

    expect(features).toHaveLength(1);
    expect(features[0]).toMatchObject({
      id: 'weather-message-decimal-cwap',
      geometry: { type: 'Polygon' },
      properties: {
        geometryIntent: 'POLYGON',
        geometryLabel: 'FROM/TO POLYGON',
        forecastHour: 3,
        maxAltitudeFeet: 39000
      }
    });
    expect((features[0].geometry?.coordinates as number[][][])[0][0]).toEqual([-73.75, 40.5]);
  });

  it('validates required ALTRV sections and practical timing hints', () => {
    const diagnostics = validateAltrvSections({
      A: 'HAWK01',
      B: 'F22',
      C: '',
      D: 'FL240B260 3000N 15000W',
      E: 'KZNY',
      F: 'remarks only',
      G: ''
    });

    expect(diagnostics).toContainEqual(expect.objectContaining({ section: 'C', severity: 'ERROR' }));
    expect(diagnostics).toContainEqual(expect.objectContaining({ section: 'F', severity: 'WARN' }));
  });
});
