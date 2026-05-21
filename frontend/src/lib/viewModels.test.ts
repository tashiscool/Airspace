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
        movementVector: 'NE 25KT'
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
