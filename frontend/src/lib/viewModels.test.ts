import { describe, expect, it } from 'vitest';
import {
  composeAltrvSections,
  extractAltrvSections,
  notamRowsFromSources,
  rowFromMessage,
  validateAltrvSections,
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
