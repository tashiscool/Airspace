import { describe, expect, it } from 'vitest';
import { messageBodyWithMetadata, normalizeRecipients, recipientPresetSuggestions, relatedMessages } from './messagingView';

describe('messaging view helpers', () => {
  it('normalizes recipients and composes retained metadata', () => {
    expect(normalizeRecipients('carf, USNOF; carf\nkzny')).toEqual(['CARF', 'USNOF', 'KZNY']);
    expect(messageBodyWithMetadata('SVC RQ', 'carf, usnof', 'brief.pdf, map.png')).toBe('TO: CARF, USNOF\nATTACHMENTS: brief.pdf, map.png\nSVC RQ');
  });

  it('finds related traffic by mission, reservation, or family', () => {
    const selected = { id: 'm1', family: 'SIGMET', direction: 'INBOUND', status: 'ACCEPTED', missionId: 'mission-1', reservationId: 'r1' };
    const related = relatedMessages([
      selected,
      { id: 'm2', family: 'DOM', direction: 'INBOUND', status: 'ACCEPTED', missionId: 'mission-1' },
      { id: 'm3', family: 'FDC', direction: 'INBOUND', status: 'ACCEPTED', reservationId: 'r1' },
      { id: 'm4', family: 'SIGMET', direction: 'INBOUND', status: 'ACCEPTED' },
      { id: 'm5', family: 'METAR', direction: 'INBOUND', status: 'ACCEPTED' }
    ], selected);

    expect(related.map((message) => message.id)).toEqual(['m2', 'm3', 'm4']);
  });

  it('suggests known operational recipients', () => {
    expect(recipientPresetSuggestions('kz')).toEqual(expect.arrayContaining(['KZNY', 'KZDC']));
  });
});
