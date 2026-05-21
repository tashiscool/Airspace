import { describe, expect, it } from 'vitest';
import { conflictReviewSummary } from './deconflictionView';

describe('deconfliction view helpers', () => {
  it('normalizes conflict metrics with thresholds and source metadata', () => {
    const summary = conflictReviewSummary({
      id: 'c1',
      type: 'Feature',
      geometry: { type: 'LineString', coordinates: [] },
      properties: {
        minimumLateralDistanceNauticalMiles: 22,
        minimumRequiredLateralNauticalMiles: 60,
        verticalSeparationFeet: 800,
        minimumRequiredVerticalFeet: 1000,
        durationMinutes: 7,
        sourceRatios: '0.25,0.75',
        sourcePoints: ['FIXA', 'FIXB']
      }
    });

    expect(summary?.severity).toBe('HIGH');
    expect(summary?.metrics.find((metric) => metric.label === 'Lateral')).toMatchObject({
      status: 'VIOLATION',
      threshold: 60
    });
    expect(summary?.metrics.find((metric) => metric.label === 'Vertical')).toMatchObject({
      status: 'LIMITED',
      threshold: 1000
    });
    expect(summary?.sourceRatios).toEqual(['0.25', '0.75']);
    expect(summary?.sourcePoints).toEqual(['FIXA', 'FIXB']);
  });
});
