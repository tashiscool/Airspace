import { describe, expect, it } from 'vitest';
import { shouldWarnForNavigation } from './navigationGuard';

describe('navigation guard helpers', () => {
  it('warns only for internal route changes with unsaved edits', () => {
    expect(shouldWarnForNavigation(true, '/missions/m1/reservations/r1', '/deconfliction/r1')).toBe(true);
    expect(shouldWarnForNavigation(true, '/missions/m1/reservations/r1', '/missions/m1/reservations/r1')).toBe(false);
    expect(shouldWarnForNavigation(false, '/missions/m1/reservations/r1', '/deconfliction/r1')).toBe(false);
    expect(shouldWarnForNavigation(true, '/missions/m1/reservations/r1', 'https://example.com/deconfliction/r1')).toBe(false);
  });
});
