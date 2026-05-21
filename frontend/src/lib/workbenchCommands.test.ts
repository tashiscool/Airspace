import { describe, expect, it } from 'vitest';
import { buildWorkbenchCommands, filterWorkbenchCommands, nextPaletteIndex, routesForSelection, routeForTarget } from './workbenchCommands';

describe('workbench commands', () => {
  it('builds selected-context commands', () => {
    const commands = buildWorkbenchCommands({ reservationId: 'r-42', decisionId: 'd-7' });

    expect(commands.find((command) => command.id === 'review-conflicts')?.route).toBe('/deconfliction/r-42');
    expect(commands.find((command) => command.id === 'verify-replay')?.route).toBe('/decisions/d-7');
  });

  it('routes typed target jumps', () => {
    expect(routeForTarget('mission', ' M1 ')).toBe('/missions/M1');
    expect(routeForTarget('reservation', 'R1')).toBe('/deconfliction/R1');
    expect(routeForTarget('message', 'MSG1')).toBe('/messages/MSG1');
    expect(routeForTarget('feed', 'F1')).toBe('/feed/F1');
    expect(routeForTarget('decision', 'D1')).toBe('/decisions/D1');
    expect(routeForTarget('decision', '   ')).toBeUndefined();
  });

  it('filters by labels, hints, and routes', () => {
    const commands = buildWorkbenchCommands();

    expect(filterWorkbenchCommands(commands, 'notam').map((command) => command.id)).toContain('review-notams');
    expect(filterWorkbenchCommands(commands, '/feed').map((command) => command.id)).toContain('/feed');
  });

  it('derives quick routes from the active workbench selection', () => {
    expect(routesForSelection({
      missionId: 'M1',
      reservationId: 'R1',
      messageId: 'MSG1',
      feedArtifactId: 'F1',
      decisionId: 'D1',
      sourceFamily: 'NOTAM'
    })).toEqual([
      { label: 'Mission', route: '/missions/M1' },
      { label: 'Reservation', route: '/deconfliction/R1' },
      { label: 'Message', route: '/messages/MSG1' },
      { label: 'Feed', route: '/feed/F1' },
      { label: 'Decision', route: '/decisions/D1' },
      { label: 'NOTAMs', route: '/notams' }
    ]);
  });

  it('cycles command palette selection indexes', () => {
    expect(nextPaletteIndex(-1, 1, 3)).toBe(0);
    expect(nextPaletteIndex(-1, -1, 3)).toBe(2);
    expect(nextPaletteIndex(2, 1, 3)).toBe(0);
    expect(nextPaletteIndex(0, -1, 3)).toBe(2);
    expect(nextPaletteIndex(0, 1, 0)).toBe(-1);
  });
});
