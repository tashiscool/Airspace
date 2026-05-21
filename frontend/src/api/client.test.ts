import { beforeEach, describe, expect, it, vi } from 'vitest';
import { api } from './client';

const calls: Array<{ url: string; init: RequestInit }> = [];

beforeEach(() => {
  calls.length = 0;
  vi.stubGlobal('localStorage', {
    getItem: (key: string) => (key === 'airspace.token' ? 'test-token' : null),
    setItem: vi.fn(),
    removeItem: vi.fn()
  });
  vi.stubGlobal('fetch', vi.fn(async (url: string, init: RequestInit) => {
    calls.push({ url, init });
    return {
      ok: true,
      status: 200,
      statusText: 'OK',
      json: async () => ({ accepted: true, id: 'result-id', points: [], warnings: [], errors: [] })
    };
  }));
});

describe('product API client', () => {
  it('sends reservation supplement transitions and reference imports to product endpoints', async () => {
    await api.transitionReservationSupplement('reservation-1', 'supplement-1', {
      status: 'APPROVED',
      actor: 'supervisor',
      note: 'reviewed'
    });
    await api.previewReferenceImport('type,identifier,latitude,longitude\nFIX,ABC,1,2');
    await api.applyReferenceImport('type,identifier,latitude,longitude\nFIX,ABC,1,2');

    expect(calls[0].url).toBe('/api/reservations/reservation-1/supplements/supplement-1/transition');
    expect(JSON.parse(String(calls[0].init.body))).toMatchObject({ status: 'APPROVED', actor: 'supervisor' });
    expect(calls[1].url).toBe('/api/reference/import/preview');
    expect(calls[2].url).toBe('/api/reference/import/apply');
    expect(calls.every((call) => (call.init.headers as Record<string, string>).Authorization === 'Bearer test-token')).toBe(true);
  });

  it('keeps workflow endpoint paths stable for parser and replay actions', async () => {
    await api.forceParseReservation('reservation-2', 'planner', 'reason');
    await api.forceDeconflictReservation('reservation-2', 'planner', 'reason');
    await api.replayDecision('decision-1');
    await api.feedTransactions('artifact-1');
    await api.missionWeatherVerdict('mission-1');
    await api.missionWeatherChanges('mission-1', '2026-05-20T00:00:00Z', 5);
    await api.affectedMissions('wx-1', 10);
    await api.missionRouteImpact('mission-1', 'reservation-1');
    await api.relevantPireps('mission-1', {
      reservationId: 'reservation-1',
      lowerAltitudeFeet: 22000,
      upperAltitudeFeet: 28000,
      altitudeToleranceFeet: 2000,
      recencyMinutes: 60,
      corridorNauticalMiles: 40
    });
    await api.coordinateWeather('mission-1', { reservationId: 'reservation-1' });
    await api.pilotBrief('mission-1');

    expect(calls.map((call) => call.url)).toEqual([
      '/api/reservations/reservation-2/force-parse',
      '/api/reservations/reservation-2/force-deconflict',
      '/api/decisions/decision-1/replay',
      '/api/feed/artifacts/artifact-1/transactions',
      '/api/missions/mission-1/weather-verdict',
      '/api/missions/mission-1/weather-changes?since=2026-05-20T00%3A00%3A00Z&limit=5',
      '/api/weather/affected-missions?sourceId=wx-1&limit=10',
      '/api/missions/mission-1/route-impact?reservationId=reservation-1',
      '/api/missions/mission-1/pireps/relevant',
      '/api/missions/mission-1/coordinate-weather',
      '/api/missions/mission-1/pilot-brief'
    ]);
    expect(JSON.parse(String(calls[8].init.body))).toMatchObject({
      reservationId: 'reservation-1',
      altitudeToleranceFeet: 2000,
      corridorNauticalMiles: 40
    });
  });

  it('surfaces JSON diagnostics from failed API responses', async () => {
    vi.stubGlobal('fetch', vi.fn(async (url: string, init: RequestInit) => {
      calls.push({ url, init });
      return {
        ok: false,
        status: 409,
        statusText: 'Conflict',
        headers: new Headers({ 'content-type': 'application/json' }),
        json: async () => ({ diagnostics: ['lock required before mutation'] })
      };
    }));

    await expect(api.lockMission('mission-1', 'planner')).rejects.toThrow(
      '409 Conflict: lock required before mutation'
    );
  });
});
