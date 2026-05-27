import { beforeEach, describe, expect, it, vi } from 'vitest';
import { api } from './client';

const calls: Array<{ url: string; init: RequestInit }> = [];

function bodyFor(url: string) {
  const call = calls.find((candidate) => candidate.url === url);
  expect(call, `Expected request to ${url}`).toBeDefined();
  return JSON.parse(String(call?.init.body));
}

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
    await api.weatherLiveStatus();
    await api.pollLiveWeather({ products: ['metar', 'airsigmet'], hoursBeforeNow: 2, maxResults: 25 });
    await api.weatherPatterns();
    await api.weatherPatternFeatures();
    await api.weatherEvents();
    await api.sampleWeatherRoute({ route: [[30, -150, 24000], [31, -149, 24000]], corridorNauticalMiles: 40 });
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
    await api.runAgent({ agentType: 'ALL', missionId: 'mission-1' });
    await api.weatherImpactAgent({ missionId: 'mission-1' });
    await api.missionRiskAgent({ missionId: 'mission-1' });
    await api.rerouteAnalysisAgent({ missionId: 'mission-1', reservationId: 'reservation-1' });
    await api.coordinationDraftAgent({ missionId: 'mission-1' });
    await api.pilotBriefAgent({ missionId: 'mission-1' });
    await api.dataIntegrityAgent({ missionId: 'mission-1' });
    await api.replayAuditAgent({ decisionId: 'decision-1' });
    await api.agentDelta({ previousDecisionId: 'decision-0', decisionId: 'decision-1', missionId: 'mission-1' });
    await api.generateAgentScenario({ scenarioType: 'VIABLE_REROUTE', missionNumber: 'NXGEN-1' });
    await api.agentStatus();
    await api.agentMetrics();
    await api.agentRuns(5, { agentType: 'ALL', missionId: 'mission-1', accepted: true, sourceFamily: 'WEATHER' });
    await api.agentRun('run-1');
    await api.agentTasks('OPEN', 8, { priority: 'HIGH', assignedRole: 'planner', sourceFamily: 'WEATHER', routeContains: 'mission-1' });
    await api.agentTask('task-1');
    await api.transitionAgentTask('task-1', { status: 'ACKNOWLEDGED', actor: 'planner', note: 'reviewed' });

    expect(calls.map((call) => call.url)).toEqual([
      '/api/reservations/reservation-2/force-parse',
      '/api/reservations/reservation-2/force-deconflict',
      '/api/decisions/decision-1/replay',
      '/api/feed/artifacts/artifact-1/transactions',
      '/api/missions/mission-1/weather-verdict',
      '/api/missions/mission-1/weather-changes?since=2026-05-20T00%3A00%3A00Z&limit=5',
      '/api/weather/affected-missions?sourceId=wx-1&limit=10',
      '/api/weather/live/status',
      '/api/weather/live/poll',
      '/api/weather/patterns',
      '/api/weather/patterns/features',
      '/api/weather/events',
      '/api/weather/route-sample',
      '/api/missions/mission-1/route-impact?reservationId=reservation-1',
      '/api/missions/mission-1/pireps/relevant',
      '/api/missions/mission-1/coordinate-weather',
      '/api/missions/mission-1/pilot-brief',
      '/api/agents/run',
      '/api/agents/weather-impact',
      '/api/agents/mission-risk',
      '/api/agents/reroute-analysis',
      '/api/agents/coordination-draft',
      '/api/agents/pilot-brief',
      '/api/agents/data-integrity',
      '/api/agents/replay-audit',
      '/api/agents/delta',
      '/api/agents/scenario/generate',
      '/api/agents/status',
      '/api/agents/metrics',
      '/api/agents/runs?limit=5&agentType=ALL&missionId=mission-1&accepted=true&sourceFamily=WEATHER',
      '/api/agents/runs/run-1',
      '/api/agents/tasks?status=OPEN&limit=8&priority=HIGH&assignedRole=planner&sourceFamily=WEATHER&routeContains=mission-1',
      '/api/agents/tasks/task-1',
      '/api/agents/tasks/task-1/transition'
    ]);
    expect(bodyFor('/api/weather/live/poll')).toMatchObject({ products: ['metar', 'airsigmet'], hoursBeforeNow: 2 });
    expect(bodyFor('/api/weather/route-sample')).toMatchObject({ corridorNauticalMiles: 40 });
    expect(bodyFor('/api/missions/mission-1/pireps/relevant')).toMatchObject({
      reservationId: 'reservation-1',
      altitudeToleranceFeet: 2000,
      corridorNauticalMiles: 40
    });
    expect(bodyFor('/api/agents/run')).toMatchObject({ agentType: 'ALL', missionId: 'mission-1' });
    expect(bodyFor('/api/agents/reroute-analysis')).toMatchObject({ missionId: 'mission-1', reservationId: 'reservation-1' });
    expect(bodyFor('/api/agents/delta')).toMatchObject({ previousDecisionId: 'decision-0', decisionId: 'decision-1' });
    expect(bodyFor('/api/agents/scenario/generate')).toMatchObject({ scenarioType: 'VIABLE_REROUTE', missionNumber: 'NXGEN-1' });
    expect(bodyFor('/api/agents/tasks/task-1/transition')).toMatchObject({ status: 'ACKNOWLEDGED', actor: 'planner' });
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
