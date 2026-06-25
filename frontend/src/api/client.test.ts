import { beforeEach, describe, expect, it, vi } from 'vitest';
import { api } from './client';

const calls: Array<{ url: string; init: RequestInit }> = [];

function bodyFor(url: string) {
  const call = calls.find((candidate) => candidate.url === url && candidate.init.body !== undefined);
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
    await api.runAirspaceAgent({ agentType: 'COORDINATION_DRAFT_AGENT', missionId: 'mission-1' });
    await api.weatherImpactAgent({ missionId: 'mission-1' });
    await api.missionRiskAgent({ missionId: 'mission-1' });
    await api.rerouteAnalysisAgent({ missionId: 'mission-1', reservationId: 'reservation-1' });
    await api.coordinationDraftAgent({ missionId: 'mission-1' });
    await api.pilotBriefAgent({ missionId: 'mission-1' });
    await api.dataIntegrityAgent({ missionId: 'mission-1' });
    await api.replayAuditAgent({ decisionId: 'decision-1' });
    await api.safetyLabAgent({ agentType: 'SAFETY_LAB_ALL', scenarioId: 'oceanic-altrv-convection' });
    await api.agentDelta({ previousDecisionId: 'decision-0', decisionId: 'decision-1', missionId: 'mission-1' });
    await api.generateAgentScenario({ scenarioType: 'VIABLE_REROUTE', missionNumber: 'NXGEN-1' });
    await api.evaluateAgentStability({ iterations: 3, agentRunRequest: { agentType: 'MISSION_RISK', missionId: 'mission-1' } });
    await api.agentRiskAssessments();
    await api.mcpServers();
    await api.mcpTools('airspace-first-party');
    await api.callMcpTool({
      serverId: 'airspace-first-party',
      toolId: 'airspace.mission.weather_verdict',
      arguments: { missionId: 'mission-1' }
    });
    await api.mcpReceipts(5);
    await api.enqueueAgentJob({
      actor: 'planner',
      agentRunRequest: { agentType: 'MISSION_RISK', missionId: 'mission-1' }
    });
    await api.agentJobs(5);
    await api.agentJob('job-1');
    await api.simulationScenarios();
    await api.simulationScenarioBundle('low-vis-rvr-smgcs');
    await api.validateSimulationScenario({ id: 'scenario-1', scenario: { id: 'scenario-1', name: 'Scenario 1', route: [[30, -150], [31, -149]], events: [], sourceRefs: [], expectedSourceFamilies: [], sensitivityDefaults: {} } as never, kpiGates: [], expectedSummary: {} });
    await api.importSimulationScenario({ id: 'scenario-1', scenario: { id: 'scenario-1', name: 'Scenario 1', route: [[30, -150], [31, -149]], events: [{ id: 'evt-1', offsetMinutes: 0, family: 'WEATHER', sourceRefs: [] }], expectedSourceFamilies: [], sensitivityDefaults: {} }, kpiGates: [], expectedSummary: {} });
    await api.validateTrafficReplay({
      id: 'traffic-replay-1',
      sourceMode: 'LOCAL_FIXTURE_REPLAY',
      flightPlans: [],
      positions: [],
      airportDemand: [],
      sectorDemand: [],
      trafficManagementInitiatives: [],
      sourceRefs: [],
      assumptions: [],
      diagnostics: []
    });
    await api.historicalReplayDays();
    await api.historicalReplayDay('public-like-jfk-lowvis-opsnet-bts-awc');
    await api.loadHistoricalReplay({ dayId: 'public-like-jfk-lowvis-opsnet-bts-awc', runSimulation: true });
    await api.historicalReplayCalibration({ dayId: 'public-like-jfk-lowvis-opsnet-bts-awc' });
    await api.previewNationalDemandCapacity({ id: 'national-1', flightCount: 1000, airportCount: 12, sectorCount: 24 });
    await api.tfmBoard({ demandCapacityConfig: { id: 'tfm-1', flightCount: 500, airportCount: 10, sectorCount: 18 }, focusMinute: 30 });
    await api.outcomeMetrics({ runSimulation: true, scenarioId: 'low-vis-rvr-smgcs', demandCapacityConfig: { id: 'outcome-1', flightCount: 300, airportCount: 6, sectorCount: 12 } });
    await api.runSimulation({ scenarioId: 'low-vis-rvr-smgcs', includeSensitivity: true, tickIntervalSeconds: 60, durationMinutes: 30, randomSeed: 7 });
    await api.runSimulationCampaign({ scenarioIds: ['low-vis-rvr-smgcs'], actor: 'planner', randomSeed: 7 });
    await api.simulationRun('sim-run-1');
    await api.simulationTimeline('sim-run-1');
    await api.simulationFeatures('sim-run-1');
    await api.simulationWorldState('sim-run-1');
    await api.simulationReplay('sim-run-1');
    await api.simulationCampaignReport('sim-campaign-1');
    await api.simulationCampaignDossier('sim-campaign-1');
    await api.generateSimulationScenarios({ scenarioType: 'LOW_VISIBILITY_PROCEDURE_AMBIGUITY', count: 2, focusAreas: ['RVR'] });
    await api.redTeamSimulation({ runId: 'sim-run-1' });
    await api.agentStatus();
    await api.agentWorkloads();
    await api.agentMetrics();
    await api.agentRuns(5, { agentType: 'ALL', missionId: 'mission-1', accepted: true, sourceFamily: 'WEATHER' });
    await api.agentRun('run-1');
    await api.airspaceAgentRuns(3, { agentType: 'SAFETY_LAB_ALL', missionId: 'mission-1', accepted: true, sourceFamily: 'WEATHER' });
    await api.airspaceAgentRun('run-1');
    await api.agentTasks('OPEN', 8, { priority: 'HIGH', assignedRole: 'planner', sourceFamily: 'WEATHER', routeContains: 'mission-1' });
    await api.agentTask('task-1');
    await api.transitionAgentTask('task-1', { status: 'ACKNOWLEDGED', actor: 'planner', note: 'reviewed' });
    await api.acknowledgeAirspaceAgentTask('task-1', { actor: 'planner', note: 'ack' });
    await api.resolveAirspaceAgentTask('task-1', { actor: 'planner', note: 'done' });
    await api.gaps();
    await api.releaseGates();
    await api.providersStatus();
    await api.pollProviderWeather({ products: ['metar'], maxResults: 5 });
    await api.runCalibration({ datasetId: 'fixture-low-vis-weather', includeSyntheticScale: true, actor: 'planner' });
    await api.calibrationReports();
    await api.safetyDossier();
    await api.commonOperatingPicture();
    await api.collaborativeParticipants();
    await api.collaborativeProposals();
    await api.createCollaborativeProposal({ missionId: 'mission-1', recommendedAction: 'REROUTE', sourceRefs: ['WEATHER:1'] });
    await api.commentOnCollaborativeProposal('proposal-1', { actor: 'dispatcher', note: 'accept if approved' });
    await api.acceptCollaborativeProposal('proposal-1', { actor: 'dispatcher', role: 'AIRLINE_OPERATOR' });
    await api.rejectCollaborativeProposal('proposal-2', { actor: 'dispatcher', note: 'cannot accept' });
    await api.approveCollaborativeProposal('proposal-1', { actor: 'tmu', role: 'TRAFFIC_MANAGER' });
    await api.deliverCollaborativeProposal('proposal-1', { actor: 'tmu', deliveryChannel: 'TELECON', externalReceiptId: 'cop-77' });
    await api.approveCoordination('draft-1', { actor: 'supervisor', note: 'reviewed' });
    await api.markCoordinationDelivered('draft-1', { actor: 'supervisor', deliveryChannel: 'PHONE', externalReceiptId: 'ops-log-1' });

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
      '/api/agents/airspace/run',
      '/api/agents/weather-impact',
      '/api/agents/mission-risk',
      '/api/agents/reroute-analysis',
      '/api/agents/coordination-draft',
      '/api/agents/pilot-brief',
      '/api/agents/data-integrity',
      '/api/agents/replay-audit',
      '/api/agents/safety-lab',
      '/api/agents/delta',
      '/api/agents/scenario/generate',
      '/api/agents/evaluate/stability',
      '/api/agents/risk-assessments',
      '/api/agents/mcp/servers',
      '/api/agents/mcp/servers/airspace-first-party/tools',
      '/api/agents/mcp/tools/call',
      '/api/agents/mcp/receipts?limit=5',
      '/api/agents/jobs',
      '/api/agents/jobs?limit=5',
      '/api/agents/jobs/job-1',
      '/api/simulations/scenarios',
      '/api/simulations/scenarios/low-vis-rvr-smgcs/bundle',
      '/api/simulations/scenarios/validate',
      '/api/simulations/scenarios/import',
      '/api/simulations/traffic-replay/validate',
      '/api/simulations/historical-replay/days',
      '/api/simulations/historical-replay/days/public-like-jfk-lowvis-opsnet-bts-awc',
      '/api/simulations/historical-replay/load',
      '/api/simulations/historical-replay/calibrate',
      '/api/simulations/national-demand/preview',
      '/api/tfm/board',
      '/api/outcomes/metrics',
      '/api/simulations/run',
      '/api/simulations/campaign',
      '/api/simulations/runs/sim-run-1',
      '/api/simulations/runs/sim-run-1/timeline',
      '/api/simulations/runs/sim-run-1/features',
      '/api/simulations/runs/sim-run-1/world-state',
      '/api/simulations/runs/sim-run-1/replay',
      '/api/simulations/campaigns/sim-campaign-1/report',
      '/api/simulations/campaigns/sim-campaign-1/dossier',
      '/api/simulations/agents/generate-scenarios',
      '/api/simulations/agents/red-team',
      '/api/agents/status',
      '/api/agents/workloads',
      '/api/agents/metrics',
      '/api/agents/runs?limit=5&agentType=ALL&missionId=mission-1&accepted=true&sourceFamily=WEATHER',
      '/api/agents/runs/run-1',
      '/api/agents/airspace/runs?limit=3&agentType=SAFETY_LAB_ALL&missionId=mission-1&accepted=true&sourceFamily=WEATHER',
      '/api/agents/airspace/runs/run-1',
      '/api/agents/tasks?status=OPEN&limit=8&priority=HIGH&assignedRole=planner&sourceFamily=WEATHER&routeContains=mission-1',
      '/api/agents/tasks/task-1',
      '/api/agents/tasks/task-1/transition',
      '/api/agents/airspace/tasks/task-1/acknowledge',
      '/api/agents/airspace/tasks/task-1/resolve',
      '/api/gaps',
      '/api/gaps/release-gates',
      '/api/providers/status',
      '/api/providers/weather/poll',
      '/api/calibration/run',
      '/api/calibration/reports',
      '/api/safety/dossier',
      '/api/collaboration/common-operating-picture',
      '/api/collaboration/participants',
      '/api/collaboration/proposals',
      '/api/collaboration/proposals',
      '/api/collaboration/proposals/proposal-1/comment',
      '/api/collaboration/proposals/proposal-1/accept',
      '/api/collaboration/proposals/proposal-2/reject',
      '/api/collaboration/proposals/proposal-1/approve',
      '/api/collaboration/proposals/proposal-1/deliver',
      '/api/coordination/draft-1/approve',
      '/api/coordination/draft-1/mark-delivered'
    ]);
    expect(bodyFor('/api/weather/live/poll')).toMatchObject({ products: ['metar', 'airsigmet'], hoursBeforeNow: 2 });
    expect(bodyFor('/api/weather/route-sample')).toMatchObject({ corridorNauticalMiles: 40 });
    expect(bodyFor('/api/missions/mission-1/pireps/relevant')).toMatchObject({
      reservationId: 'reservation-1',
      altitudeToleranceFeet: 2000,
      corridorNauticalMiles: 40
    });
    expect(bodyFor('/api/agents/run')).toMatchObject({ agentType: 'ALL', missionId: 'mission-1' });
    expect(bodyFor('/api/agents/airspace/run')).toMatchObject({ agentType: 'COORDINATION_DRAFT_AGENT', missionId: 'mission-1' });
    expect(bodyFor('/api/agents/reroute-analysis')).toMatchObject({ missionId: 'mission-1', reservationId: 'reservation-1' });
    expect(bodyFor('/api/agents/delta')).toMatchObject({ previousDecisionId: 'decision-0', decisionId: 'decision-1' });
    expect(bodyFor('/api/agents/safety-lab')).toMatchObject({ agentType: 'SAFETY_LAB_ALL', scenarioId: 'oceanic-altrv-convection' });
    expect(bodyFor('/api/agents/scenario/generate')).toMatchObject({ scenarioType: 'VIABLE_REROUTE', missionNumber: 'NXGEN-1' });
    expect(bodyFor('/api/agents/evaluate/stability')).toMatchObject({ iterations: 3, agentRunRequest: { agentType: 'MISSION_RISK', missionId: 'mission-1' } });
    expect(bodyFor('/api/agents/mcp/tools/call')).toMatchObject({ toolId: 'airspace.mission.weather_verdict' });
    expect(bodyFor('/api/agents/jobs')).toMatchObject({ agentRunRequest: { agentType: 'MISSION_RISK', missionId: 'mission-1' } });
    expect(bodyFor('/api/simulations/scenarios/validate')).toMatchObject({ id: 'scenario-1' });
    expect(bodyFor('/api/simulations/scenarios/import')).toMatchObject({ id: 'scenario-1' });
    expect(bodyFor('/api/simulations/traffic-replay/validate')).toMatchObject({ id: 'traffic-replay-1', sourceMode: 'LOCAL_FIXTURE_REPLAY' });
    expect(bodyFor('/api/simulations/historical-replay/load')).toMatchObject({ dayId: 'public-like-jfk-lowvis-opsnet-bts-awc', runSimulation: true });
    expect(bodyFor('/api/simulations/historical-replay/calibrate')).toMatchObject({ dayId: 'public-like-jfk-lowvis-opsnet-bts-awc' });
    expect(bodyFor('/api/simulations/national-demand/preview')).toMatchObject({ id: 'national-1', flightCount: 1000 });
    expect(bodyFor('/api/tfm/board')).toMatchObject({ demandCapacityConfig: { id: 'tfm-1', flightCount: 500 }, focusMinute: 30 });
    expect(bodyFor('/api/outcomes/metrics')).toMatchObject({ runSimulation: true, scenarioId: 'low-vis-rvr-smgcs', demandCapacityConfig: { id: 'outcome-1', flightCount: 300 } });
    expect(bodyFor('/api/simulations/run')).toMatchObject({ scenarioId: 'low-vis-rvr-smgcs', includeSensitivity: true, tickIntervalSeconds: 60, durationMinutes: 30, randomSeed: 7 });
    expect(bodyFor('/api/simulations/campaign')).toMatchObject({ scenarioIds: ['low-vis-rvr-smgcs'], actor: 'planner', randomSeed: 7 });
    expect(bodyFor('/api/simulations/agents/generate-scenarios')).toMatchObject({ scenarioType: 'LOW_VISIBILITY_PROCEDURE_AMBIGUITY', count: 2 });
    expect(bodyFor('/api/simulations/agents/red-team')).toMatchObject({ runId: 'sim-run-1' });
    expect(bodyFor('/api/agents/tasks/task-1/transition')).toMatchObject({ status: 'ACKNOWLEDGED', actor: 'planner' });
    expect(bodyFor('/api/agents/airspace/tasks/task-1/acknowledge')).toMatchObject({ actor: 'planner', note: 'ack' });
    expect(bodyFor('/api/agents/airspace/tasks/task-1/resolve')).toMatchObject({ actor: 'planner', note: 'done' });
    expect(bodyFor('/api/providers/weather/poll')).toMatchObject({ products: ['metar'], maxResults: 5 });
    expect(bodyFor('/api/calibration/run')).toMatchObject({ datasetId: 'fixture-low-vis-weather', includeSyntheticScale: true });
    expect(bodyFor('/api/collaboration/proposals')).toMatchObject({ missionId: 'mission-1', recommendedAction: 'REROUTE' });
    expect(bodyFor('/api/collaboration/proposals/proposal-1/comment')).toMatchObject({ actor: 'dispatcher', note: 'accept if approved' });
    expect(bodyFor('/api/collaboration/proposals/proposal-1/deliver')).toMatchObject({ deliveryChannel: 'TELECON', externalReceiptId: 'cop-77' });
    expect(bodyFor('/api/coordination/draft-1/approve')).toMatchObject({ actor: 'supervisor' });
    expect(bodyFor('/api/coordination/draft-1/mark-delivered')).toMatchObject({ deliveryChannel: 'PHONE', externalReceiptId: 'ops-log-1' });
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
