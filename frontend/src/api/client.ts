import type {
  AffectedMissionSummary,
  AirspaceGapStatus,
  AgentRunRequest,
  AgentRunResult,
  AgenticRiskAssessment,
  AgentJobRequest,
  AgentJobResult,
  AgentStabilityRequest,
  AgentStabilityResult,
  AgentStoreStatus,
  AgentTask,
  AgentTaskTransitionRequest,
  AgentWorkloadDefinition,
  AgentOperationalDelta,
  CalibrationRunSummary,
  CollaborativeParticipantSummary,
  CollaborativeProposalActionRequest,
  CollaborativeProposalRequest,
  CollaborativeProposalSummary,
  CommonOperatingPictureSummary,
  CoordinationDeliveryStatusSummary,
  DecisionSummary,
  FeedArtifactSummary,
  FeedTransactionSummary,
  FeatureCollection,
  HistoryEventSummary,
  HistoricalReplayCalibrationReport,
  HistoricalReplayDay,
  HistoricalReplayLoadRequest,
  HistoricalReplayLoadResult,
  MessageSummary,
  McpEvidenceReceipt,
  McpServerDefinition,
  McpToolDescriptor,
  McpToolInvocationRequest,
  McpToolInvocationResult,
  MissionDetail,
  MissionWeatherVerdictSummary,
  MissionSummary,
  NationalDemandCapacityConfig,
  NationalDemandCapacityReport,
  OutcomeMetricsReport,
  OutcomeMetricsRequest,
  PilotBriefSummary,
  PirepRelevanceRequest,
  PirepRelevanceResult,
  ReferencePointSummary,
  ReferenceDataImportResult,
  ReservationSupplementSummary,
  ReplayVerificationResult,
  RouteImpactSummary,
  SearchResultSummary,
  ScenarioFixtureBundle,
  ScenarioFixtureRequest,
  SafetyDossierExport,
  ScenarioBundle,
  ScenarioValidationResult,
  SimulationAgentReport,
  SimulationAgentRequest,
  SimulationCampaignReport,
  SimulationRunResult,
  SimulationScenario,
  SimulationStepResult,
  SimulationReplayBundle,
  SimulationWorldState,
  SimulatedAircraft,
  TrafficReplayBundle,
  TrafficReplayValidationResult,
  TfmCommandCenterRequest,
  TfmCommandCenterSummary,
  WeatherEnsembleConfig,
  SectorWorkloadModel,
  AirportOpsTimeline,
  ProviderHealthSummary,
  ReleaseGateSummary,
  SafetyCaseDossierSummary,
  CoordinationDraftSummary,
  UserSummary,
  WeatherSourceSummary,
  WeatherLiveStatusSummary,
  WeatherLivePollSummary,
  WeatherPatternSummary,
  WeatherEventSummary,
  WeatherPatternRouteSampleRequest,
  RouteWeatherPatternIntersectionSummary
} from '../types';

const API_BASE = import.meta.env.VITE_API_BASE ?? '';

async function request<T>(path: string, init?: RequestInit): Promise<T> {
  const token = localStorage.getItem('airspace.token');
  const response = await fetch(`${API_BASE}${path}`, {
    ...init,
    headers: {
      'Content-Type': 'application/json',
      ...(token ? { Authorization: `Bearer ${token}` } : {}),
      ...(init?.headers ?? {})
    }
  });
  if (!response.ok) {
    throw new Error(await responseErrorMessage(response));
  }
  if (response.status === 204) {
    return undefined as T;
  }
  return response.json() as Promise<T>;
}

async function responseErrorMessage(response: Response) {
  const prefix = `${response.status} ${response.statusText}`;
  const contentType = response.headers.get('content-type') ?? '';
  try {
    if (contentType.includes('application/json')) {
      const body = await response.json() as { message?: string; error?: string; errors?: string[]; diagnostics?: string[] };
      const detail = body.message ?? body.error ?? body.errors?.join(', ') ?? body.diagnostics?.join(', ');
      return detail ? `${prefix}: ${detail}` : prefix;
    }
    const text = await response.text();
    return text ? `${prefix}: ${text.slice(0, 500)}` : prefix;
  } catch {
    return prefix;
  }
}

export const api = {
  login: (username: string, password: string) =>
    request<{ accepted: boolean; token?: string; user?: UserSummary; diagnostics: string[] }>('/api/auth/login', {
      method: 'POST',
      body: JSON.stringify({ username, password })
    }),
  me: () => request<UserSummary | null>('/api/auth/me'),
  missions: () => request<MissionSummary[]>('/api/missions'),
  createMission: (body: { missionNumber?: string; title?: string; rawText?: string; actor?: string }) =>
    request<MissionSummary>('/api/missions', { method: 'POST', body: JSON.stringify(body) }),
  mission: (id: string) => request<MissionDetail>(`/api/missions/${id}`),
  missionWeatherVerdict: (id: string) => request<MissionWeatherVerdictSummary>(`/api/missions/${id}/weather-verdict`),
  missionWeatherChanges: (id: string, since?: string, limit?: number) => {
    const params = new URLSearchParams();
    if (since) params.set('since', since);
    if (limit) params.set('limit', String(limit));
    const query = params.toString();
    return request<WeatherSourceSummary[]>(`/api/missions/${id}/weather-changes${query ? `?${query}` : ''}`);
  },
  affectedMissions: (sourceId?: string, limit?: number) => {
    const params = new URLSearchParams();
    if (sourceId) params.set('sourceId', sourceId);
    if (limit) params.set('limit', String(limit));
    const query = params.toString();
    return request<AffectedMissionSummary[]>(`/api/weather/affected-missions${query ? `?${query}` : ''}`);
  },
  weatherLiveStatus: () => request<WeatherLiveStatusSummary>('/api/weather/live/status'),
  pollLiveWeather: (body: { products?: string[]; hoursBeforeNow?: number; maxResults?: number } = {}) =>
    request<WeatherLivePollSummary>('/api/weather/live/poll', { method: 'POST', body: JSON.stringify(body) }),
  weatherPatterns: () => request<WeatherPatternSummary[]>('/api/weather/patterns'),
  weatherPatternFeatures: () => request<FeatureCollection>('/api/weather/patterns/features'),
  weatherEvents: () => request<WeatherEventSummary[]>('/api/weather/events'),
  sampleWeatherRoute: (body: WeatherPatternRouteSampleRequest) =>
    request<RouteWeatherPatternIntersectionSummary[]>('/api/weather/route-sample', { method: 'POST', body: JSON.stringify(body) }),
  missionRouteImpact: (id: string, reservationId?: string) =>
    request<RouteImpactSummary>(`/api/missions/${id}/route-impact${reservationId ? `?reservationId=${encodeURIComponent(reservationId)}` : ''}`),
  relevantPireps: (id: string, body: PirepRelevanceRequest) =>
    request<PirepRelevanceResult>(`/api/missions/${id}/pireps/relevant`, { method: 'POST', body: JSON.stringify(body) }),
  coordinateWeather: (id: string, body: { hazardOrDecisionId?: string; missionId?: string; reservationId?: string; actor?: string }) =>
    request<CoordinationDraftSummary>(`/api/missions/${id}/coordinate-weather`, { method: 'POST', body: JSON.stringify(body) }),
  pilotBrief: (id: string, since?: string) =>
    request<PilotBriefSummary>(`/api/missions/${id}/pilot-brief${since ? `?since=${encodeURIComponent(since)}` : ''}`),
  lockMission: (id: string, actor: string) =>
    request<MissionSummary>(`/api/missions/${id}/lock`, { method: 'POST', body: JSON.stringify({ actor }) }),
  unlockMission: (id: string, actor: string) =>
    request<MissionSummary>(`/api/missions/${id}/unlock`, { method: 'POST', body: JSON.stringify({ actor }) }),
  createReservation: (missionId: string, rawText: string, actor: string) =>
    request(`/api/missions/${missionId}/reservations`, { method: 'POST', body: JSON.stringify({ rawText, actor }) }),
  updateReservation: (id: string, rawText: string, actor: string) =>
    request(`/api/reservations/${id}`, { method: 'PUT', body: JSON.stringify({ rawText, actor }) }),
  validateReservation: (id: string, actor: string) =>
    request(`/api/reservations/${id}/validate`, { method: 'POST', body: JSON.stringify({ actor }) }),
  parseReservation: (id: string, actor: string) =>
    request(`/api/reservations/${id}/parse`, { method: 'POST', body: JSON.stringify({ actor }) }),
  deconflictReservation: (id: string, actor: string) =>
    request(`/api/reservations/${id}/deconflict`, { method: 'POST', body: JSON.stringify({ actor }) }),
  forceParseReservation: (id: string, actor: string, note?: string) =>
    request(`/api/reservations/${id}/force-parse`, { method: 'POST', body: JSON.stringify({ actor, note }) }),
  forceDeconflictReservation: (id: string, actor: string, note?: string) =>
    request(`/api/reservations/${id}/force-deconflict`, { method: 'POST', body: JSON.stringify({ actor, note }) }),
  submitReservation: (id: string, actor: string) =>
    request(`/api/reservations/${id}/submit`, { method: 'POST', body: JSON.stringify({ actor }) }),
  approveReservation: (id: string, actor: string) =>
    request(`/api/reservations/${id}/approve`, { method: 'POST', body: JSON.stringify({ actor }) }),
  rejectReservation: (id: string, actor: string, note?: string) =>
    request(`/api/reservations/${id}/reject`, { method: 'POST', body: JSON.stringify({ actor, note }) }),
  cancelReservation: (id: string, actor: string, note?: string) =>
    request(`/api/reservations/${id}/cancel`, { method: 'POST', body: JSON.stringify({ actor, note }) }),
  completeReservation: (id: string, actor: string) =>
    request(`/api/reservations/${id}/complete`, { method: 'POST', body: JSON.stringify({ actor }) }),
  lockReservation: (id: string, actor: string) =>
    request(`/api/reservations/${id}/lock`, { method: 'POST', body: JSON.stringify({ actor }) }),
  unlockReservation: (id: string, actor: string) =>
    request(`/api/reservations/${id}/unlock`, { method: 'POST', body: JSON.stringify({ actor }) }),
  reservationFeatures: (id: string) => request<FeatureCollection>(`/api/reservations/${id}/features`),
  reservationSupplements: (id: string) => request<ReservationSupplementSummary[]>(`/api/reservations/${id}/supplements`),
  createReservationSupplement: (id: string, body: Partial<ReservationSupplementSummary> & { actor?: string }) =>
    request<ReservationSupplementSummary>(`/api/reservations/${id}/supplements`, { method: 'POST', body: JSON.stringify(body) }),
  transitionReservationSupplement: (reservationId: string, supplementId: string, body: { status: string; actor?: string; note?: string }) =>
    request<ReservationSupplementSummary>(`/api/reservations/${reservationId}/supplements/${supplementId}/transition`, {
      method: 'POST',
      body: JSON.stringify(body)
    }),
  messages: () => request<MessageSummary[]>('/api/messages'),
  message: (id: string) => request<MessageSummary>(`/api/messages/${id}`),
  sendMessage: (body: Partial<MessageSummary> & { actor?: string }) =>
    request<MessageSummary>('/api/messages/send', { method: 'POST', body: JSON.stringify(body) }),
  replyMessage: (id: string, body: Partial<MessageSummary> & { actor?: string }) =>
    request<MessageSummary>(`/api/messages/${id}/reply`, { method: 'POST', body: JSON.stringify(body) }),
  forwardMessage: (id: string, body: Partial<MessageSummary> & { actor?: string }) =>
    request<MessageSummary>(`/api/messages/${id}/forward`, { method: 'POST', body: JSON.stringify(body) }),
  ingestFeed: (body: { sourceId: string; type: string; rawPayload: string }) =>
    request('/api/feed/ingest', { method: 'POST', body: JSON.stringify(body) }),
  feedArtifacts: () => request<FeedArtifactSummary[]>('/api/feed/artifacts'),
  feedArtifact: (id: string) => request<FeedArtifactSummary>(`/api/feed/artifacts/${id}`),
  feedTransactions: (id: string) => request<FeedTransactionSummary[]>(`/api/feed/artifacts/${id}/transactions`),
  evaluateDecision: (body: { rawUsnsMessages?: string[]; rawCarfMessages?: string[]; route?: number[][]; missionId?: string; reservationId?: string; decisionTime?: string }) =>
    request<DecisionSummary>('/api/decisions/evaluate', { method: 'POST', body: JSON.stringify(body) }),
  decision: (id: string) => request<DecisionSummary>(`/api/decisions/${id}`),
  replayDecision: (id: string) =>
    request<ReplayVerificationResult>(`/api/decisions/${id}/replay`, { method: 'POST' }),
  decisionFeatures: (id: string) => request<FeatureCollection>(`/api/decisions/${id}/features`),
  history: () => request<HistoryEventSummary[]>('/api/history'),
  search: (query: string) => request<SearchResultSummary[]>(`/api/search?q=${encodeURIComponent(query)}`),
  referencePoints: (type?: string) => request<ReferencePointSummary[]>(`/api/reference/points${type ? `?type=${encodeURIComponent(type)}` : ''}`),
  createReferencePoint: (body: Partial<ReferencePointSummary>) =>
    request<ReferencePointSummary>('/api/reference/points', { method: 'POST', body: JSON.stringify(body) }),
  previewReferenceImport: (payload: string, actor = 'admin') =>
    request<ReferenceDataImportResult>('/api/reference/import/preview', { method: 'POST', body: JSON.stringify({ payload, actor }) }),
  applyReferenceImport: (payload: string, actor = 'admin') =>
    request<ReferenceDataImportResult>('/api/reference/import/apply', { method: 'POST', body: JSON.stringify({ payload, actor, apply: true }) }),
  metrics: () => request<Record<string, number>>('/api/metrics'),
  config: () => request<Record<string, unknown>>('/api/config'),
  gaps: () => request<AirspaceGapStatus[]>('/api/gaps'),
  releaseGates: () => request<ReleaseGateSummary[]>('/api/gaps/release-gates'),
  providersStatus: () => request<ProviderHealthSummary[]>('/api/providers/status'),
  pollProviderWeather: (body: { products?: string[]; hoursBeforeNow?: number; maxResults?: number } = {}) =>
    request<ProviderHealthSummary>('/api/providers/weather/poll', { method: 'POST', body: JSON.stringify(body) }),
  runCalibration: (body: { datasetId?: string; includeSyntheticScale?: boolean; actor?: string } = {}) =>
    request<CalibrationRunSummary>('/api/calibration/run', { method: 'POST', body: JSON.stringify(body) }),
  calibrationReports: () => request<CalibrationRunSummary[]>('/api/calibration/reports'),
  safetyDossier: () => request<SafetyCaseDossierSummary>('/api/safety/dossier'),
  commonOperatingPicture: () => request<CommonOperatingPictureSummary>('/api/collaboration/common-operating-picture'),
  collaborativeParticipants: () => request<CollaborativeParticipantSummary[]>('/api/collaboration/participants'),
  collaborativeProposals: () => request<CollaborativeProposalSummary[]>('/api/collaboration/proposals'),
  createCollaborativeProposal: (body: CollaborativeProposalRequest) =>
    request<CollaborativeProposalSummary>('/api/collaboration/proposals', { method: 'POST', body: JSON.stringify(body) }),
  commentOnCollaborativeProposal: (proposalId: string, body: CollaborativeProposalActionRequest) =>
    request<CollaborativeProposalSummary>(`/api/collaboration/proposals/${proposalId}/comment`, { method: 'POST', body: JSON.stringify(body) }),
  acceptCollaborativeProposal: (proposalId: string, body: CollaborativeProposalActionRequest = {}) =>
    request<CollaborativeProposalSummary>(`/api/collaboration/proposals/${proposalId}/accept`, { method: 'POST', body: JSON.stringify(body) }),
  rejectCollaborativeProposal: (proposalId: string, body: CollaborativeProposalActionRequest = {}) =>
    request<CollaborativeProposalSummary>(`/api/collaboration/proposals/${proposalId}/reject`, { method: 'POST', body: JSON.stringify(body) }),
  approveCollaborativeProposal: (proposalId: string, body: CollaborativeProposalActionRequest = {}) =>
    request<CollaborativeProposalSummary>(`/api/collaboration/proposals/${proposalId}/approve`, { method: 'POST', body: JSON.stringify(body) }),
  deliverCollaborativeProposal: (proposalId: string, body: CollaborativeProposalActionRequest = {}) =>
    request<CollaborativeProposalSummary>(`/api/collaboration/proposals/${proposalId}/deliver`, { method: 'POST', body: JSON.stringify(body) }),
  approveCoordination: (draftId: string, body: { actor?: string; note?: string; deliveryChannel?: string; externalReceiptId?: string } = {}) =>
    request<CoordinationDeliveryStatusSummary>(`/api/coordination/${draftId}/approve`, { method: 'POST', body: JSON.stringify(body) }),
  markCoordinationDelivered: (draftId: string, body: { actor?: string; note?: string; deliveryChannel?: string; externalReceiptId?: string } = {}) =>
    request<CoordinationDeliveryStatusSummary>(`/api/coordination/${draftId}/mark-delivered`, { method: 'POST', body: JSON.stringify(body) }),
  runAgent: (body: AgentRunRequest) =>
    request<AgentRunResult>('/api/agents/run', { method: 'POST', body: JSON.stringify(body) }),
  weatherImpactAgent: (body: AgentRunRequest = {}) =>
    request<AgentRunResult>('/api/agents/weather-impact', { method: 'POST', body: JSON.stringify(body) }),
  missionRiskAgent: (body: AgentRunRequest) =>
    request<AgentRunResult>('/api/agents/mission-risk', { method: 'POST', body: JSON.stringify(body) }),
  rerouteAnalysisAgent: (body: AgentRunRequest) =>
    request<AgentRunResult>('/api/agents/reroute-analysis', { method: 'POST', body: JSON.stringify(body) }),
  coordinationDraftAgent: (body: AgentRunRequest) =>
    request<AgentRunResult>('/api/agents/coordination-draft', { method: 'POST', body: JSON.stringify(body) }),
  pilotBriefAgent: (body: AgentRunRequest) =>
    request<AgentRunResult>('/api/agents/pilot-brief', { method: 'POST', body: JSON.stringify(body) }),
  dataIntegrityAgent: (body: AgentRunRequest) =>
    request<AgentRunResult>('/api/agents/data-integrity', { method: 'POST', body: JSON.stringify(body) }),
  replayAuditAgent: (body: AgentRunRequest) =>
    request<AgentRunResult>('/api/agents/replay-audit', { method: 'POST', body: JSON.stringify(body) }),
  safetyLabAgent: (body: AgentRunRequest = {}) =>
    request<AgentRunResult>('/api/agents/safety-lab', { method: 'POST', body: JSON.stringify(body) }),
  agentDelta: (body: AgentRunRequest) =>
    request<AgentOperationalDelta[]>('/api/agents/delta', { method: 'POST', body: JSON.stringify(body) }),
  generateAgentScenario: (body: ScenarioFixtureRequest) =>
    request<ScenarioFixtureBundle>('/api/agents/scenario/generate', { method: 'POST', body: JSON.stringify(body) }),
  evaluateAgentStability: (body: AgentStabilityRequest) =>
    request<AgentStabilityResult>('/api/agents/evaluate/stability', { method: 'POST', body: JSON.stringify(body) }),
  agentRiskAssessments: () => request<AgenticRiskAssessment[]>('/api/agents/risk-assessments'),
  mcpServers: () => request<McpServerDefinition[]>('/api/agents/mcp/servers'),
  mcpTools: (serverId: string) => request<McpToolDescriptor[]>(`/api/agents/mcp/servers/${encodeURIComponent(serverId)}/tools`),
  callMcpTool: (body: McpToolInvocationRequest) =>
    request<McpToolInvocationResult>('/api/agents/mcp/tools/call', { method: 'POST', body: JSON.stringify(body) }),
  mcpReceipts: (limit?: number) =>
    request<McpEvidenceReceipt[]>(`/api/agents/mcp/receipts${limit ? `?limit=${limit}` : ''}`),
  enqueueAgentJob: (body: AgentJobRequest) =>
    request<AgentJobResult>('/api/agents/jobs', { method: 'POST', body: JSON.stringify(body) }),
  agentJobs: (limit?: number) =>
    request<AgentJobResult[]>(`/api/agents/jobs${limit ? `?limit=${limit}` : ''}`),
  agentJob: (id: string) => request<AgentJobResult>(`/api/agents/jobs/${id}`),
  simulationScenarios: () => request<SimulationScenario[]>('/api/simulations/scenarios'),
  validateSimulationScenario: (body: ScenarioBundle) =>
    request<ScenarioValidationResult>('/api/simulations/scenarios/validate', { method: 'POST', body: JSON.stringify(body) }),
  importSimulationScenario: (body: ScenarioBundle) =>
    request<ScenarioBundle>('/api/simulations/scenarios/import', { method: 'POST', body: JSON.stringify(body) }),
  validateTrafficReplay: (body: TrafficReplayBundle) =>
    request<TrafficReplayValidationResult>('/api/simulations/traffic-replay/validate', { method: 'POST', body: JSON.stringify(body) }),
  historicalReplayDays: () => request<HistoricalReplayDay[]>('/api/simulations/historical-replay/days'),
  historicalReplayDay: (id: string) => request<HistoricalReplayDay>(`/api/simulations/historical-replay/days/${id}`),
  loadHistoricalReplay: (body: HistoricalReplayLoadRequest = {}) =>
    request<HistoricalReplayLoadResult>('/api/simulations/historical-replay/load', { method: 'POST', body: JSON.stringify(body) }),
  historicalReplayCalibration: (body: HistoricalReplayLoadRequest = {}) =>
    request<HistoricalReplayCalibrationReport>('/api/simulations/historical-replay/calibrate', { method: 'POST', body: JSON.stringify(body) }),
  previewNationalDemandCapacity: (body: NationalDemandCapacityConfig) =>
    request<NationalDemandCapacityReport>('/api/simulations/national-demand/preview', { method: 'POST', body: JSON.stringify(body) }),
  tfmBoard: (body: TfmCommandCenterRequest = {}) =>
    request<TfmCommandCenterSummary>('/api/tfm/board', { method: 'POST', body: JSON.stringify(body) }),
  outcomeMetrics: (body: OutcomeMetricsRequest = {}) =>
    request<OutcomeMetricsReport>('/api/outcomes/metrics', { method: 'POST', body: JSON.stringify(body) }),
  simulationScenarioBundle: (id: string) => request<ScenarioBundle>(`/api/simulations/scenarios/${id}/bundle`),
  runSimulation: (body: {
    scenarioId?: string;
    actor?: string;
    includeSensitivity?: boolean;
    sensitivityOverrides?: Record<string, number>;
    tickIntervalSeconds?: number;
    durationMinutes?: number;
    randomSeed?: number;
    aircraftFleet?: SimulatedAircraft[];
    trafficReplay?: TrafficReplayBundle;
    nationalDemandCapacityConfig?: NationalDemandCapacityConfig;
    weatherEnsembleConfig?: WeatherEnsembleConfig;
    sectorWorkloadModel?: SectorWorkloadModel;
    airportOpsTimeline?: AirportOpsTimeline;
  } = {}) =>
    request<SimulationRunResult>('/api/simulations/run', { method: 'POST', body: JSON.stringify(body) }),
  runSimulationCampaign: (body: { scenarioIds?: string[]; includeSensitivity?: boolean; actor?: string; iterationsPerScenario?: number; simulatedDayCount?: number; randomSeed?: number; kpiGates?: unknown[] } = {}) =>
    request<SimulationCampaignReport>('/api/simulations/campaign', { method: 'POST', body: JSON.stringify(body) }),
  simulationRun: (id: string) => request<SimulationRunResult>(`/api/simulations/runs/${id}`),
  simulationTimeline: (id: string) => request<SimulationStepResult[]>(`/api/simulations/runs/${id}/timeline`),
  simulationFeatures: (id: string) => request<FeatureCollection>(`/api/simulations/runs/${id}/features`),
  simulationWorldState: (id: string) => request<SimulationWorldState>(`/api/simulations/runs/${id}/world-state`),
  simulationReplay: (id: string) => request<SimulationReplayBundle>(`/api/simulations/runs/${id}/replay`),
  simulationCampaignReport: (id: string) => request<SimulationCampaignReport>(`/api/simulations/campaigns/${id}/report`),
  simulationCampaignDossier: (id: string) => request<SafetyDossierExport>(`/api/simulations/campaigns/${id}/dossier`),
  generateSimulationScenarios: (body: SimulationAgentRequest = {}) =>
    request<SimulationAgentReport>('/api/simulations/agents/generate-scenarios', { method: 'POST', body: JSON.stringify(body) }),
  redTeamSimulation: (body: SimulationAgentRequest = {}) =>
    request<SimulationAgentReport>('/api/simulations/agents/red-team', { method: 'POST', body: JSON.stringify(body) }),
  agentStatus: () => request<AgentStoreStatus>('/api/agents/status'),
  agentWorkloads: () => request<AgentWorkloadDefinition[]>('/api/agents/workloads'),
  agentMetrics: () => request<Record<string, number>>('/api/agents/metrics'),
  agentRuns: (limit?: number, filters?: { agentType?: string; missionId?: string; reservationId?: string; decisionId?: string; accepted?: boolean; sourceFamily?: string }) => {
    const params = new URLSearchParams();
    if (limit) params.set('limit', String(limit));
    if (filters?.agentType) params.set('agentType', filters.agentType);
    if (filters?.missionId) params.set('missionId', filters.missionId);
    if (filters?.reservationId) params.set('reservationId', filters.reservationId);
    if (filters?.decisionId) params.set('decisionId', filters.decisionId);
    if (filters?.accepted !== undefined) params.set('accepted', String(filters.accepted));
    if (filters?.sourceFamily) params.set('sourceFamily', filters.sourceFamily);
    const query = params.toString();
    return request<AgentRunResult[]>(`/api/agents/runs${query ? `?${query}` : ''}`);
  },
  agentRun: (id: string) => request<AgentRunResult>(`/api/agents/runs/${id}`),
  agentTasks: (status?: string, limit?: number, filters?: { priority?: string; assignedRole?: string; sourceFamily?: string; routeContains?: string }) => {
    const params = new URLSearchParams();
    if (status) params.set('status', status);
    if (limit) params.set('limit', String(limit));
    if (filters?.priority) params.set('priority', filters.priority);
    if (filters?.assignedRole) params.set('assignedRole', filters.assignedRole);
    if (filters?.sourceFamily) params.set('sourceFamily', filters.sourceFamily);
    if (filters?.routeContains) params.set('routeContains', filters.routeContains);
    const query = params.toString();
    return request<AgentTask[]>(`/api/agents/tasks${query ? `?${query}` : ''}`);
  },
  agentTask: (id: string) => request<AgentTask>(`/api/agents/tasks/${id}`),
  transitionAgentTask: (id: string, body: AgentTaskTransitionRequest) =>
    request<AgentTask>(`/api/agents/tasks/${id}/transition`, { method: 'POST', body: JSON.stringify(body) })
};
