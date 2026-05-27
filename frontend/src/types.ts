export type UserSummary = {
  id: string;
  username: string;
  displayName: string;
  roles: string[];
};

export type MissionSummary = {
  id: string;
  missionNumber: string;
  title?: string;
  status: string;
  lockedBy?: string;
  lockedAt?: string;
  reservationCount: number;
  updatedAt?: string;
};

export type ReservationSummary = {
  id: string;
  missionId: string;
  state: string;
  rawText?: string;
  lockedBy?: string;
  conflictCount: number;
  diagnostics: string[];
};

export type ReservationSupplementSummary = {
  id: string;
  reservationId: string;
  kind: string;
  status: string;
  title?: string;
  text?: string;
  actor?: string;
  updatedAt?: string;
};

export type ReferenceDataImportResult = {
  accepted: boolean;
  parsedCount: number;
  appliedCount: number;
  warnings: string[];
  errors: string[];
  points: ReferencePointSummary[];
};

export type MessageSummary = {
  id: string;
  missionId?: string;
  reservationId?: string;
  family: string;
  direction: string;
  status: string;
  subject?: string;
  rawText?: string;
  createdAt?: string;
};

export type WeatherSourceSummary = {
  id: string;
  family: string;
  label?: string;
  route?: string;
  severity?: string;
  rationale?: string;
  observedAt?: string;
  ageMinutes?: number;
  agingCategory?: string;
  relevanceScore?: number;
  stale: boolean;
};

export type MissionWeatherVerdictSummary = {
  missionId: string;
  action: string;
  priority: string;
  confidence: number;
  sourceCount: number;
  stale: boolean;
  summary: string;
  recommendedAction: string;
  sources: WeatherSourceSummary[];
  diagnostics: string[];
};

export type RouteImpactSummary = {
  missionId: string;
  reservationId?: string;
  action: string;
  recommendedAction?: string;
  confidence: number;
  rationale: string;
  originalRouteDistanceNm?: number;
  originalRouteEstimatedMinutes?: number;
  originalRouteEstimatedFuelLb?: number;
  originalRouteEstimatedCostUsd?: number;
  impactedSegmentCount: number;
  blockingConstraintCount: number;
  impactedSegments: string[];
  sourceRefs: string[];
  avoidanceCandidates: string[];
  candidateComparisons?: RouteCandidateComparisonSummary[];
  whyRerouteTrace?: RerouteTraceSummary[];
  diagnostics: string[];
};

export type RouteCandidateComparisonSummary = {
  id: string;
  label?: string;
  rationale?: string;
  confidence?: number;
  cost?: RouteCostEstimateSummary;
  routePointLabels?: string[];
  avoidedConstraints?: ConstraintImpactSummary[];
  residualConstraints?: ConstraintImpactSummary[];
  sourceRefs?: string[];
  trace?: RerouteTraceSummary[];
};

export type RouteCostEstimateSummary = {
  distanceNm?: number;
  additionalDistanceNm?: number;
  estimatedMinutes?: number;
  additionalMinutes?: number;
  estimatedFuelLb?: number;
  additionalFuelLb?: number;
  estimatedCostUsd?: number;
  additionalCostUsd?: number;
  cruiseSpeedKnots?: number;
  fuelBurnLbPerNm?: number;
  fuelCostUsdPerLb?: number;
  delayCostUsdPerMinute?: number;
};

export type ConstraintImpactSummary = {
  id: string;
  family?: string;
  label?: string;
  severity?: string;
  sourceRef?: string;
  rationale?: string;
};

export type RerouteTraceSummary = {
  stage?: string;
  ruleId?: string;
  message?: string;
  sourceRef?: string;
  confidence?: number;
};

export type PirepRelevanceResult = {
  missionId: string;
  totalPireps: number;
  relevantCount: number;
  staleCount: number;
  averageRelevanceScore: number;
  altitudeToleranceFeet: number;
  recencyMinutes: number;
  corridorNauticalMiles: number;
  relevant: WeatherSourceSummary[];
  excluded: WeatherSourceSummary[];
};

export type PirepRelevanceRequest = {
  reservationId?: string;
  lowerAltitudeFeet?: number;
  upperAltitudeFeet?: number;
  altitudeToleranceFeet?: number;
  recencyMinutes?: number;
  corridorNauticalMiles?: number;
  route?: number[][];
};

export type CoordinationDraftSummary = {
  id: string;
  missionId: string;
  reservationId?: string;
  subject: string;
  family: string;
  direction: string;
  rawText: string;
  recommendedAction: string;
  recipients: string[];
  sourceRefs: string[];
};

export type PilotBriefSummary = {
  missionId: string;
  missionNumber: string;
  generatedAt: string;
  verdict: MissionWeatherVerdictSummary;
  routeImpact: RouteImpactSummary;
  coordinationDraft: CoordinationDraftSummary;
  changes: WeatherSourceSummary[];
  sourceSummaryLines: string[];
  decisionTraceSummary: string;
  printableText: string;
};

export type AffectedMissionSummary = {
  missionId: string;
  missionNumber: string;
  status: string;
  action: string;
  priority: string;
  confidence: number;
  sourceCount: number;
  impactedSegmentCount: number;
  blockingConstraintCount: number;
  route?: string;
  rationale?: string;
  lastObservedAt?: string;
  ageSeconds: number;
  stale: boolean;
  guidanceLatencySeconds: number;
  guidanceTargetMet: boolean;
  rerouteCandidateCount?: number;
  bestCandidateLabel?: string;
  rerouteAdditionalDistanceNm?: number;
  rerouteAdditionalMinutes?: number;
  rerouteAdditionalFuelLb?: number;
  rerouteAdditionalCostUsd?: number;
  avoidedConstraintCount?: number;
  residualConstraintCount?: number;
  sourceRefs: string[];
  routeCoordinates?: number[][];
};

export type MissionDetail = {
  mission: MissionSummary;
  reservations: ReservationSummary[];
  messages: MessageSummary[];
  history: HistoryEventSummary[];
};

export type HistoryEventSummary = {
  id: string;
  aggregateType: string;
  aggregateId: string;
  eventType: string;
  actor?: string;
  note?: string;
  createdAt: string;
};

export type SearchResultSummary = {
  id: string;
  type: string;
  title: string;
  status?: string;
  snippet?: string;
  route?: string;
  updatedAt?: string;
};

export type ReferencePointSummary = {
  id: string;
  identifier: string;
  pointType: string;
  latitude: number;
  longitude: number;
  altitudeFeet?: number;
  source?: string;
  metadataJson?: string;
  updatedAt?: string;
};

export type DecisionSummary = {
  id: string;
  action: string;
  recommendedAction?: string;
  confidence: number;
  rationale?: string;
  routeImpact?: RouteImpactSummary;
  resultJson?: string;
  auditJson?: string;
  replayJson?: string;
  result?: unknown;
};

export type ReplayVerificationResult = {
  accepted: boolean;
  warnings: string[];
  errors: string[];
  result?: unknown;
};

export type AgentSourceCitation = {
  sourceFamily?: string;
  sourceId?: string;
  label?: string;
  route?: string;
  ruleId?: string;
  sourceSpan?: string;
};

export type AgentFinding = {
  id: string;
  category?: string;
  severity?: string;
  message?: string;
  confidence?: number;
  citations?: AgentSourceCitation[];
};

export type AgentRecommendation = {
  id: string;
  action?: string;
  summary?: string;
  rationale?: string;
  confidence?: number;
  humanApprovalRequired?: boolean;
  citations?: AgentSourceCitation[];
};

export type AgentTask = {
  id: string;
  title?: string;
  status?: string;
  priority?: string;
  assignedRole?: string;
  route?: string;
  rationale?: string;
  citations?: AgentSourceCitation[];
};

export type AgentAuditEnvelope = {
  id?: string;
  agentType?: string;
  agentVersion?: string;
  policyVersion?: string;
  generatedAt?: string;
  inputHash?: string;
  outputHash?: string;
  citations?: AgentSourceCitation[];
  diagnostics?: string[];
};

export type AgentEvaluationSummary = {
  accepted?: boolean;
  findingCount?: number;
  recommendationCount?: number;
  taskCount?: number;
  deltaCount?: number;
  citedClaimCount?: number;
  uncitedClaimCount?: number;
  policyViolationCount?: number;
  citationCoverage?: number;
  sourceFamilyCounts?: Record<string, number>;
  warnings?: string[];
  errors?: string[];
};

export type AgentReasoningEnvelope = {
  id?: string;
  promptVersion?: string;
  modelId?: string;
  reasoningMode?: string;
  inputSummary?: string;
  draftHash?: string;
  generatedAt?: string;
  allowedFacts?: string[];
  requiredOutputRules?: string[];
  prohibitedActions?: string[];
  citations?: AgentSourceCitation[];
};

export type AgentTraceAnswer = {
  question?: string;
  answer?: string;
  confidence?: number;
  ruleIds?: string[];
  sourceRefs?: string[];
  unsupportedClaims?: string[];
  citations?: AgentSourceCitation[];
};

export type AgentOperatingLoopStep = {
  stage?: string;
  status?: string;
  summary?: string;
  citations?: AgentSourceCitation[];
};

export type AgentOperationalDelta = {
  id: string;
  changeType?: string;
  sourceFamily?: string;
  sourceId?: string;
  previousValue?: string;
  currentValue?: string;
  severity?: string;
  observedAt?: string;
  citations?: AgentSourceCitation[];
};

export type AgentRunRequest = {
  agentType?: string;
  missionId?: string;
  reservationId?: string;
  decisionId?: string;
  previousDecisionId?: string;
  hazardOrDecisionId?: string;
  question?: string;
  actor?: string;
};

export type ScenarioFixtureRequest = {
  scenarioType?: string;
  missionNumber?: string;
  includeMalformedInputs?: boolean;
};

export type ScenarioFixtureBundle = {
  id: string;
  scenarioType?: string;
  missionNumber?: string;
  carfAltrv?: string;
  usnsMessages?: string[];
  weatherMessages?: string[];
  pireps?: string[];
  notams?: string[];
  route?: number[][];
  expectedSummary?: Record<string, string>;
};

export type AgentTaskTransitionRequest = {
  status?: string;
  actor?: string;
  note?: string;
};

export type AgentRunResult = {
  id: string;
  agentType: string;
  missionId?: string;
  reservationId?: string;
  decisionId?: string;
  summary?: string;
  confidence?: number;
  accepted: boolean;
  generatedAt?: string;
  auditEnvelope?: AgentAuditEnvelope;
  evaluation?: AgentEvaluationSummary;
  reasoningEnvelope?: AgentReasoningEnvelope;
  traceAnswer?: AgentTraceAnswer;
  findings: AgentFinding[];
  recommendations: AgentRecommendation[];
  tasks: AgentTask[];
  citations: AgentSourceCitation[];
  deltas?: AgentOperationalDelta[];
  operatingLoop?: AgentOperatingLoopStep[];
  diagnostics: string[];
};

export type AgentStoreStatus = {
  mode?: string;
  durable?: boolean;
  path?: string;
  runCount?: number;
  taskCount?: number;
  latestRunAt?: string;
};

export type FeedArtifactSummary = {
  id: string;
  sourceId: string;
  type: string;
  accepted: boolean;
  rawPayloadHash: string;
  rawPayload?: string;
  receivedAt?: string;
  downstreamArtifactIds: string[];
  diagnostics: string[];
};

export type FeedTransactionSummary = {
  id: string;
  type: string;
  status: string;
  supported: boolean;
  normalizedText?: string;
  notamType?: string;
  notamAccountability?: string;
  notamAffectedLocation?: string;
  notamQCode?: string;
  notamTraffic?: string;
  notamPurpose?: string;
  notamScope?: string;
  notamLowerFlightLevel?: string;
  notamUpperFlightLevel?: string;
  notamEstimatedEnd?: boolean;
  notamPermanentEnd?: boolean;
  notamHasGeometry?: boolean;
  notamCenterLatitude?: number;
  notamCenterLongitude?: number;
  notamRadiusNauticalMiles?: number;
  domesticNotamRecordType?: string;
  domesticNotamKeyword?: string;
  domesticNotamAccountability?: string;
  domesticNotamLocation?: string;
  domesticNotamNumber?: string;
  domesticNotamUnofficial?: boolean;
  domesticNotamQ23?: string;
  domesticNotamQ45?: string;
  domesticNotamSemanticFacilityFamily?: string;
  domesticNotamSemanticCondition?: string;
  domesticNotamSemanticAction?: string;
  domesticNotamReducerRuleId?: string;
  domesticNotamReducerName?: string;
  serviceCommandType?: string;
  serviceCommandService?: string;
  serviceCommandDomain?: string;
  serviceCommandOperation?: string;
  serviceCommandRequestFormat?: string;
  serviceCommandLocation?: string;
  serviceCommandNotamId?: string;
  serviceCommandRangeStart?: string;
  serviceCommandRangeEnd?: string;
  serviceCommandTableName?: string;
  serviceCommandAccepted?: boolean;
  serviceCommandCount?: boolean;
  serviceCommandList?: boolean;
  serviceCommandHistory?: boolean;
  serviceCommandAll?: boolean;
  serviceCommandCurrent?: boolean;
  serviceCommandWmscrEcho?: boolean;
  serviceCommandPrivilegedHistoryRequest?: boolean;
  familySemantic?: string;
  familyPreview?: string;
  familyLifecycle?: string;
  familyNotamAction?: string;
  familySnowtamId?: string;
  familyBirdtamId?: string;
  familyAshtamId?: string;
  familyGenotSeries?: string;
  familyFdcId?: string;
  warnings: string[];
  errors: string[];
};

export type WeatherLiveStatusSummary = {
  enabled: boolean;
  baseUrl?: string;
  pollIntervalSeconds: number;
  maxResults: number;
  userAgent?: string;
  lastPollAt?: string;
  patternCount: number;
  eventCount: number;
  diagnostics: string[];
};

export type WeatherLivePollSummary = {
  accepted: boolean;
  sourceId?: string;
  receivedAt?: string;
  envelopeCount: number;
  acceptedCount: number;
  patternCount: number;
  diagnostics: string[];
};

export type WeatherPatternSummary = {
  id: string;
  patternType?: string;
  productFamily?: string;
  sourceFamily?: string;
  sourceProductId?: string;
  sourceUrl?: string;
  geometryIntent?: string;
  issuedAt?: string;
  receivedAt?: string;
  validStart?: string;
  validEnd?: string;
  forecastHour?: number;
  lowerAltitudeFeet?: number;
  upperAltitudeFeet?: number;
  movementBearingDegrees?: number;
  movementSpeedKnots?: number;
  severity?: string;
  confidence: number;
  freshnessCategory?: string;
  rationale?: string;
  rawText?: string;
  geometry: number[][];
  sourceRefs: string[];
  diagnostics: string[];
};

export type WeatherEventSummary = {
  id: string;
  eventType?: string;
  label?: string;
  severity?: string;
  confidence: number;
  validStart?: string;
  validEnd?: string;
  affectedMissionCount: number;
  productCount: number;
  pirepCount: number;
  rationale?: string;
  sourceRefs: string[];
  patternIds: string[];
  representativeGeometry: number[][];
};

export type WeatherPatternRouteSampleRequest = {
  route: number[][];
  lowerAltitudeFeet?: number;
  upperAltitudeFeet?: number;
  corridorNauticalMiles?: number;
  startTime?: string;
  endTime?: string;
};

export type RouteWeatherPatternIntersectionSummary = {
  patternId: string;
  patternType?: string;
  severity?: string;
  confidence: number;
  timeOverlap: boolean;
  altitudeOverlap: boolean;
  geometryOverlap: boolean;
  segmentIndex: number;
  nearestDistanceNauticalMiles: number;
  rationale?: string;
  sourceRefs: string[];
};

export type FeatureCollection = {
  type: 'FeatureCollection';
  features: AirspaceFeature[];
};

export type AirspaceFeature = {
  id?: string;
  type?: string;
  geometry?: Record<string, unknown>;
  properties?: Record<string, unknown>;
};
