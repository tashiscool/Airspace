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
  humanReviewMode?: HumanReviewMode;
  humanReviewReason?: string;
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
  humanReviewMode?: HumanReviewMode;
  humanReviewReason?: string;
  citations?: AgentSourceCitation[];
};

export type AgentEvidenceReceipt = {
  id?: string;
  sourceFamily?: string;
  sourceId?: string;
  label?: string;
  route?: string;
  receiptHash?: string;
};

export type AgentCostBudget = {
  maxCostUsd?: number;
  estimatedCostUsd?: number;
  timeoutMillis?: number;
  retryCap?: number;
  circuitBreakerArmed?: boolean;
  fallbackMode?: string;
};

export type AgentWorkloadDefinition = {
  id: string;
  label?: string;
  category?: string;
  gapCoverage?: string;
  description?: string;
  defaultTrigger?: string;
  humanApprovalRequired?: boolean;
  externalSendAllowed?: boolean;
  policyGuards?: string[];
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
  stabilityAccepted?: boolean;
  sourceFamilyCounts?: Record<string, number>;
  stabilityWarnings?: string[];
  warnings?: string[];
  errors?: string[];
};

export type HumanReviewMode = 'NONE' | 'REVIEW_ONLY' | 'PULL_CLARIFICATION' | 'PUSH_APPROVAL' | string;

export type AgentAssessment = {
  id: string;
  schemaVersion?: string;
  agentType?: string;
  claim?: string;
  verdict?: string;
  confidence?: number;
  evidence?: string[];
  counterEvidence?: string[];
  uncertainty?: string;
  requiredHumanAction?: string;
  humanReviewMode?: HumanReviewMode;
  citations?: AgentSourceCitation[];
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
  availableTools?: string[];
  blockedTools?: string[];
  toolPolicySummary?: string;
  toolReceiptIds?: string[];
};

export type AgentToolCall = {
  id?: string;
  toolName?: string;
  serverId?: string;
  sideEffectLevel?: string;
  status?: string;
  startedAt?: string;
  completedAt?: string;
  arguments?: Record<string, unknown>;
  resultSummary?: string;
  evidenceReceiptId?: string;
  inputHash?: string;
  outputHash?: string;
  redactionStatus?: string;
  policyDecision?: string;
  durationMillis?: number;
  sourceRefs?: AgentSourceCitation[];
  diagnostics?: string[];
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
  scenarioId?: string;
  runId?: string;
  campaignId?: string;
  historicalReplayDayId?: string;
  missionId?: string;
  reservationId?: string;
  decisionId?: string;
  previousDecisionId?: string;
  hazardOrDecisionId?: string;
  question?: string;
  actor?: string;
  feedArtifactId?: string;
  referenceType?: string;
  referenceIdentifier?: string;
  toolCalls?: AgentToolCall[];
};

export type AgenticRiskProfile = {
  autonomyScope?: string;
  toolSurface?: string;
  worstCaseBlastRadius?: string;
  permissionScope?: string;
  dataEgress?: string;
  poisonedDataExposure?: string;
  auditAttribution?: string;
  toolProvenance?: string;
  modelProvenance?: string;
  rollbackPath?: string;
  costClass?: string;
  slaClass?: string;
  requiredHumanReviewMode?: HumanReviewMode;
};

export type AgenticRiskAssessment = {
  id: string;
  subjectType?: string;
  subjectId?: string;
  summary?: string;
  riskProfile?: AgenticRiskProfile;
  diagnostics?: string[];
};

export type AgentStabilityMetric = {
  id: string;
  name?: string;
  value?: number;
  threshold?: number;
  accepted?: boolean;
  details?: string;
};

export type AgentStabilityRequest = {
  agentRunRequest?: AgentRunRequest;
  iterations?: number;
  minAgreement?: number;
  minCitationJaccard?: number;
  maxCountCoefficientOfVariation?: number;
};

export type AgentStabilityResult = {
  id: string;
  accepted?: boolean;
  iterations?: number;
  startedAt?: string;
  completedAt?: string;
  runIds?: string[];
  metrics?: AgentStabilityMetric[];
  diagnostics?: string[];
};

export type McpServerDefinition = {
  id: string;
  name?: string;
  description?: string;
  enabled?: boolean;
  external?: boolean;
  local?: boolean;
  setupRequired?: boolean;
  credentialsRequired?: boolean;
  transport?: string;
  version?: string;
  diagnostics?: string[];
};

export type McpToolDescriptor = {
  id: string;
  serverId?: string;
  name?: string;
  description?: string;
  version?: string;
  enabled?: boolean;
  external?: boolean;
  setupRequired?: boolean;
  credentialsRequired?: boolean;
  sideEffectLevel?: 'READ_ONLY' | 'DRAFT_ONLY' | 'REQUIRES_APPROVAL' | 'MUTATING' | string;
  riskProfile?: AgenticRiskProfile;
  requiredArguments?: string[];
  argumentSchema?: Record<string, unknown>;
  sourceFamilies?: string[];
  diagnostics?: string[];
};

export type McpToolInvocationRequest = {
  serverId?: string;
  toolId?: string;
  actor?: string;
  externalConsent?: boolean;
  arguments?: Record<string, unknown>;
};

export type McpEvidenceReceipt = {
  id: string;
  serverId?: string;
  toolId?: string;
  sideEffectLevel?: string;
  status?: string;
  policyDecision?: string;
  redactionStatus?: string;
  inputHash?: string;
  outputHash?: string;
  inputSummary?: string;
  outputSummary?: string;
  startedAt?: string;
  completedAt?: string;
  durationMillis?: number;
  sourceRefs?: AgentSourceCitation[];
  diagnostics?: string[];
};

export type McpToolInvocationResult = {
  id?: string;
  serverId?: string;
  toolId?: string;
  status?: string;
  policyDecision?: string;
  startedAt?: string;
  completedAt?: string;
  result?: unknown;
  resultSummary?: string;
  evidenceReceipt?: McpEvidenceReceipt;
  toolCall?: AgentToolCall;
  sourceRefs?: AgentSourceCitation[];
  diagnostics?: string[];
};

export type AgentJobRequest = {
  idempotencyKey?: string;
  actor?: string;
  agentRunRequest?: AgentRunRequest;
  toolArguments?: Record<string, Record<string, unknown>>;
};

export type AgentJobResult = {
  id: string;
  status?: 'QUEUED' | 'RUNNING' | 'SUCCEEDED' | 'FAILED' | 'DENIED' | 'CANCELLED' | string;
  request?: AgentJobRequest;
  runResult?: AgentRunResult;
  createdAt?: string;
  startedAt?: string;
  completedAt?: string;
  toolCalls?: AgentToolCall[];
  receipts?: McpEvidenceReceipt[];
  diagnostics?: string[];
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
  humanApprovalRequired?: boolean;
  externalSendPerformed?: boolean;
  officialStateMutationPerformed?: boolean;
  generatedAt?: string;
  costBudget?: AgentCostBudget;
  auditEnvelope?: AgentAuditEnvelope;
  evaluation?: AgentEvaluationSummary;
  reasoningEnvelope?: AgentReasoningEnvelope;
  traceAnswer?: AgentTraceAnswer;
  findings: AgentFinding[];
  recommendations: AgentRecommendation[];
  tasks: AgentTask[];
  assessments?: AgentAssessment[];
  toolCalls?: AgentToolCall[];
  citations: AgentSourceCitation[];
  deltas?: AgentOperationalDelta[];
  operatingLoop?: AgentOperatingLoopStep[];
  evidenceReceipts?: AgentEvidenceReceipt[];
  policyGuards?: string[];
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

export type AirspaceGapStatus = {
  id: string;
  category: string;
  status: string;
  summary?: string;
  evidence?: string;
  nextStep?: string;
  externallyBlocked: boolean;
  certificationClaimAllowed: boolean;
  requiredFor: string[];
  sourceRefs: string[];
  diagnostics: string[];
};

export type ReleaseGateSummary = {
  id: string;
  label?: string;
  status: string;
  passed: boolean;
  summary?: string;
  requiredTests: string[];
  requiredDocs: string[];
  excludedClaims: string[];
  knownNonGoals: string[];
  blockingGapIds: string[];
};

export type ProviderFreshnessStatusSummary = {
  status?: string;
  lastPollAt?: string;
  ageSeconds?: number;
  pollIntervalSeconds?: number;
  stale: boolean;
  rationale?: string;
};

export type ProviderHealthSummary = {
  id: string;
  label?: string;
  providerType?: string;
  sourceMode?: string;
  credentialRequirement?: string;
  consentScope?: string;
  egressPolicy?: string;
  enabled: boolean;
  authoritative: boolean;
  liveOperationalUseAllowed: boolean;
  endpoint?: string;
  freshness?: ProviderFreshnessStatusSummary;
  diagnostics: string[];
};

export type CalibrationRunSummary = {
  id: string;
  accepted: boolean;
  generatedAt?: string;
  datasetId?: string;
  actor?: string;
  routeImpactReport?: RouteImpactCalibrationReport;
  diagnostics: string[];
};

export type RouteImpactCalibrationReport = {
  id: string;
  calibrationVersion?: string;
  datasetId?: string;
  routeOutcomeCount: number;
  weatherOutcomeCount: number;
  pirepOutcomeCount: number;
  sectorCapacityOutcomeCount: number;
  uncalibratedCoefficientCount: number;
  deterministicAgreementRate: number;
  averageProbabilityError: number;
  summary?: string;
  uncalibratedCoefficients: string[];
  recommendations: string[];
  diagnostics: string[];
};

export type SafetyCaseDossierSummary = {
  id: string;
  generatedAt?: string;
  releaseGate?: string;
  certificationClaimAllowed: boolean;
  summary?: string;
  scenariosTested: string[];
  sourcesUsed: string[];
  knownGaps: string[];
  rejectedOverclaims: string[];
  replayHashes: string[];
  coverageSummaries: string[];
  humanReviewCheckpoints: string[];
};

export type CoordinationDeliveryStatusSummary = {
  draftId: string;
  status: string;
  actor?: string;
  updatedAt?: string;
  deliveryChannel?: string;
  externalReceiptId?: string;
  humanApproved: boolean;
  externalSendPerformed: boolean;
  rationale?: string;
  diagnostics: string[];
};

export type CollaborativeParticipantSummary = {
  participantId?: string;
  displayName?: string;
  organization?: string;
  role?: string;
  facility?: string;
  channel?: string;
  canApprove: boolean;
  canDeliver: boolean;
  active: boolean;
};

export type CollaborativeProposalRequest = {
  missionId?: string;
  reservationId?: string;
  hazardOrDecisionId?: string;
  proposalType?: string;
  recommendedAction?: string;
  summary?: string;
  rationale?: string;
  actor?: string;
  role?: string;
  sourceRefs?: string[];
  recipientParticipantIds?: string[];
};

export type CollaborativeProposalActionRequest = {
  actor?: string;
  role?: string;
  note?: string;
  deliveryChannel?: string;
  externalReceiptId?: string;
  recipient?: string;
};

export type CollaborativeCommentSummary = {
  id?: string;
  proposalId?: string;
  actor?: string;
  role?: string;
  text?: string;
  createdAt?: string;
  sourceRefs: string[];
};

export type CollaborativeApprovalSummary = {
  id?: string;
  proposalId?: string;
  actor?: string;
  role?: string;
  status?: string;
  note?: string;
  updatedAt?: string;
  humanApproved: boolean;
};

export type CollaborativeDeliveryReceiptSummary = {
  id?: string;
  proposalId?: string;
  channel?: string;
  recipient?: string;
  status?: string;
  externalReceiptId?: string;
  deliveredAt?: string;
  actor?: string;
  externalSendPerformed: boolean;
  diagnostics: string[];
};

export type CollaborativeProposalSummary = {
  id?: string;
  state?: string;
  proposalType?: string;
  missionId?: string;
  reservationId?: string;
  hazardOrDecisionId?: string;
  recommendedAction?: string;
  summary?: string;
  rationale?: string;
  proposer?: string;
  proposerRole?: string;
  createdAt?: string;
  updatedAt?: string;
  commonOperatingPictureVersion?: string;
  humanApprovalRequired: boolean;
  recipientParticipantIds: string[];
  sourceRefs: string[];
  comments: CollaborativeCommentSummary[];
  approvals: CollaborativeApprovalSummary[];
  deliveryReceipts: CollaborativeDeliveryReceiptSummary[];
  diagnostics: string[];
};

export type CommonOperatingPictureSummary = {
  id?: string;
  version?: string;
  generatedAt?: string;
  syncStatus?: string;
  sourceMode?: string;
  activeMissionCount: number;
  affectedMissionCount: number;
  providerCount: number;
  staleProviderCount: number;
  activeProposalCount: number;
  pendingApprovalCount: number;
  deliveredReceiptCount: number;
  participants: CollaborativeParticipantSummary[];
  proposals: CollaborativeProposalSummary[];
  sourceRefs: string[];
  diagnostics: string[];
};

export type SimulationEvent = {
  id: string;
  offsetMinutes: number;
  family: string;
  eventType?: string;
  label?: string;
  payload?: string;
  expectedAction?: string;
  sourceRefs: string[];
};

export type SimulationScenario = {
  id: string;
  name: string;
  capabilityStory?: string;
  narrative?: string;
  missionNumber?: string;
  carfAltrv?: string;
  expectedFinalAction?: string;
  expectedGuidance?: string;
  route: number[][];
  events: SimulationEvent[];
  expectedSourceFamilies: string[];
  sensitivityDefaults: Record<string, number>;
};

export type SimulationKpiSummary = {
  timeToGuidanceSeconds: number;
  falseClearCount: number;
  falseBlockCount: number;
  sourceRefPreservationRate: number;
  rerouteFoundRate: number;
  staleDataDowngradeCount: number;
  coordinationDraftLatencySeconds: number;
  pilotBriefAvailable: boolean;
  replayVerificationPassRate: number;
  minuteStepCount: number;
  aircraftStateUpdateCount: number;
  peakSectorWorkloadRatio: number;
  maxHandoffDelaySeconds: number;
  maxSurfaceDelaySeconds: number;
  stochasticEnsembleMemberCount: number;
  behaviorDecisionCount: number;
  trafficReplayAircraftCount: number;
  nationalFlightCount: number;
  overloadedAirportCount: number;
  overloadedSectorCount: number;
  peakAirportDemandCapacityRatio: number;
  peakNationalSectorDemandCapacityRatio: number;
  nationalTmiRecommendationCount: number;
  simulatedDayCount: number;
  baselineDelayMinutes: number;
  mitigatedDelayMinutes: number;
  delayMinutesSaved: number;
  rerouteMiles: number;
  additionalFuelPounds: number;
  holdingFuelSavedPounds: number;
  fuelImpactPounds: number;
  sectorOverloadAvoidedCount: number;
  sourceRefCompletenessRate: number;
  operatorTimeToDecisionSeconds: number;
  diagnostics: string[];
};

export type SimulationAircraftState = {
  aircraftId?: string;
  latitude: number;
  longitude: number;
  altitudeFeet: number;
  groundSpeedKnots: number;
  climbRateFeetPerMinute: number;
  fuelRemainingPounds: number;
  estimatedFuelBurnPounds: number;
  runwayRequiredFeet: number;
  runwayAvailableFeet: number;
  takeoffPerformanceAcceptable: boolean;
  performancePhase?: string;
  rationale?: string;
};

export type SimulationAirportSurfaceState = {
  airportId?: string;
  runwayVisualRangeFeet: number;
  runwayId?: string;
  runwayLengthFeet: number;
  brakingAction?: string;
  smgcsActive: boolean;
  lowVisibilityProcedureActive: boolean;
  terminologyAmbiguity: boolean;
  departureQueueDepth: number;
  surfaceDelaySeconds: number;
  rationale?: string;
};

export type SimulationSectorWorkloadState = {
  sectorId?: string;
  activeAircraft: number;
  baselineCapacity: number;
  controllerPositionsStaffed: number;
  handoffQueueDepth: number;
  estimatedHandoffDelaySeconds: number;
  frequencyCongestion: number;
  workloadRatio: number;
  capacityState?: string;
  rationale?: string;
};

export type SimulationPilotOperatorState = {
  behaviorModel?: string;
  pilotAction?: string;
  controllerAction?: string;
  operatorAction?: string;
  humanApprovalRequired: boolean;
  communicationDelaySeconds: number;
  acceptanceProbability: number;
  rationale?: string;
};

export type SimulationWeatherEvolutionState = {
  evolutionModel?: string;
  ensembleMemberCount: number;
  forecastHour: number;
  movementSpeedKnots: number;
  growthRate: number;
  decayRate: number;
  meanBlockedProbability: number;
  probabilitySpread: number;
  stormPhase?: string;
  rationale?: string;
};

export type SimulationTrafficReplayState = {
  sourceMode?: string;
  replaySourceId?: string;
  providerFamily?: string;
  providerReceiptId?: string;
  authorizationMode?: string;
  replayedAircraftCount: number;
  replayedFlightPlanCount: number;
  replayedPositionCount: number;
  airportDemandSnapshotCount: number;
  sectorDemandSnapshotCount: number;
  activeTrafficManagementInitiativeCount: number;
  activeTmiRecommendationCount: number;
  activeTmiTypes: string[];
  liveSwimNasDataUsed: boolean;
  fixtureBacked: boolean;
  rationale?: string;
};

export type SimulationDynamicsSnapshot = {
  minute: number;
  aircraft?: SimulationAircraftState;
  airportSurface?: SimulationAirportSurfaceState;
  sectorWorkload?: SimulationSectorWorkloadState;
  pilotOperator?: SimulationPilotOperatorState;
  weatherEvolution?: SimulationWeatherEvolutionState;
  trafficReplay?: SimulationTrafficReplayState;
  nationalDemandCapacity?: NationalDemandCapacitySnapshot;
  trafficManagementRecommendations: TmiRecommendationModel[];
  assumptions: string[];
};

export type AircraftClass = 'HEAVY_JET' | 'NARROWBODY' | 'REGIONAL_JET' | 'TURBOPROP' | 'MILITARY_REFUELING' | 'GENERIC';

export type AircraftPerformanceProfile = {
  aircraftClass?: AircraftClass;
  cruiseSpeedKnots: number;
  climbRateFeetPerMinute: number;
  descentRateFeetPerMinute: number;
  fuelBurnPoundsPerMinute: number;
  takeoffDistanceFeet: number;
  landingDistanceFeet: number;
  minimumRvrFeet: number;
  brakingActionPenaltyFactor: number;
  assumptions?: string;
};

export type AircraftTrajectoryState = {
  latitude: number;
  longitude: number;
  altitudeFeet: number;
  groundSpeedKnots: number;
  routeProgress: number;
  delaySeconds: number;
  phase?: string;
};

export type FlightPlanIntent = {
  origin?: string;
  destination?: string;
  requestedAltitudeBlock?: string;
  route: number[][];
  missionId?: string;
  reservationId?: string;
  rationale?: string;
};

export type SimulatedAircraft = {
  id?: string;
  callsign?: string;
  aircraftClass?: AircraftClass;
  performanceProfile?: AircraftPerformanceProfile;
  flightPlan?: FlightPlanIntent;
  trajectory?: AircraftTrajectoryState;
  rerouteAssignment?: string;
  impacted: boolean;
  impactedSourceRefs: string[];
};

export type TrafficFlowScenario = {
  id?: string;
  sourceMode?: string;
  aircraft: SimulatedAircraft[];
  assumptions?: string;
};

export type TrafficManagementInitiativeType =
  | 'GDP'
  | 'AFP'
  | 'FEA'
  | 'FCA'
  | 'MILES_IN_TRAIL'
  | 'MINUTES_IN_TRAIL'
  | 'REROUTE_ADVISORY'
  | 'REQUIRED_REROUTE'
  | 'GROUND_STOP'
  | 'DEPARTURE_METERING'
  | 'ARRIVAL_RATE'
  | 'SECTOR_CAPACITY'
  | 'CTOP'
  | 'LEVEL_CAPPING'
  | 'FIX_BALANCING'
  | 'AIRBORNE_HOLDING'
  | 'LOCAL_REVIEW'
  | 'UNKNOWN';

export type TrafficFlowProgramModel = {
  programId?: string;
  programType?: TrafficManagementInitiativeType;
  controlledResourceType?: string;
  controlledResourceId?: string;
  targetAirport?: string;
  flowAreaId?: string;
  startOffsetMinutes: number;
  endOffsetMinutes: number;
  acceptanceRatePerHour: number;
  arrivalRatePerHour: number;
  departureRatePerHour: number;
  edctWindowMinutes: number;
  delayAssignmentPolicy?: string;
  affectedFlightIds: string[];
  sourceRefs: string[];
};

export type FlowEvaluationAreaModel = {
  areaId?: string;
  areaType?: TrafficManagementInitiativeType;
  name?: string;
  geometryType?: string;
  geometry: number[][];
  sectorIds: string[];
  fixIds: string[];
  routeFilters: string[];
  lowerAltitudeFeet: number;
  upperAltitudeFeet: number;
  startOffsetMinutes: number;
  endOffsetMinutes: number;
  acceptanceRatePerHour: number;
  controlArea: boolean;
  rationale?: string;
  sourceRefs: string[];
};

export type MilesInTrailRestrictionModel = {
  restrictionId?: string;
  milesInTrail: number;
  minutesInTrail: number;
  fixId?: string;
  sectorId?: string;
  routeId?: string;
  destination?: string;
  altitudeStratum?: string;
  startOffsetMinutes: number;
  endOffsetMinutes: number;
  affectedFlightIds: string[];
  sourceRefs: string[];
};

export type RerouteAdvisoryModel = {
  advisoryId?: string;
  advisoryType?: string;
  required: boolean;
  routeName?: string;
  routeText?: string;
  routePoints: number[][];
  reason?: string;
  startOffsetMinutes: number;
  endOffsetMinutes: number;
  affectedFlightIds: string[];
  sourceRefs: string[];
};

export type GroundStopModel = {
  groundStopId?: string;
  scope?: string;
  airportId?: string;
  airspaceId?: string;
  equipmentCriteria?: string;
  reason?: string;
  startOffsetMinutes: number;
  endOffsetMinutes: number;
  partial: boolean;
  affectedFlightIds: string[];
  sourceRefs: string[];
};

export type DepartureMeteringModel = {
  meteringId?: string;
  commonPoint?: string;
  intervalSeconds: number;
  releaseRatePerHour: number;
  startOffsetMinutes: number;
  endOffsetMinutes: number;
  departureAirports: string[];
  affectedFlightIds: string[];
  sourceRefs: string[];
};

export type ArrivalRateModel = {
  rateId?: string;
  airportId?: string;
  runwayConfiguration?: string;
  acceptanceRatePerHour: number;
  demandRatePerHour: number;
  startOffsetMinutes: number;
  endOffsetMinutes: number;
  reason?: string;
  sourceRefs: string[];
};

export type SectorCapacityModel = {
  capacityId?: string;
  sectorId?: string;
  baselineCapacity: number;
  reducedCapacity: number;
  demandCount: number;
  startOffsetMinutes: number;
  endOffsetMinutes: number;
  capacityState?: string;
  reason?: string;
  sourceRefs: string[];
};

export type TmiRecommendationModel = {
  id?: string;
  recommendedType?: TrafficManagementInitiativeType;
  action?: string;
  targetResourceType?: string;
  targetResourceId?: string;
  trigger?: string;
  severity?: string;
  expectedDelayMinutes: number;
  confidence: number;
  rationale?: string;
  sourceTmiIds: string[];
  affectedFlightIds: string[];
  sourceRefs: string[];
};

export type TrafficReplayFlightPlan = {
  flightId?: string;
  callsign?: string;
  aircraftClass?: AircraftClass;
  operator?: string;
  origin?: string;
  destination?: string;
  filedRouteText?: string;
  filedRoutePoints: number[][];
  scheduledDepartureTime?: string;
  scheduledArrivalTime?: string;
  estimatedDepartureTime?: string;
  estimatedArrivalTime?: string;
  actualDepartureTime?: string;
  actualArrivalTime?: string;
  requestedAltitudeFeet?: number;
  requestedAltitudeBlock?: string;
  filedSpeedKnots?: number;
  missionId?: string;
  reservationId?: string;
  sourceRefs: string[];
};

export type TrafficReplayPosition = {
  flightId?: string;
  timestamp?: string;
  offsetMinutes: number;
  latitude: number;
  longitude: number;
  altitudeFeet: number;
  groundSpeedKnots: number;
  headingDegrees: number;
  routeProgress: number;
  delaySeconds: number;
  phase?: string;
  sourceRefs: string[];
};

export type TrafficReplayAirportDemand = {
  airportId?: string;
  timestamp?: string;
  offsetMinutes: number;
  departureDemandPerHour: number;
  arrivalDemandPerHour: number;
  departureCapacityPerHour: number;
  arrivalCapacityPerHour: number;
  departureQueueDepth: number;
  arrivalQueueDepth: number;
  averageDelaySeconds: number;
  runwayConfiguration?: string;
  sourceRefs: string[];
};

export type TrafficReplaySectorDemand = {
  sectorId?: string;
  timestamp?: string;
  offsetMinutes: number;
  activeAircraft: number;
  baselineCapacity: number;
  handoffQueueDepth: number;
  frequencyUtilization: number;
  estimatedHandoffDelaySeconds: number;
  sourceRefs: string[];
};

export type TrafficManagementInitiative = {
  id?: string;
  type?: string;
  primitiveType?: TrafficManagementInitiativeType;
  status?: string;
  scope?: string;
  targetResourceId?: string;
  reason?: string;
  constraintId?: string;
  startOffsetMinutes: number;
  endOffsetMinutes: number;
  expectedDelayMinutes: number;
  confidence: number;
  flowArea?: FlowEvaluationAreaModel;
  flowProgram?: TrafficFlowProgramModel;
  milesInTrail?: MilesInTrailRestrictionModel;
  rerouteAdvisory?: RerouteAdvisoryModel;
  groundStop?: GroundStopModel;
  departureMetering?: DepartureMeteringModel;
  arrivalRate?: ArrivalRateModel;
  sectorCapacity?: SectorCapacityModel;
  recommendation?: TmiRecommendationModel;
  affectedFlightIds: string[];
  sourceRefs: string[];
};

export type TrafficReplayBundle = {
  id?: string;
  sourceId?: string;
  sourceMode?: string;
  providerFamily?: string;
  generatedAt?: string;
  timeBasis?: string;
  providerReceiptId?: string;
  authorizationMode?: string;
  sourceRefs: string[];
  flightPlans: TrafficReplayFlightPlan[];
  positions: TrafficReplayPosition[];
  airportDemand: TrafficReplayAirportDemand[];
  sectorDemand: TrafficReplaySectorDemand[];
  trafficManagementInitiatives: TrafficManagementInitiative[];
  assumptions: string[];
  diagnostics: string[];
};

export type TrafficReplayValidationResult = {
  replayId?: string;
  sourceMode?: string;
  accepted: boolean;
  flightPlanCount: number;
  positionCount: number;
  airportDemandSnapshotCount: number;
  sectorDemandSnapshotCount: number;
  trafficManagementInitiativeCount: number;
  diagnostics: string[];
};

export type HistoricalReplayOutcome = {
  id?: string;
  outcomeType?: string;
  label?: string;
  expectedAction?: string;
  observedAction?: string;
  targetResourceType?: string;
  targetResourceId?: string;
  offsetMinutes: number;
  expectedDelayMinutes: number;
  expectedConfidence: number;
  routeBlocked: boolean;
  rerouteExpected: boolean;
  staleDataExpected: boolean;
  capacityCompressionExpected: boolean;
  falseClearLabel: boolean;
  falseBlockLabel: boolean;
  sourceRefs: string[];
  diagnostics: string[];
};

export type HistoricalReplayDay = {
  id?: string;
  name?: string;
  operatingDate?: string;
  scenarioId?: string;
  sourceMode?: string;
  sourceFamily?: string;
  authorizationMode?: string;
  providerFamily?: string;
  sourceVersion?: string;
  expectedFinalAction?: string;
  trafficReplay?: TrafficReplayBundle;
  airportIds: string[];
  sectorIds: string[];
  tags: string[];
  publicSourceRefs: string[];
  providerReceiptIds: string[];
  dataQualityWarnings: string[];
  assumptions: string[];
  expectedOutcomes: HistoricalReplayOutcome[];
  expectedSummary: Record<string, string>;
};

export type HistoricalReplayLoadRequest = {
  dayId?: string;
  scenarioId?: string;
  actor?: string;
  strictValidation?: boolean;
  runSimulation?: boolean;
  includeCalibrationReport?: boolean;
  trafficReplay?: TrafficReplayBundle;
};

export type HistoricalReplayCalibrationReport = {
  id?: string;
  generatedAt?: string;
  calibrationVersion?: string;
  corpusDayCount: number;
  expectedOutcomeCount: number;
  routeOutcomeCount: number;
  weatherOutcomeCount: number;
  pirepOutcomeCount: number;
  notamOutcomeCount: number;
  sectorCapacityOutcomeCount: number;
  falseClearLabelCount: number;
  falseBlockLabelCount: number;
  uncalibratedCoefficientCount: number;
  deterministicAgreementRate: number;
  sourceRefPreservationRate: number;
  sourceModes: string[];
  sourceRefs: string[];
  uncalibratedCoefficients: string[];
  diagnostics: string[];
  outcomeCountsByType: Record<string, number>;
};

export type HistoricalReplayLoadResult = {
  dayId?: string;
  replayId?: string;
  sourceMode?: string;
  authorizationMode?: string;
  accepted: boolean;
  ranSimulation: boolean;
  runId?: string;
  finalAction?: string;
  flightPlanCount: number;
  positionCount: number;
  airportDemandSnapshotCount: number;
  sectorDemandSnapshotCount: number;
  tmiCount: number;
  expectedOutcomeCount: number;
  calibrationReport?: HistoricalReplayCalibrationReport;
  diagnostics: string[];
  warnings: string[];
};

export type NationalDemandCapacityConfig = {
  id?: string;
  flightCount?: number;
  airportCount?: number;
  sectorCount?: number;
  durationMinutes?: number;
  tickIntervalMinutes?: number;
  randomSeed?: number;
  demandSpikeFactor?: number;
  capacityReductionFactor?: number;
  includeWeatherCapacityReduction?: boolean;
  airportIds?: string[];
  sectorIds?: string[];
  sourceMode?: string;
};

export type NationalDemandCapacitySnapshot = {
  minute: number;
  totalFlightCount: number;
  activeFlightCount: number;
  airportCount: number;
  sectorCount: number;
  totalAirportDepartureDemandPerHour: number;
  totalAirportArrivalDemandPerHour: number;
  totalAirportDepartureCapacityPerHour: number;
  totalAirportArrivalCapacityPerHour: number;
  overloadedAirportCount: number;
  overloadedSectorCount: number;
  maxAirportDemandCapacityRatio: number;
  maxSectorDemandCapacityRatio: number;
  busiestAirportId?: string;
  busiestSectorId?: string;
  totalExpectedDelayMinutes: number;
  generatedTmiRecommendationCount: number;
  recommendations: TmiRecommendationModel[];
  diagnostics: string[];
};

export type NationalDemandCapacityReport = {
  id?: string;
  sourceMode?: string;
  flightCount: number;
  airportCount: number;
  sectorCount: number;
  durationMinutes: number;
  tickIntervalMinutes: number;
  peakAirportDemandCapacityRatio: number;
  peakSectorDemandCapacityRatio: number;
  peakOverloadedAirportCount: number;
  peakOverloadedSectorCount: number;
  totalTmiRecommendationCount: number;
  trafficReplay?: TrafficReplayBundle;
  snapshots: NationalDemandCapacitySnapshot[];
  assumptions: string[];
  diagnostics: string[];
};

export type TfmCommandCenterRequest = {
  demandCapacityConfig?: NationalDemandCapacityConfig;
  focusMinute?: number;
  maxAirportRows?: number;
  maxSectorRows?: number;
  maxConstraintRows?: number;
  maxProposalRows?: number;
  maxRouteAlternatives?: number;
};

export type TfmAirportDemandSummary = {
  airportId?: string;
  offsetMinutes: number;
  timestamp?: string;
  departureDemandPerHour: number;
  arrivalDemandPerHour: number;
  departureCapacityPerHour: number;
  arrivalCapacityPerHour: number;
  demandCapacityRatio: number;
  departureQueueDepth: number;
  arrivalQueueDepth: number;
  averageDelayMinutes: number;
  runwayConfiguration?: string;
  status?: string;
  sourceRefs: string[];
};

export type TfmSectorLoadSummary = {
  sectorId?: string;
  offsetMinutes: number;
  timestamp?: string;
  activeAircraft: number;
  baselineCapacity: number;
  workloadRatio: number;
  handoffQueueDepth: number;
  frequencyUtilization: number;
  estimatedHandoffDelaySeconds: number;
  status?: string;
  sourceRefs: string[];
};

export type TfmActiveConstraintSummary = {
  id?: string;
  type?: string;
  status?: string;
  scope?: string;
  targetResourceId?: string;
  reason?: string;
  startOffsetMinutes: number;
  endOffsetMinutes: number;
  expectedDelayMinutes: number;
  confidence: number;
  affectedFlightCount: number;
  affectedFlightIds: string[];
  sourceRefs: string[];
};

export type TfmProposedTmiSummary = {
  id?: string;
  type?: string;
  action?: string;
  targetResourceType?: string;
  targetResourceId?: string;
  status?: string;
  severity?: string;
  expectedDelayMinutes: number;
  confidence: number;
  affectedFlightCount: number;
  trigger?: string;
  rationale?: string;
  requiresHumanApproval: boolean;
  affectedFlightIds: string[];
  sourceTmiIds: string[];
  sourceRefs: string[];
};

export type TfmRouteAlternativeSummary = {
  id?: string;
  routeName?: string;
  routeText?: string;
  advisoryType?: string;
  required: boolean;
  targetResourceId?: string;
  reason?: string;
  startOffsetMinutes: number;
  endOffsetMinutes: number;
  expectedDelayMinutes: number;
  confidence: number;
  affectedFlightCount: number;
  residualRisk?: string;
  routePoints: number[][];
  affectedFlightIds: string[];
  sourceRefs: string[];
};

export type TfmImpactTotals = {
  flightCount: number;
  activeFlightCount: number;
  airportCount: number;
  sectorCount: number;
  overloadedAirportCount: number;
  overloadedSectorCount: number;
  maxAirportDemandCapacityRatio: number;
  maxSectorDemandCapacityRatio: number;
  totalExpectedDelayMinutes: number;
  activeConstraintCount: number;
  proposedTmiCount: number;
  routeAlternativeCount: number;
  affectedFlightCount: number;
  humanApprovalRequired: boolean;
  sourceMode?: string;
  sourceFreshnessStatus?: string;
  commonOperatingPictureStatus?: string;
};

export type TfmCommandCenterSummary = {
  id?: string;
  generatedAt?: string;
  boardMode?: string;
  sourceMode?: string;
  authorizationMode?: string;
  selectedMinute: number;
  demandCapacityReport?: NationalDemandCapacityReport;
  selectedSnapshot?: NationalDemandCapacitySnapshot;
  impactTotals: TfmImpactTotals;
  airportDemand: TfmAirportDemandSummary[];
  sectorLoad: TfmSectorLoadSummary[];
  activeConstraints: TfmActiveConstraintSummary[];
  proposedTmis: TfmProposedTmiSummary[];
  routeAlternatives: TfmRouteAlternativeSummary[];
  humanFactorsNotes: string[];
  assumptions: string[];
  diagnostics: string[];
  sourceRefs: string[];
};

export type OutcomeMetricsRequest = {
  runId?: string;
  campaignId?: string;
  scenarioId?: string;
  runSimulation?: boolean;
  simulationRunRequest?: {
    scenarioId?: string;
    actor?: string;
    includeSensitivity?: boolean;
    sensitivityOverrides?: Record<string, number>;
    tickIntervalSeconds?: number;
    durationMinutes?: number;
    randomSeed?: number;
    nationalDemandCapacityConfig?: NationalDemandCapacityConfig;
  };
  demandCapacityConfig?: NationalDemandCapacityConfig;
  includeTfmBoard?: boolean;
};

export type OutcomeMetricSummary = {
  id?: string;
  label?: string;
  value: number;
  unit?: string;
  status?: string;
  rationale?: string;
  sourceRefs: string[];
};

export type OutcomeMetricsReport = {
  id?: string;
  generatedAt?: string;
  scope?: string;
  sourceMode?: string;
  runId?: string;
  campaignId?: string;
  scenarioId?: string;
  baselineDelayMinutes: number;
  mitigatedDelayMinutes: number;
  delayMinutesSaved: number;
  rerouteMiles: number;
  additionalFuelPounds: number;
  holdingFuelSavedPounds: number;
  fuelImpactPounds: number;
  sectorOverloadAvoidedCount: number;
  falseClearCount: number;
  falseBlockCount: number;
  sourceRefCompletenessRate: number;
  operatorTimeToDecisionSeconds: number;
  rerouteCandidateCount: number;
  routeAlternativeCount: number;
  affectedFlightCount: number;
  overloadedAirportCount: number;
  overloadedSectorCount: number;
  proposedTmiCount: number;
  metrics: OutcomeMetricSummary[];
  sourceRefs: string[];
  assumptions: string[];
  diagnostics: string[];
};

export type RunwayOpsState = {
  airportId?: string;
  runwayId?: string;
  runwayLengthFeet: number;
  runwayVisualRangeFeet: number;
  brakingAction?: string;
  runwayClosed: boolean;
  smgcsActive: boolean;
  lowVisibilityProcedureActive: boolean;
  departureRatePerHour: number;
  arrivalRatePerHour: number;
  queueDepth: number;
  surfaceDelaySeconds: number;
};

export type SurfaceProcedureState = {
  procedureFamily?: string;
  active: boolean;
  terminologyAmbiguity: boolean;
  localProcedureName?: string;
  requestedPhraseology?: string;
  confirmationStatus?: string;
  rationale?: string;
};

export type AirportOpsTimeline = {
  airportId?: string;
  sourceMode?: string;
  runwayStates: RunwayOpsState[];
  surfaceProcedures: SurfaceProcedureState[];
  assumptions?: string;
};

export type WeatherEnsembleConfig = {
  memberCount: number;
  randomSeed: number;
  movementSpeedKnots: number;
  forecastConfidence: number;
  growthRate: number;
  decayRate: number;
  assumptions?: string;
};

export type WeatherEnsembleMember = {
  id?: string;
  latitudeOffset: number;
  longitudeOffset: number;
  blockedProbability: number;
  confidence: number;
  stormPhase?: string;
};

export type WeatherEvolutionTick = {
  minute: number;
  config?: WeatherEnsembleConfig;
  members: WeatherEnsembleMember[];
  meanBlockedProbability: number;
  probabilitySpread: number;
};

export type ControllerPositionState = {
  positionId?: string;
  role?: string;
  staffed: boolean;
  activeAircraft: number;
  handoffQueueDepth: number;
  coordinationDelaySeconds: number;
};

export type FrequencyCongestionState = {
  frequencyId?: string;
  utilization: number;
  blockedTransmissions: number;
  averageWaitSeconds: number;
  congestionState?: string;
};

export type SectorWorkloadModel = {
  sectorId?: string;
  activeAircraft: number;
  baselineCapacity: number;
  workloadRatio: number;
  capacityState?: string;
  controllerPositions: ControllerPositionState[];
  frequencyCongestion?: FrequencyCongestionState;
};

export type BehaviorStateMachine = {
  actor?: string;
  currentState?: string;
  nextAction?: string;
  humanApprovalRequired: boolean;
  communicationDelaySeconds: number;
  acceptanceProbability: number;
  transitionHistory: string[];
};

export type SimulationTick = {
  id?: string;
  minute: number;
  simulatedTime?: string;
  engineAction?: string;
  recommendedAction?: string;
  confidence: number;
  aircraft: SimulatedAircraft[];
  airportOps?: AirportOpsTimeline;
  sectorWorkload?: SectorWorkloadModel;
  nationalDemandCapacity?: NationalDemandCapacitySnapshot;
  weatherEvolution?: WeatherEvolutionTick;
  behaviorStates: BehaviorStateMachine[];
  sourceRefs: string[];
  replayHashes: Record<string, string>;
};

export type SimulationWorldState = {
  runId?: string;
  scenarioId?: string;
  tickIntervalSeconds: number;
  durationMinutes: number;
  randomSeed: number;
  generatedAt?: string;
  nationalDemandCapacityReport?: NationalDemandCapacityReport;
  ticks: SimulationTick[];
  assumptions: string[];
};

export type SimulationStepResult = {
  id: string;
  offsetMinutes: number;
  simulatedTime?: string;
  injectedEvent: SimulationEvent;
  engineAction: string;
  recommendedAction?: string;
  confidence: number;
  missionVerdict?: MissionWeatherVerdictSummary;
  routeImpact?: RouteImpactSummary;
  coordinationDraft?: CoordinationDraftSummary;
  pilotBrief?: PilotBriefSummary;
  features?: FeatureCollection;
  dynamics?: SimulationDynamicsSnapshot;
  affectedMissionDeltas: string[];
  routeImpactDeltas: string[];
  sourceRefs: string[];
  replayAuditIds: Record<string, string>;
  diagnostics: string[];
};

export type CampaignKpiGate = {
  metric?: string;
  operator?: string;
  threshold: number;
  blocking: boolean;
  rationale?: string;
};

export type ScenarioBundle = {
  id?: string;
  name?: string;
  airportId?: string;
  region?: string;
  useCase?: string;
  scenario?: SimulationScenario;
  trafficFlow?: TrafficFlowScenario;
  trafficReplay?: TrafficReplayBundle;
  airportOps?: AirportOpsTimeline;
  weatherEnsembleConfig?: WeatherEnsembleConfig;
  kpiGates: CampaignKpiGate[];
  expectedSummary: Record<string, string>;
};

export type ScenarioValidationResult = {
  accepted: boolean;
  scenarioId?: string;
  warnings: string[];
  errors: string[];
};

export type SimulationReplayBundle = {
  id?: string;
  runId?: string;
  scenarioId?: string;
  finalAction?: string;
  generatedAt?: string;
  worldState?: SimulationWorldState;
  steps: SimulationStepResult[];
  replayHashes: Record<string, string>;
  diagnostics: string[];
};

export type SafetyDossierExport = {
  id?: string;
  campaignId?: string;
  generatedAt?: string;
  format?: string;
  markdown?: string;
  jsonSummary: Record<string, unknown>;
  scenarios: string[];
  assumptions: string[];
  knownGaps: string[];
  nonCertificationWarnings: string[];
  replayHashes: string[];
};

export type SimulationAgentRequest = {
  scenarioType?: string;
  missionNumber?: string;
  runId?: string;
  campaignId?: string;
  includeMalformedInputs?: boolean;
  count?: number;
  focusAreas?: string[];
};

export type SimulationAgentReport = {
  id?: string;
  agentType?: string;
  generatedAt?: string;
  generatedScenarioDrafts: ScenarioBundle[];
  findings: string[];
  citations: string[];
  policyGuards: string[];
};

export type SimulationRunResult = {
  id: string;
  scenarioId: string;
  scenarioName?: string;
  missionId?: string;
  reservationId?: string;
  missionNumber?: string;
  startedAt?: string;
  completedAt?: string;
  finalAction: string;
  expectedFinalAction?: string;
  kpiSummary: SimulationKpiSummary;
  steps: SimulationStepResult[];
  sensitivity: Record<string, number>;
  worldState?: SimulationWorldState;
  replayBundle?: SimulationReplayBundle;
  nationalDemandCapacityReport?: NationalDemandCapacityReport;
  diagnostics: string[];
};

export type SimulationCampaignReport = {
  id: string;
  generatedAt?: string;
  scenarioCount: number;
  passedScenarioCount: number;
  aggregateKpis: SimulationKpiSummary;
  runs: SimulationRunResult[];
  diagnostics: string[];
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
