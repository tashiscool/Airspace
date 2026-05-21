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
  impactedSegmentCount: number;
  blockingConstraintCount: number;
  impactedSegments: string[];
  sourceRefs: string[];
  avoidanceCandidates: string[];
  diagnostics: string[];
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
  sourceRefs: string[];
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
  warnings: string[];
  errors: string[];
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
