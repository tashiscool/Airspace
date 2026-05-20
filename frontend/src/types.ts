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
