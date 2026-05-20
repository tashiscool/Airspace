import type {
  DecisionSummary,
  FeedArtifactSummary,
  FeedTransactionSummary,
  FeatureCollection,
  HistoryEventSummary,
  MessageSummary,
  MissionDetail,
  MissionSummary,
  ReferencePointSummary,
  ReferenceDataImportResult,
  ReservationSupplementSummary,
  ReplayVerificationResult,
  SearchResultSummary,
  UserSummary
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
    throw new Error(`${response.status} ${response.statusText}`);
  }
  if (response.status === 204) {
    return undefined as T;
  }
  return response.json() as Promise<T>;
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
  evaluateDecision: (body: { rawUsnsMessages?: string[]; rawCarfMessages?: string[]; route?: number[][]; decisionTime?: string }) =>
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
  config: () => request<Record<string, unknown>>('/api/config')
};
