import { useEffect, useMemo, useState } from 'react';
import type { Dispatch, SetStateAction } from 'react';
import { useMutation, useQuery, useQueryClient } from '@tanstack/react-query';
import { createColumnHelper } from '@tanstack/react-table';
import { Link, useNavigate, useParams } from 'react-router-dom';
import { Lock, Map, Plus, Unlock } from 'lucide-react';
import { api } from '../api/client';
import { DataTable } from '../components/DataTable';
import { ErrorNotice, MutationNotice, QueryNotice } from '../components/Notices';
import { OperationsMap } from '../components/OperationsMap';
import { StatusBadge } from '../components/StatusBadge';
import { WeatherVerdictStrip } from '../components/WeatherVerdict';
import { compactId, fmtZ, missionWeatherVerdict, sourceRefLabel, sourceRefRoute, type MissionWeatherVerdict } from '../lib/viewModels';
import { writeWorkbenchJson, type WorkbenchSelection } from '../lib/workbenchState';
import type { MessageSummary, MissionWeatherVerdictSummary, ReservationSummary } from '../types';

const reservationColumn = createColumnHelper<ReservationSummary>();
const reservationColumns = [
  reservationColumn.accessor('id', { header: 'Reservation', cell: (info) => compactId(info.getValue()) }),
  reservationColumn.accessor('state', { header: 'State', cell: (info) => <StatusBadge value={info.getValue()} /> }),
  reservationColumn.accessor('conflictCount', { header: 'Conflicts' }),
  reservationColumn.accessor('lockedBy', { header: 'Locked By', cell: (info) => info.getValue() ?? '—' }),
  reservationColumn.accessor((row) => row.diagnostics.length, { id: 'diagnostics', header: 'Diagnostics' })
];

const messageColumn = createColumnHelper<MessageSummary>();
const messageColumns = [
  messageColumn.accessor('family', { header: 'Family' }),
  messageColumn.accessor('direction', { header: 'Direction' }),
  messageColumn.accessor('status', { header: 'Status', cell: (info) => <StatusBadge value={info.getValue()} /> }),
  messageColumn.accessor('subject', { header: 'Subject' })
];

const defaultRouteReservationText = `A. TEST02 KZNY
B. 1F22/I
C. KZNY
D. FL240B260 3000N 15000W 0000 3000N 15100W 0100
E. TEST02
F. ETD TEST02 021200 MAR 2010 AVANA 021300
G. TAS: 300 KTAS`;

type PirepScope = {
  lowerAltitudeFeet: number;
  upperAltitudeFeet: number;
  altitudeToleranceFeet: number;
  recencyMinutes: number;
  corridorNauticalMiles: number;
};

export function MissionPage() {
  const { missionId = '' } = useParams();
  const navigate = useNavigate();
  const queryClient = useQueryClient();
  const [selectedReservationId, setSelectedReservationId] = useState('');
  const [selectedMessageId, setSelectedMessageId] = useState('');
  const [pirepScope, setPirepScope] = useState({
    lowerAltitudeFeet: 22000,
    upperAltitudeFeet: 28000,
    altitudeToleranceFeet: 2000,
    recencyMinutes: 60,
    corridorNauticalMiles: 50
  });
  const mission = useQuery({ queryKey: ['mission', missionId], queryFn: () => api.mission(missionId), enabled: !!missionId });
  const selectedReservation = mission.data?.reservations.find((reservation) => reservation.id === selectedReservationId) ?? mission.data?.reservations[0];
  const selectedMessage = mission.data?.messages.find((message) => message.id === selectedMessageId) ?? mission.data?.messages[0];
  const messages = useQuery({ queryKey: ['messages', 'mission-weather'], queryFn: api.messages });
  const backendVerdict = useQuery({
    queryKey: ['mission-weather-verdict', missionId],
    queryFn: () => api.missionWeatherVerdict(missionId),
    enabled: !!missionId
  });
  const routeImpact = useQuery({
    queryKey: ['mission-route-impact', missionId, selectedReservationId],
    queryFn: () => api.missionRouteImpact(missionId, selectedReservationId || undefined),
    enabled: !!missionId
  });
  const pilotBrief = useQuery({
    queryKey: ['mission-pilot-brief', missionId],
    queryFn: () => api.pilotBrief(missionId),
    enabled: !!missionId
  });
  const weatherChanges = useQuery({
    queryKey: ['mission-weather-changes', missionId],
    queryFn: () => api.missionWeatherChanges(missionId, undefined, 8),
    enabled: !!missionId
  });
  const relevantPireps = useQuery({
    queryKey: ['mission-relevant-pireps', missionId, selectedReservation?.id, pirepScope],
    queryFn: () => api.relevantPireps(missionId, { ...pirepScope, reservationId: selectedReservation?.id }),
    enabled: !!missionId
  });
  const coordinationDraft = useMutation({
    mutationFn: () => api.coordinateWeather(missionId, { reservationId: selectedReservation?.id, actor: 'planner' })
  });
  const firstReservationId = mission.data?.reservations[0]?.id;
  const features = useQuery({
    queryKey: ['reservation-features', firstReservationId, 'mission-preview'],
    queryFn: () => api.reservationFeatures(firstReservationId!),
    enabled: !!firstReservationId
  });
  const createReservation = useMutation({
    mutationFn: () => api.createReservation(missionId, defaultRouteReservationText, 'planner'),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['mission', missionId] })
  });
  const lock = useMutation({ mutationFn: () => api.lockMission(missionId, 'planner'), onSuccess: () => queryClient.invalidateQueries({ queryKey: ['mission', missionId] }) });
  const unlock = useMutation({ mutationFn: () => api.unlockMission(missionId, 'planner'), onSuccess: () => queryClient.invalidateQueries({ queryKey: ['mission', missionId] }) });
  const conflictCount = useMemo(() => (mission.data?.reservations ?? []).reduce((sum, reservation) => sum + reservation.conflictCount, 0), [mission.data]);
  const weatherVerdict = useMemo(() => {
    return backendVerdict.data ? verdictFromBackend(backendVerdict.data) : missionWeatherVerdict(missionId, messages.data ?? mission.data?.messages ?? []);
  }, [backendVerdict.data, missionId, messages.data, mission.data?.messages]);
  useEffect(() => {
    if (!mission.data?.mission) return;
    const selection: WorkbenchSelection = {
      missionId,
      reservationId: selectedReservation?.id,
      messageId: selectedMessage?.id,
      sourceFamily: 'CARF_ALTRV',
      label: mission.data.mission.missionNumber,
      lockState: mission.data.mission.lockedBy ? `Locked by ${mission.data.mission.lockedBy}` : 'Unlocked',
      conflictCount
    };
    writeWorkbenchJson('airspace.workbench.selection', selection);
    window.dispatchEvent(new Event('airspace-workbench-selection'));
  }, [mission.data?.mission, missionId, selectedReservation?.id, selectedMessage?.id, conflictCount]);
  return (
    <section className="workspace operational-page">
      <div className="page-header">
        <div>
          <h2>{mission.data?.mission.missionNumber ?? 'Mission Workspace'}</h2>
          <p>{mission.data?.mission.title ?? 'Mission detail, reservations, messages, history, and map preview.'}</p>
        </div>
        {mission.data?.mission.status && <StatusBadge value={mission.data.mission.status} />}
      </div>
      <div className="toolbar wrap">
        <button onClick={() => lock.mutate()}><Lock size={14} /> Lock</button>
        <button className="secondary" onClick={() => unlock.mutate()}><Unlock size={14} /> Unlock</button>
        <button onClick={() => createReservation.mutate()}><Plus size={14} /> Add Reservation</button>
        {firstReservationId && <button className="secondary" onClick={() => navigate(`/deconfliction/${firstReservationId}`)}><Map size={14} /> Deconfliction</button>}
        <button className="secondary" onClick={() => navigate(`/missions/${missionId}/brief`)}>Pilot Brief</button>
      </div>
      <div className="notice-stack">
        <QueryNotice query={mission} label="Mission detail" />
        <QueryNotice query={messages} label="Weather verdict inputs" />
        <QueryNotice query={backendVerdict} label="Mission weather verdict" />
        <QueryNotice query={weatherChanges} label="Weather changes" />
        <QueryNotice query={routeImpact} label="Route impact" />
        <QueryNotice query={relevantPireps} label="Relevant PIREPs" />
        <ErrorNotice error={features.error} title="Mission map preview unavailable" />
        <MutationNotice mutation={lock} label="Lock mission" />
        <MutationNotice mutation={unlock} label="Unlock mission" />
        <MutationNotice mutation={createReservation} label="Create reservation" />
        <MutationNotice mutation={coordinationDraft} label="Coordinate weather" />
      </div>
      <section className="mission-summary-grid">
        <Metric label="Reservations" value={String(mission.data?.reservations.length ?? 0)} />
        <Metric label="Conflicts" value={String(conflictCount)} attention={conflictCount > 0} />
        <Metric label="Messages" value={String(mission.data?.messages.length ?? 0)} />
        <Metric label="Locked By" value={mission.data?.mission.lockedBy ?? 'Unlocked'} />
        <Metric label="Updated" value={fmtZ(mission.data?.mission.updatedAt)} />
      </section>
      <WeatherVerdictStrip verdict={weatherVerdict} missionId={missionId} />
      <div className="workspace-grid-main">
        <section className="panel">
          <div className="panel-heading">
            <h3>What Changed Since Last Brief</h3>
            <span>{weatherChanges.data?.length ?? 0}</span>
          </div>
          <div className="change-feed">
            {(weatherChanges.data?.length ? weatherChanges.data : pilotBrief.data?.changes ?? []).map((item) => (
              <button key={item.id} className="change-row" onClick={() => item.route && navigate(item.route)}>
                <StatusBadge value={item.severity ?? item.family} />
                <span>{item.family} · {item.label}</span>
                <small>{fmtZ(item.observedAt)} · {item.rationale}</small>
              </button>
            ))}
            {!weatherChanges.data?.length && !pilotBrief.data?.changes?.length && (
              <p className="muted">No weather, PIREP, or NOTAM deltas are currently linked to this mission.</p>
            )}
          </div>
        </section>
        <section className="panel">
          <div className="panel-heading">
            <h3>Route Impact & Avoidance</h3>
            <span>{routeImpact.data?.action ?? 'pending'}</span>
          </div>
          <div className="route-impact-card">
            <StatusBadge value={routeImpact.data?.action ?? 'MONITOR'} />
            <strong>{routeImpact.data?.recommendedAction ?? 'Evaluate selected reservation route'}</strong>
            <p>{routeImpact.data?.rationale ?? 'The workbench will evaluate weather, PIREPs, NOTAMs, and CARF constraints against the selected mission route.'}</p>
            <small>{routeImpact.data?.impactedSegmentCount ?? 0} impacted segment(s) · {routeImpact.data?.blockingConstraintCount ?? 0} blocking constraint(s) · {Math.round((routeImpact.data?.confidence ?? 0) * 100)}% confidence</small>
            {!!routeImpact.data?.impactedSegments?.length && (
              <div className="source-ref-grid">{routeImpact.data.impactedSegments.map((item) => <span key={item}>{item}</span>)}</div>
            )}
            {!!routeImpact.data?.sourceRefs?.length && (
              <div className="route-source-grid">
                {routeImpact.data.sourceRefs.map((ref) => (
                  <button
                    key={ref}
                    type="button"
                    className={`route-source route-source-${sourceRefLabel(ref).family.toLowerCase().replace(/[^a-z0-9]+/g, '-')}`}
                    disabled={!sourceRefRoute(ref)}
                    onClick={() => {
                      const route = sourceRefRoute(ref);
                      if (route) navigate(route);
                    }}
                  >
                    <strong>{sourceRefLabel(ref).family}</strong>
                    {sourceRefLabel(ref).id || ref}
                  </button>
                ))}
              </div>
            )}
            {!!routeImpact.data?.avoidanceCandidates?.length && (
              <div className="source-ref-grid">{routeImpact.data.avoidanceCandidates.map((item) => <span key={item}>{item}</span>)}</div>
            )}
            <button onClick={() => coordinationDraft.mutate()}>Coordinate From Hazard</button>
          </div>
          {coordinationDraft.data && (
            <>
              <pre className="raw-panel">{coordinationDraft.data.rawText}</pre>
              <button className="secondary" onClick={() => navigate(coordinationDraftHref(coordinationDraft.data))}>Open Draft In Messaging</button>
            </>
          )}
        </section>
        <section className="panel pilot-brief-panel">
          <div className="panel-heading"><h3>Pilot Brief</h3><span>{fmtZ(pilotBrief.data?.generatedAt)}</span></div>
          <pre className="raw-panel">{pilotBrief.data?.printableText ?? 'Generate a mission brief once weather, PIREPs, NOTAMs, and route impact inputs are available.'}</pre>
        </section>
        <section className="panel">
          <div className="panel-heading">
            <h3>Relevant PIREPs</h3>
            <span>{relevantPireps.data?.relevantCount ?? 0}/{relevantPireps.data?.totalPireps ?? 0} · {Math.round((relevantPireps.data?.averageRelevanceScore ?? 0) * 100)}% avg</span>
          </div>
          <div className="scope-controls" aria-label="PIREP relevance scope">
            <label>
              Lower FL
              <input
                type="number"
                value={Math.round(pirepScope.lowerAltitudeFeet / 100)}
                onChange={(event) => updatePirepScope(setPirepScope, 'lowerAltitudeFeet', Number(event.target.value) * 100)}
              />
            </label>
            <label>
              Upper FL
              <input
                type="number"
                value={Math.round(pirepScope.upperAltitudeFeet / 100)}
                onChange={(event) => updatePirepScope(setPirepScope, 'upperAltitudeFeet', Number(event.target.value) * 100)}
              />
            </label>
            <label>
              Tolerance ft
              <input
                type="number"
                step="500"
                min="0"
                value={pirepScope.altitudeToleranceFeet}
                onChange={(event) => updatePirepScope(setPirepScope, 'altitudeToleranceFeet', Number(event.target.value))}
              />
            </label>
            <label>
              Recency min
              <input
                type="number"
                min="5"
                value={pirepScope.recencyMinutes}
                onChange={(event) => updatePirepScope(setPirepScope, 'recencyMinutes', Number(event.target.value))}
              />
            </label>
            <label>
              Corridor NM
              <input
                type="number"
                min="1"
                value={pirepScope.corridorNauticalMiles}
                onChange={(event) => updatePirepScope(setPirepScope, 'corridorNauticalMiles', Number(event.target.value))}
              />
            </label>
          </div>
          <div className="source-ref-grid">
            {(relevantPireps.data?.relevant?.length ? relevantPireps.data.relevant : [{ id: 'empty', family: 'PIREP', label: 'No route/altitude/recency relevant PIREPs in current inputs.' }]).map((item) => (
              <span key={item.id}>{pirepSourceLabel(item)}</span>
            ))}
          </div>
          {!!relevantPireps.data?.excluded?.length && (
            <>
              <p className="muted">{relevantPireps.data.excluded.length} PIREP(s) excluded by route, altitude, recency, or age policy; {relevantPireps.data.staleCount} stale.</p>
              <div className="source-ref-grid decayed-source-grid">
                {relevantPireps.data.excluded.slice(0, 6).map((item) => (
                  <span key={`excluded-${item.id}`}>{pirepSourceLabel(item)}</span>
                ))}
              </div>
            </>
          )}
          <p className="muted">
            Scope: selected reservation route, FL{Math.round(pirepScope.lowerAltitudeFeet / 100)}-FL{Math.round(pirepScope.upperAltitudeFeet / 100)}
            {' '}±{pirepScope.altitudeToleranceFeet} ft, last {pirepScope.recencyMinutes} minutes, {pirepScope.corridorNauticalMiles} NM corridor.
          </p>
        </section>
        <section className="panel">
          <div className="panel-heading">
            <h3>Reservations</h3>
            <span>{mission.data?.reservations.length ?? 0}</span>
          </div>
          <DataTable
            data={mission.data?.reservations ?? []}
            columns={reservationColumns}
            onRowClick={(row) => setSelectedReservationId(row.id)}
            onRowDoubleClick={(row) => navigate(`/missions/${missionId}/reservations/${row.id}`)}
            isRowSelected={(row) => row.id === selectedReservation?.id}
          />
          {selectedReservation && (
            <div className="selection-console">
              <strong>{compactId(selectedReservation.id)}</strong>
              <StatusBadge value={selectedReservation.state} />
              <span>{selectedReservation.conflictCount} conflict(s)</span>
              <button className="secondary" onClick={() => navigate(`/missions/${missionId}/reservations/${selectedReservation.id}`)}>Open</button>
              <button className="secondary" onClick={() => navigate(`/deconfliction/${selectedReservation.id}`)}>Review</button>
            </div>
          )}
        </section>
        <section className="panel">
          <div className="panel-heading"><h3>Messages</h3><span>{mission.data?.messages.length ?? 0}</span></div>
          <DataTable
            data={mission.data?.messages ?? []}
            columns={messageColumns}
            onRowClick={(row) => setSelectedMessageId(row.id)}
            onRowDoubleClick={(row) => navigate(`/messages/${row.id}`)}
            isRowSelected={(row) => row.id === selectedMessage?.id}
          />
          {selectedMessage && (
            <div className="selection-console">
              <strong>{selectedMessage.subject || selectedMessage.family}</strong>
              <StatusBadge value={selectedMessage.status} />
              <span>{selectedMessage.direction}</span>
              <button className="secondary" onClick={() => navigate(`/messages/${selectedMessage.id}`)}>Open Message</button>
            </div>
          )}
        </section>
        <section className="panel">
          <div className="panel-heading"><h3>History</h3><Link to="/history">Open history</Link></div>
          <div className="event-list">
            {(mission.data?.history ?? []).map((event) => (
              <div className="event" key={event.id}>
                <strong>{event.eventType}</strong>
                <span>{event.actor ?? 'system'} · {fmtZ(event.createdAt)}</span>
                {event.note && <p>{event.note}</p>}
              </div>
            ))}
            {!mission.data?.history?.length && <p className="muted">No history has been recorded for this mission.</p>}
          </div>
        </section>
        <OperationsMap features={features.data} />
      </div>
    </section>
  );
}

function verdictFromBackend(verdict: MissionWeatherVerdictSummary): MissionWeatherVerdict {
  return {
    action: verdict.action as MissionWeatherVerdict['action'],
    priority: (verdict.priority || 'LOW') as MissionWeatherVerdict['priority'],
    confidence: verdict.confidence,
    count: verdict.sourceCount,
    summary: verdict.summary,
    recommendedAction: verdict.recommendedAction,
    sources: (verdict.sources ?? []).map((source) => ({
      id: source.id,
      hazard: source.label || source.family,
      action: verdict.action as MissionWeatherVerdict['action'],
      priority: (verdict.priority || 'LOW') as MissionWeatherVerdict['priority'],
      coordination: verdict.recommendedAction,
      rationale: source.rationale || verdict.summary,
      sourceFamily: source.family,
      sourceLabel: source.label || source.id,
      createdAt: source.observedAt
    }))
  };
}

function coordinationDraftHref(draft: { missionId: string; reservationId?: string; subject: string; family: string; direction: string; rawText: string; recipients?: string[] }) {
  const params = new URLSearchParams({
    missionId: draft.missionId,
    family: draft.family,
    direction: draft.direction,
    subject: draft.subject,
    body: draft.rawText,
    recipients: (draft.recipients ?? []).join(', ')
  });
  if (draft.reservationId) params.set('reservationId', draft.reservationId);
  return `/messages?${params.toString()}`;
}

function Metric({ label, value, attention }: { label: string; value: string; attention?: boolean }) {
  return (
    <div className={attention ? 'metric-tile attention' : 'metric-tile'}>
      <span>{label}</span>
      <strong>{value}</strong>
    </div>
  );
}

function pirepSourceLabel(item: { family?: string; label?: string; agingCategory?: string; relevanceScore?: number; ageMinutes?: number; stale?: boolean }) {
  const score = item.relevanceScore == null ? undefined : `${Math.round(item.relevanceScore * 100)}%`;
  const age = item.ageMinutes == null || item.ageMinutes < 0 ? 'age unknown' : `${Math.round(item.ageMinutes)}m`;
  return `${item.family ?? 'PIREP'} · ${item.label ?? 'report'} · ${item.agingCategory ?? (item.stale ? 'STALE' : 'CURRENT')} · ${score ?? 'score n/a'} · ${age}`;
}

function updatePirepScope(setScope: Dispatch<SetStateAction<PirepScope>>, key: keyof PirepScope, value: number) {
  setScope((scope) => ({
    ...scope,
    [key]: Number.isFinite(value) ? Math.max(0, value) : scope[key]
  }));
}
