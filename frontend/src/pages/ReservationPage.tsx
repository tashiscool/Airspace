import { useEffect, useMemo, useState } from 'react';
import { useMutation, useQuery, useQueryClient } from '@tanstack/react-query';
import { useNavigate, useParams } from 'react-router-dom';
import { CheckCircle2, FileWarning, Lock, Play, Save, ShieldCheck, Unlock, XCircle } from 'lucide-react';
import { api } from '../api/client';
import { OperationsMap } from '../components/OperationsMap';
import { ErrorNotice, MutationNotice, QueryNotice } from '../components/Notices';
import { StatusBadge } from '../components/StatusBadge';
import { isPlainLeftClick, shouldWarnForNavigation } from '../lib/navigationGuard';
import {
  type AltrvSections,
  type SectionKey,
  composeAltrvSections,
  extractAltrvSections,
  fmtZ,
  notamRowsFromSources,
  sourceRefLabel,
  sourceRefRoute,
  validateAltrvSections
} from '../lib/viewModels';
import {
  DEFAULT_LAYOUT_PREFS,
  readWorkbenchJson,
  writeWorkbenchJson,
  type WorkbenchLayoutPrefs,
  type WorkbenchSelection
} from '../lib/workbenchState';

const sectionLabels: Record<SectionKey, string> = {
  A: 'Call signs / accountability',
  B: 'Aircraft',
  C: 'Departure / controlling agency',
  D: 'Route / area / altitude',
  E: 'Destination / event notes',
  F: 'Timing / AVANA / level-off',
  G: 'Comments / TAS / remarks'
};

export function ReservationPage() {
  const { missionId = '', reservationId = '' } = useParams();
  const queryClient = useQueryClient();
  const navigate = useNavigate();
  const mission = useQuery({ queryKey: ['mission', missionId], queryFn: () => api.mission(missionId), enabled: !!missionId });
  const messages = useQuery({ queryKey: ['messages'], queryFn: api.messages });
  const supplements = useQuery({
    queryKey: ['reservation-supplements', reservationId],
    queryFn: () => api.reservationSupplements(reservationId),
    enabled: !!reservationId
  });
  const features = useQuery({
    queryKey: ['reservation-features', reservationId],
    queryFn: () => api.reservationFeatures(reservationId),
    enabled: !!reservationId
  });
  const routeImpact = useQuery({
    queryKey: ['reservation-route-impact', missionId, reservationId],
    queryFn: () => api.missionRouteImpact(missionId, reservationId),
    enabled: !!missionId && !!reservationId
  });
  const reservation = useMemo(
    () => mission.data?.reservations.find((item) => item.id === reservationId),
    [mission.data, reservationId]
  );
  const [rawText, setRawText] = useState('');
  const [sections, setSections] = useState<AltrvSections>(() => extractAltrvSections());
  const [prefs, setPrefs] = useState<WorkbenchLayoutPrefs>(() => readWorkbenchJson('airspace.workbench.layout', DEFAULT_LAYOUT_PREFS));
  const [mode, setMode] = useState<'sections' | 'raw'>(prefs.reservationEditorMode);
  const [activeSupplement, setActiveSupplement] = useState<'MISSION' | 'MESSAGES' | 'CONSOLE' | 'APREQ' | 'COORDINATION' | 'NOTAM' | 'APPROVALS'>(
    isSupplementTab(prefs.reservationSupplementTab) ? prefs.reservationSupplementTab : 'CONSOLE'
  );
  const [forceReason, setForceReason] = useState('Operator override from reservation workspace');

  useEffect(() => {
    if (reservation?.rawText) {
      setRawText(reservation.rawText);
      setSections(extractAltrvSections(reservation.rawText));
    }
  }, [reservation?.rawText]);

  const effectiveRaw = mode === 'sections' ? composeAltrvSections(sections) : rawText;
  const hasUnsavedChanges = reservation?.rawText != null && effectiveRaw.trim() !== reservation.rawText.trim();
  useEffect(() => {
    if (!hasUnsavedChanges) return;
    const warn = (event: BeforeUnloadEvent) => {
      event.preventDefault();
      event.returnValue = '';
    };
    window.addEventListener('beforeunload', warn);
    return () => window.removeEventListener('beforeunload', warn);
  }, [hasUnsavedChanges]);
  useEffect(() => {
    if (!hasUnsavedChanges) return;
    const warnNavigation = (event: MouseEvent) => {
      if (event.defaultPrevented || !isPlainLeftClick(event)) return;
      const target = event.target instanceof Element ? event.target.closest<HTMLAnchorElement>('a[href]') : null;
      if (!target) return;
      const currentLocation = `${window.location.pathname}${window.location.search}${window.location.hash}`;
      if (!shouldWarnForNavigation(true, currentLocation, target.href, window.location.origin)) return;
      if (window.confirm('Reservation has unsaved edits. Leave without saving?')) return;
      event.preventDefault();
      event.stopPropagation();
    };
    document.addEventListener('click', warnNavigation, true);
    return () => document.removeEventListener('click', warnNavigation, true);
  }, [hasUnsavedChanges]);
  useEffect(() => {
    const currentPrefs = readWorkbenchJson('airspace.workbench.layout', DEFAULT_LAYOUT_PREFS);
    const next = { ...currentPrefs, reservationEditorMode: mode, reservationSupplementTab: activeSupplement };
    setPrefs(next);
    writeWorkbenchJson('airspace.workbench.layout', next);
  }, [mode, activeSupplement]);
  useEffect(() => {
    const selection: WorkbenchSelection = {
      missionId,
      reservationId,
      sourceFamily: 'CARF_ALTRV',
      label: `Reservation ${reservationId}`,
      lockState: reservation?.lockedBy ? `Locked by ${reservation.lockedBy}` : 'Unlocked',
      conflictCount: reservation?.conflictCount
    };
    writeWorkbenchJson('airspace.workbench.selection', selection);
    window.dispatchEvent(new Event('airspace-workbench-selection'));
  }, [missionId, reservationId, reservation?.lockedBy, reservation?.conflictCount]);
  const invalidate = () => {
    queryClient.invalidateQueries({ queryKey: ['mission', missionId] });
    queryClient.invalidateQueries({ queryKey: ['reservation-supplements', reservationId] });
    queryClient.invalidateQueries({ queryKey: ['reservation-features', reservationId] });
  };
  const action = {
    save: useMutation({ mutationFn: () => api.updateReservation(reservationId, effectiveRaw, 'planner'), onSuccess: invalidate }),
    parse: useMutation({ mutationFn: () => api.parseReservation(reservationId, 'planner'), onSuccess: invalidate }),
    deconflict: useMutation({ mutationFn: () => api.deconflictReservation(reservationId, 'planner'), onSuccess: invalidate }),
    forceParse: useMutation({ mutationFn: () => api.forceParseReservation(reservationId, 'planner', forceReason), onSuccess: invalidate }),
    forceDeconflict: useMutation({ mutationFn: () => api.forceDeconflictReservation(reservationId, 'planner', forceReason), onSuccess: invalidate }),
    submit: useMutation({ mutationFn: () => api.submitReservation(reservationId, 'planner'), onSuccess: invalidate }),
    approve: useMutation({ mutationFn: () => api.approveReservation(reservationId, 'supervisor'), onSuccess: invalidate }),
    reject: useMutation({ mutationFn: () => api.rejectReservation(reservationId, 'supervisor', 'Rejected from workspace'), onSuccess: invalidate }),
    cancel: useMutation({ mutationFn: () => api.cancelReservation(reservationId, 'planner', 'Cancelled from workspace'), onSuccess: invalidate }),
    complete: useMutation({ mutationFn: () => api.completeReservation(reservationId, 'planner'), onSuccess: invalidate }),
    lock: useMutation({ mutationFn: () => api.lockReservation(reservationId, 'planner'), onSuccess: invalidate }),
    unlock: useMutation({ mutationFn: () => api.unlockReservation(reservationId, 'planner'), onSuccess: invalidate })
  };
  const addSupplement = useMutation({
    mutationFn: (kind: string) => api.createReservationSupplement(reservationId, {
      kind,
      status: 'DRAFT',
      title: `${kind} from reservation workspace`,
      text: `${kind} coordination record generated by planner.`,
      actor: 'planner'
    }),
    onSuccess: invalidate
  });
  const transitionSupplement = useMutation({
    mutationFn: (value: { id: string; status: string }) =>
      api.transitionReservationSupplement(reservationId, value.id, {
        status: value.status,
        actor: value.status === 'APPROVED' ? 'supervisor' : 'planner',
        note: `${value.status} from reservation workspace`
      }),
    onSuccess: invalidate
  });
  const coordinationDraft = useMutation({
    mutationFn: () => api.coordinateWeather(missionId, { reservationId, actor: 'planner' })
  });

  const relatedMessages = (messages.data ?? []).filter((message) => message.reservationId === reservationId || message.missionId === missionId);
  const notams = notamRowsFromSources(relatedMessages, supplements.data ?? []);
  const sectionDiagnostics = validateAltrvSections(mode === 'sections' ? sections : extractAltrvSections(rawText));
  const sectionErrors = sectionDiagnostics.filter((item) => item.severity === 'ERROR').length;
  const guardedNavigate = (route: string) => {
    if (hasUnsavedChanges && !window.confirm('Reservation has unsaved edits. Leave without saving?')) return;
    navigate(route);
  };

  return (
    <section className="workspace operational-page">
      <div className="page-header">
        <div>
          <h2>Reservation {reservationId}</h2>
          <p>{mission.data?.mission.missionNumber ?? 'Mission'} · {reservation?.lockedBy ? `locked by ${reservation.lockedBy}` : 'unlocked'}</p>
        </div>
        <StatusBadge value={reservation?.state ?? 'LOADING'} />
      </div>
      <div className="toolbar wrap action-bar workbench-toolbar">
        <button onClick={() => action.lock.mutate()}><Lock size={14} /> Lock</button>
        <button className="secondary" onClick={() => action.unlock.mutate()}><Unlock size={14} /> Unlock</button>
        <button onClick={() => action.save.mutate()}><Save size={14} /> Save</button>
        <button onClick={() => action.parse.mutate()}><Play size={14} /> Parse</button>
        <button onClick={() => action.deconflict.mutate()}><ShieldCheck size={14} /> Deconflict</button>
        <button className="secondary" onClick={() => action.forceParse.mutate()}><FileWarning size={14} /> Force Parse</button>
        <button className="secondary" onClick={() => action.forceDeconflict.mutate()}><FileWarning size={14} /> Force Deconflict</button>
        <button onClick={() => action.submit.mutate()}><CheckCircle2 size={14} /> Submit</button>
        <button onClick={() => action.approve.mutate()}><CheckCircle2 size={14} /> Approve</button>
        <button className="secondary" onClick={() => action.reject.mutate()}><XCircle size={14} /> Reject</button>
        <button className="secondary" onClick={() => action.cancel.mutate()}>Cancel</button>
        <button className="secondary" onClick={() => action.complete.mutate()}>Complete</button>
        <button className="secondary" onClick={() => guardedNavigate(`/deconfliction/${reservationId}`)}>Review Conflicts</button>
        {hasUnsavedChanges && <span className="unsaved-indicator">Unsaved edits</span>}
      </div>
      <div className="notice-stack">
        <QueryNotice query={mission} label="Mission detail" />
        <ErrorNotice error={supplements.error} title="Supplements unavailable" />
        <ErrorNotice error={features.error} title="Reservation features unavailable" />
        <QueryNotice query={routeImpact} label="Route impact" />
        {Object.entries(action).map(([label, mutation]) => (
          <MutationNotice key={label} mutation={mutation} label={label} />
        ))}
        <MutationNotice mutation={addSupplement} label="Create supplement" />
        <MutationNotice mutation={transitionSupplement} label="Transition supplement" />
        <MutationNotice mutation={coordinationDraft} label="Coordinate weather" />
      </div>

      <div className="reservation-layout">
        <section className="panel section-editor">
          <div className="panel-heading">
            <h3>Sections A-G</h3>
            <div className="segmented">
              <button className={mode === 'sections' ? 'active' : ''} onClick={() => setMode('sections')}>Fields</button>
              <button className={mode === 'raw' ? 'active' : ''} onClick={() => setMode('raw')}>Raw</button>
            </div>
          </div>
          {mode === 'sections' ? (
            <div className="sections-grid">
              {(Object.keys(sectionLabels) as SectionKey[]).map((key) => (
                <label key={key}>
                  <span>{key}. {sectionLabels[key]}</span>
                  <textarea
                    value={sections[key]}
                    onChange={(event) => setSections((current) => ({ ...current, [key]: event.target.value }))}
                  />
                </label>
              ))}
            </div>
          ) : (
            <textarea className="raw-editor" value={rawText} onChange={(event) => setRawText(event.target.value)} />
          )}
        </section>

        <aside className="panel reservation-side">
          <h3>Operational State</h3>
          <dl className="detail-list">
            <dt>State</dt><dd>{reservation?.state ?? 'Loading'}</dd>
            <dt>Conflicts</dt><dd>{reservation?.conflictCount ?? 0}</dd>
            <dt>Locked By</dt><dd>{reservation?.lockedBy ?? 'Unlocked'}</dd>
            <dt>Diagnostics</dt><dd>{reservation?.diagnostics.length ?? 0}</dd>
            <dt>Field Checks</dt><dd>{sectionErrors ? `${sectionErrors} error(s)` : `${sectionDiagnostics.length} warning(s)`}</dd>
            <dt>Edit Mode</dt><dd>{mode}</dd>
            <dt>Change State</dt><dd>{hasUnsavedChanges ? 'Unsaved edits' : 'Saved'}</dd>
          </dl>
          <label className="field-label">Force action reason<input value={forceReason} onChange={(event) => setForceReason(event.target.value)} /></label>
          <div className="field-validation">
            <strong>Section Checks</strong>
            {(sectionDiagnostics.length
              ? sectionDiagnostics
              : [{ section: 'A' as const, severity: 'WARN' as const, message: 'No frontend section warnings. Backend parse/deconflict remains authoritative.' }]
            ).map((item, index) => (
              <div key={`${item.section}:${index}`} className={item.severity === 'ERROR' ? 'validation-line err' : 'validation-line warn'}>
                <span>{item.severity}</span>
                <p>{item.section}. {item.message}</p>
              </div>
            ))}
          </div>
          <div className="route-impact-card">
            <strong>Route Impact</strong>
            <StatusBadge value={routeImpact.data?.action ?? 'MONITOR'} />
            <p>{routeImpact.data?.rationale ?? 'Route impact is evaluated from reservation geometry plus weather, PIREP, NOTAM, and CARF constraints.'}</p>
            <small>{routeImpact.data?.impactedSegmentCount ?? 0} segment(s) · {routeImpact.data?.blockingConstraintCount ?? 0} blocker(s) · {Math.round((routeImpact.data?.confidence ?? 0) * 100)}%</small>
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
                      if (route) guardedNavigate(route);
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
          {coordinationDraft.data && <pre className="raw-panel">{coordinationDraft.data.rawText}</pre>}
          <OperationsMap features={features.data} />
        </aside>
      </div>

      <section className="panel supplements-panel">
        <div className="supplement-tabs">
          {(['MISSION', 'MESSAGES', 'CONSOLE', 'APREQ', 'COORDINATION', 'NOTAM', 'APPROVALS'] as const).map((tab) => (
            <button key={tab} className={activeSupplement === tab ? 'active' : ''} onClick={() => setActiveSupplement(tab)}>{tab}</button>
          ))}
          <span />
          <button onClick={() => addSupplement.mutate('APREQ')}>Add APREQ</button>
          <button onClick={() => addSupplement.mutate('COORDINATION')}>Add Coordination</button>
          <button onClick={() => addSupplement.mutate('NOTAM')}>Add NOTAM</button>
          <button onClick={() => addSupplement.mutate('APPROVAL')}>Add Approval</button>
        </div>
        {activeSupplement === 'CONSOLE' && <Diagnostics diagnostics={reservation?.diagnostics ?? []} />}
        {activeSupplement === 'MESSAGES' && <MessageSupplement messages={relatedMessages} />}
        {activeSupplement === 'MISSION' && <MissionSupplement mission={mission.data?.mission.missionNumber} reservations={mission.data?.reservations.length ?? 0} />}
        {activeSupplement === 'NOTAM' && <NotamSupplement rows={notams} />}
        {['APREQ', 'COORDINATION', 'APPROVALS'].includes(activeSupplement) && (
          <SupplementList
            kind={activeSupplement === 'APPROVALS' ? 'APPROVAL' : activeSupplement}
            items={(supplements.data ?? []).filter((item) => item.kind.toUpperCase().includes(activeSupplement === 'APPROVALS' ? 'APPROVAL' : activeSupplement))}
            onTransition={(id, status) => transitionSupplement.mutate({ id, status })}
          />
        )}
      </section>
    </section>
  );
}

function isSupplementTab(value: string): value is 'MISSION' | 'MESSAGES' | 'CONSOLE' | 'APREQ' | 'COORDINATION' | 'NOTAM' | 'APPROVALS' {
  return ['MISSION', 'MESSAGES', 'CONSOLE', 'APREQ', 'COORDINATION', 'NOTAM', 'APPROVALS'].includes(value);
}

function Diagnostics({ diagnostics }: { diagnostics: string[] }) {
  return (
    <div className="event-list">
      {(diagnostics.length ? diagnostics : ['No parser, deconfliction, or workflow diagnostics yet.']).map((item) => (
        <div className="event" key={item}>{item}</div>
      ))}
    </div>
  );
}

function MessageSupplement({ messages }: { messages: { id: string; family: string; status: string; subject?: string; rawText?: string }[] }) {
  return (
    <div className="supplement-grid">
      {messages.map((message) => (
        <article className="event supplement" key={message.id}>
          <strong>{message.family}</strong> <StatusBadge value={message.status} />
          <div>{message.subject || message.id}</div>
          <p>{message.rawText || 'No retained message text.'}</p>
        </article>
      ))}
      {!messages.length && <p className="muted">No linked messages.</p>}
    </div>
  );
}

function MissionSupplement({ mission, reservations }: { mission?: string; reservations: number }) {
  return <p className="muted">{mission ?? 'Mission'} currently has {reservations} reservation(s) in this workspace.</p>;
}

function NotamSupplement({ rows }: { rows: ReturnType<typeof notamRowsFromSources> }) {
  return (
    <div className="supplement-grid">
      {rows.map((row) => (
        <article className="event supplement" key={`${row.source}:${row.id}`}>
          <strong>{row.family}</strong> <StatusBadge value={row.status ?? row.source} />
          <div>{row.title}</div>
          <p>{row.text}</p>
        </article>
      ))}
      {!rows.length && <p className="muted">No NOTAM constraints are linked to this reservation. NOTAMs remain separate constraints unless explicitly parsed as CARF/ALTRV.</p>}
    </div>
  );
}

function SupplementList({
  kind,
  items,
  onTransition
}: {
  kind: string;
  items: { id: string; kind: string; status: string; title?: string; text?: string; updatedAt?: string }[];
  onTransition: (id: string, status: string) => void;
}) {
  return (
    <div className="supplement-grid">
      {items.map((item) => (
        <article className="event supplement" key={item.id}>
          <strong>{item.kind}</strong> <StatusBadge value={item.status} />
          <div>{item.title}</div>
          <p>{item.text}</p>
          <small>{fmtZ(item.updatedAt)}</small>
          <div className="toolbar compact">
            <button className="secondary" onClick={() => onTransition(item.id, 'SUBMITTED')}>Submit</button>
            <button className="secondary" onClick={() => onTransition(item.id, 'APPROVED')}>Approve</button>
            <button className="secondary" onClick={() => onTransition(item.id, 'REJECTED')}>Reject</button>
          </div>
        </article>
      ))}
      {!items.length && <p className="muted">No {kind} supplements yet.</p>}
    </div>
  );
}
