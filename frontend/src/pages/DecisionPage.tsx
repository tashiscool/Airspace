import { useEffect, useMemo, useState } from 'react';
import { useMutation, useQuery } from '@tanstack/react-query';
import { useNavigate, useParams } from 'react-router-dom';
import { CheckCircle2, GitBranch, Play, RotateCcw, ShieldAlert } from 'lucide-react';
import { api } from '../api/client';
import { OperationsMap } from '../components/OperationsMap';
import { ErrorNotice, MutationNotice, QueryNotice } from '../components/Notices';
import { StatusBadge } from '../components/StatusBadge';
import { arrayValue, asRecord, decisionAvoidanceCandidates, decisionRoutePredictions, decisionSourceLinks, decisionSourceReferences, groupDecisionTrace, normalizeDecisionTrace, recordLabel, type JsonRecord } from '../lib/decisionView';
import { compactId, parseJson } from '../lib/viewModels';
import { writeWorkbenchJson, type WorkbenchSelection } from '../lib/workbenchState';

type DecisionTab = 'summary' | 'constraints' | 'trace' | 'audit' | 'replay' | 'map';

export function DecisionPage() {
  const { decisionId = 'latest' } = useParams();
  const navigate = useNavigate();
  const [tab, setTab] = useState<DecisionTab>('summary');
  const [traceFilter, setTraceFilter] = useState('');
  const [traceStageFilter, setTraceStageFilter] = useState('ALL');
  const evaluate = useMutation({
    mutationFn: () => api.evaluateDecision({
      decisionTime: new Date().toISOString(),
      route: [[30, -150.5, 24000], [30, -148.5, 24000]]
    }),
    onSuccess: (decision) => navigate(`/decisions/${decision.id}`)
  });
  const savedDecision = useQuery({
    queryKey: ['decision', decisionId],
    queryFn: () => api.decision(decisionId),
    enabled: decisionId !== 'latest'
  });
  const decisionSearch = useQuery({
    queryKey: ['search', 'decision-queue'],
    queryFn: () => api.search('decision'),
    staleTime: 30_000
  });
  const decision = evaluate.data ?? savedDecision.data;
  const replay = useMutation({ mutationFn: () => api.replayDecision(decision!.id) });
  const features = useQuery({
    queryKey: ['decision-features', decision?.id],
    queryFn: () => api.decisionFeatures(decision!.id),
    enabled: !!decision?.id
  });
  const persistedResult = parseJson<JsonRecord>(decision?.resultJson);
  const persistedAudit = parseJson<JsonRecord>(decision?.auditJson);
  const persistedReplay = parseJson<JsonRecord>(decision?.replayJson);
  const effectiveResult = asRecord(decision?.result) ?? persistedResult;
  const trace = useMemo(() => normalizeDecisionTrace(effectiveResult), [effectiveResult]);
  const traceGroups = useMemo(() => groupDecisionTrace(trace), [trace]);
  const filteredTrace = useMemo(() => {
    const q = traceFilter.trim().toLowerCase();
    return trace.filter((step) => {
      const stage = recordLabel(step, ['stage', 'category', 'ruleId'], 'trace');
      if (traceStageFilter !== 'ALL' && stage.toUpperCase() !== traceStageFilter) return false;
      return !q || JSON.stringify(step).toLowerCase().includes(q);
    });
  }, [trace, traceFilter, traceStageFilter]);
  const traceStages = useMemo(() => ['ALL', ...new Set(trace.map((step) => recordLabel(step, ['stage', 'category', 'ruleId'], 'trace').toUpperCase()))], [trace]);
  const blockingConstraints = arrayValue(effectiveResult?.blockingConstraints) ?? arrayValue(effectiveResult?.constraints) ?? [];
  const routePredictions = useMemo(() => decisionRoutePredictions(effectiveResult), [effectiveResult]);
  const avoidanceCandidates = useMemo(() => decisionAvoidanceCandidates(effectiveResult), [effectiveResult]);
  const sourceRefs = useMemo(() => decisionSourceReferences(trace, effectiveResult), [trace, effectiveResult]);
  const sourceLinks = useMemo(() => decisionSourceLinks(trace, effectiveResult), [trace, effectiveResult]);
  useEffect(() => {
    if (!decision) return;
    const selection: WorkbenchSelection = {
      decisionId: decision.id,
      sourceFamily: 'DECISION',
      label: decision.recommendedAction || decision.action,
      conflictCount: blockingConstraints.length,
      lockState: `${Math.round(decision.confidence * 100)}% confidence`
    };
    writeWorkbenchJson('airspace.workbench.selection', selection);
    window.dispatchEvent(new Event('airspace-workbench-selection'));
  }, [decision, blockingConstraints.length]);

  return (
    <section className="decision-workspace">
      <aside className="decision-queue">
        <header>Decision Queue</header>
        <button onClick={() => evaluate.mutate()}><Play size={14} /> Evaluate latest route</button>
        {decision ? (
          <button className="queue-item active">
            <span>{compactId(decision.id)}</span>
            <StatusBadge value={decision.action} />
            <small>{Math.round(decision.confidence * 100)}% confidence</small>
          </button>
        ) : (
          <p className="muted">No decision loaded. Evaluate a route or open a persisted decision.</p>
        )}
        {(decisionSearch.data ?? []).slice(0, 12).map((item) => (
          <button key={item.id} className={item.id === decision?.id ? 'queue-item active' : 'queue-item'} onClick={() => navigate(item.route ?? `/decisions/${item.id}`)}>
            <span>{item.title}</span>
            {item.status && <StatusBadge value={item.status} />}
            <small>{item.snippet || item.id}</small>
          </button>
        ))}
      </aside>
      <main className="decision-detail">
        <div className="page-header">
          <div>
            <h2><GitBranch size={18} /> Operational Decision</h2>
            <p>{decision?.rationale ?? 'USNS/CARF/NOTAM/weather/PIREP fusion result with replayable trace.'}</p>
          </div>
          {decision && <StatusBadge value={decision.action} />}
        </div>
        <div className="supplement-tabs">
          {(['summary', 'constraints', 'trace', 'audit', 'replay', 'map'] as DecisionTab[]).map((item) => (
            <button key={item} className={tab === item ? 'active' : ''} onClick={() => setTab(item)}>{item.toUpperCase()}</button>
          ))}
          <span />
          <button disabled={!decision?.id || replay.isPending} onClick={() => replay.mutate()}><RotateCcw size={14} /> Verify Replay</button>
        </div>
        <div className="notice-stack">
          {decisionId !== 'latest' && <QueryNotice query={savedDecision} label="Decision" />}
          <ErrorNotice error={features.error} title="Decision map features unavailable" />
          <MutationNotice mutation={evaluate} label="Evaluate decision" />
          <MutationNotice mutation={replay} label="Replay verification" />
        </div>
        {tab === 'summary' && (
          <>
            <section className="safety-loop-grid">
              <DecisionSafetyCard title="Inputs fused" value={inputCoverage(effectiveResult)} detail="USNS, CARF/ALTRV, NOTAM, weather, PIREP, and route candidates are normalized into one decision request." />
              <DecisionSafetyCard title="Route guidance" value={decision?.recommendedAction || decision?.action || '—'} detail="The operator sees a single action instead of raw products: clear, monitor, caution, delay, altitude change, avoid, reroute, or blocked." attention={isBlockingAction(decision?.recommendedAction || decision?.action)} />
              <DecisionSafetyCard title="Constraints" value={`${blockingConstraints.length} blocking`} detail="CARF conflicts, NOTAM restrictions, weather hazards, and route impacts remain linked to source IDs and map features." attention={blockingConstraints.length > 0} />
              <DecisionSafetyCard title="Auditability" value={`${trace.length} trace steps`} detail="Rule IDs, thresholds, source refs, warnings, and replay envelopes explain why the action was selected." />
            </section>
            <section className="decision-summary-grid">
              <Metric label="Action" value={decision?.recommendedAction || decision?.action || '—'} />
              <Metric label="Confidence" value={decision ? `${Math.round(decision.confidence * 100)}%` : '—'} />
              <Metric label="Blocking Constraints" value={String(blockingConstraints.length ?? 0)} attention={(blockingConstraints.length ?? 0) > 0} />
              <Metric label="Route Predictions" value={String(routePredictions.length ?? 0)} />
              <Metric label="Avoidance Candidates" value={String(avoidanceCandidates.length ?? 0)} attention={isBlockingAction(decision?.recommendedAction || decision?.action) && avoidanceCandidates.length === 0} />
              <Metric label="Source Refs" value={String(sourceRefs.length)} attention={isBlockingAction(decision?.recommendedAction || decision?.action) && sourceRefs.length === 0} />
              <div className="panel wide">
                <h3>Operational Rationale</h3>
                <p>{decision?.rationale ?? 'No decision has been evaluated yet.'}</p>
                <p className="muted">This is the engine’s answer to the core safety problem: convert live traffic, aircraft reports, forecasts, reservations, NOTAMs, and ATC constraints into clear operational guidance with traceable reasons.</p>
              </div>
              <div className="panel wide">
                <h3>Source Artifacts Driving This Decision</h3>
                {sourceLinks.length ? (
                  <div className="source-link-grid">
                    {sourceLinks.map((ref) => (
                      ref.route ? (
                        <button key={`${ref.type}:${ref.id}`} onClick={() => navigate(ref.route!)}>
                          <StatusBadge value={ref.type} />
                          <span>{ref.label}</span>
                          <small>{ref.id}</small>
                        </button>
                      ) : (
                        <span key={`${ref.type}:${ref.id}`} className="source-link-static">
                          <StatusBadge value={ref.type} />
                          <span>{ref.label}</span>
                          <small>{ref.id}</small>
                        </span>
                      )
                    ))}
                  </div>
                ) : (
                  <div className="source-ref-grid">
                    {(sourceRefs.length ? sourceRefs : ['No explicit weather/PIREP/NOTAM source refs were emitted by this decision.']).map((ref) => (
                      <span key={ref}>{ref}</span>
                    ))}
                  </div>
                )}
              </div>
              <div className="panel wide">
                <h3>Suggested Avoidance Corridors</h3>
                <div className="constraint-grid">
                  {(avoidanceCandidates.length ? avoidanceCandidates : ['No alternate corridor is attached to this decision.']).map((item, index) => (
                    <article className="event supplement" key={index}>
                      <strong>{recordLabel(item, ['id', 'name'], 'No candidate')}</strong>
                      <p>{recordLabel(item, ['rationale'], typeof item === 'string' ? item : JSON.stringify(item))}</p>
                      <small>{recordLabel(item, ['avoidedConstraintIds'], '')}</small>
                    </article>
                  ))}
                </div>
              </div>
            </section>
          </>
        )}
        {tab === 'constraints' && (
          <section className="panel">
            <h3>Blocking Constraints And Route Impacts</h3>
            <div className="constraint-grid">
              {(blockingConstraints.length ? blockingConstraints : ['No blocking constraints in current result']).map((item, index) => (
                <article className="event supplement" key={index}>
                  <strong>{recordLabel(item, ['type', 'constraintType'], 'constraint')}</strong>
                  <p>{recordLabel(item, ['rationale', 'sourceId', 'id'], JSON.stringify(item))}</p>
                </article>
              ))}
              {routePredictions.map((item, index) => (
                <article className="event supplement" key={`route-${index}`}>
                  <strong>Route impact</strong>
                  <p>{recordLabel(item, ['rationale', 'primaryHazardId', 'primaryHazard'], JSON.stringify(item))}</p>
                  <small>
                    probability {recordLabel(item, ['blockedProbability'], 'n/a')}
                    {' · '}confidence {recordLabel(item, ['confidence'], 'n/a')}
                    {' · '}forecast {recordLabel(item, ['forecastHour'], '?')}h
                  </small>
                </article>
              ))}
              {avoidanceCandidates.map((item, index) => (
                <article className="event supplement" key={`avoid-${index}`}>
                  <strong>Avoidance corridor · {recordLabel(item, ['id'], `candidate-${index}`)}</strong>
                  <p>{recordLabel(item, ['rationale'], JSON.stringify(item))}</p>
                  <small>avoids {recordLabel(item, ['avoidedConstraintIds'], 'none retained')}</small>
                </article>
              ))}
            </div>
          </section>
        )}
        {tab === 'trace' && (
          <section className="panel trace-panel">
            <div className="panel-heading">
              <h3>Decision Trace</h3>
              <div className="search-box"><input value={traceFilter} onChange={(event) => setTraceFilter(event.target.value)} placeholder="Filter rule, source, warning..." /></div>
            </div>
            <div className="trace-group-grid">
              {traceGroups.length ? traceGroups.map((group) => (
                <button
                  key={group.stage}
                  className={traceStageFilter === group.stage ? 'trace-group active' : 'trace-group'}
                  onClick={() => setTraceStageFilter(traceStageFilter === group.stage ? 'ALL' : group.stage)}
                  type="button"
                >
                  <span>{group.stage}</span>
                  <strong>{group.count}</strong>
                  <small>{group.warningCount ? `${group.warningCount} warning` : group.ruleIds.slice(0, 2).join(', ') || 'no rules'}</small>
                </button>
              )) : <p className="muted">No trace groups yet.</p>}
            </div>
            <div className="filter-strip">
              {traceStages.map((stage) => (
                <button key={stage} className={traceStageFilter === stage ? 'chip active' : 'chip'} onClick={() => setTraceStageFilter(stage)}>{stage}</button>
              ))}
            </div>
            {(filteredTrace.length ? filteredTrace : [{ stage: 'empty', message: trace.length ? 'No trace steps match the current filter.' : 'Evaluate or open a persisted decision to inspect trace steps.' }]).map((step, index) => (
              <div className="trace-step" key={index}>
                <span>{recordLabel(step, ['ruleId', 'stage', 'id'], `step-${index}`)}</span>
                <strong>{recordLabel(step, ['stage', 'category'], 'trace')}</strong>
                <p>{recordLabel(step, ['message', 'rationale', 'description'], JSON.stringify(step))}</p>
              </div>
            ))}
          </section>
        )}
        {tab === 'audit' && (
          <section className="panel">
            <h3>Audit Envelope</h3>
            {persistedAudit ? <pre className="raw-panel">{JSON.stringify(persistedAudit, null, 2)}</pre> : <p className="muted">No persisted audit envelope loaded.</p>}
          </section>
        )}
        {tab === 'replay' && (
          <section className="panel">
            <h3>Replay Verification</h3>
            {replay.data ? (
              <div className={replay.data.accepted ? 'verification accepted' : 'verification rejected'}>
                <strong>{replay.data.accepted ? 'Accepted' : 'Rejected'}</strong>
                {(replay.data.errors.length ? replay.data.errors : replay.data.warnings.length ? replay.data.warnings : ['Replay verified without diagnostics']).map((item, index) => (
                  <div key={index}>{item}</div>
                ))}
              </div>
            ) : (
              <p className="muted"><ShieldAlert size={14} /> Run replay verification to check persisted audit hashes, signatures, and rule-catalog compatibility.</p>
            )}
            {persistedReplay ? <pre className="raw-panel">{JSON.stringify(persistedReplay, null, 2)}</pre> : <p className="muted">No persisted replay bundle loaded.</p>}
          </section>
        )}
        {tab === 'map' && <OperationsMap features={features.data} />}
      </main>
    </section>
  );
}

function Metric({ label, value, attention }: { label: string; value: string; attention?: boolean }) {
  return (
    <div className={attention ? 'metric-tile attention' : 'metric-tile'}>
      <span>{label}</span>
      <strong>{value}</strong>
      {attention && <CheckCircle2 size={14} />}
    </div>
  );
}

function DecisionSafetyCard({ title, value, detail, attention }: { title: string; value: string; detail: string; attention?: boolean }) {
  return (
    <article className={attention ? 'safety-card attention-card' : 'safety-card'}>
      <header><span>{title}</span></header>
      <strong>{value}</strong>
      <p>{detail}</p>
    </article>
  );
}

function isBlockingAction(value?: string) {
  return /BLOCK|REROUTE|AVOID|DELAY|ALTITUDE/i.test(value ?? '');
}

function inputCoverage(result?: JsonRecord) {
  if (!result) return 'No result';
  const labels = [
    ['rawMessages', 'messages'],
    ['reservations', 'CARF'],
    ['notamRestrictions', 'NOTAM'],
    ['weatherProducts', 'weather'],
    ['pirepResults', 'PIREPs'],
    ['routeBlockagePredictions', 'route']
  ];
  const present = labels
    .filter(([key]) => {
      const value = result[key];
      return Array.isArray(value) ? value.length > 0 : value != null;
    })
    .map(([, label]) => label);
  return present.length ? present.join(' + ') : 'Decision trace';
}
