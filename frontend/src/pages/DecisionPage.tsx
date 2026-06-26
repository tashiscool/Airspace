import { useEffect, useMemo, useState } from 'react';
import { useMutation, useQuery } from '@tanstack/react-query';
import { useNavigate, useParams } from 'react-router-dom';
import { CheckCircle2, GitBranch, Play, RotateCcw, ShieldAlert } from 'lucide-react';
import { api } from '../api/client';
import { AgentRouteContextPanel } from '../components/AgentRouteContextPanel';
import { OperationsMap } from '../components/OperationsMap';
import { ErrorNotice, MutationNotice, QueryNotice } from '../components/Notices';
import { RouteCandidateComparisonPanel } from '../components/RouteCandidateComparisonPanel';
import { StatusBadge } from '../components/StatusBadge';
import { arrayValue, asRecord, decisionAvoidanceCandidates, decisionRouteImpactSummary, decisionRoutePredictions, decisionSourceLinks, decisionSourceReferences, groupDecisionTrace, normalizeDecisionTrace, recordLabel, type JsonRecord } from '../lib/decisionView';
import { compactId, mergeFeatureCollections, parseJson, routeImpactFeaturesFromSummary } from '../lib/viewModels';
import { writeWorkbenchJson, type WorkbenchSelection } from '../lib/workbenchState';

type DecisionTab = 'summary' | 'constraints' | 'trace' | 'audit' | 'replay' | 'map';

export function DecisionPage() {
  const { decisionId = 'latest' } = useParams();
  const navigate = useNavigate();
  const [tab, setTab] = useState<DecisionTab>('summary');
  const [traceFilter, setTraceFilter] = useState('');
  const [traceStageFilter, setTraceStageFilter] = useState('ALL');
  const [selectedRouteCandidateId, setSelectedRouteCandidateId] = useState('');
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
  const calibrationReports = useQuery({
    queryKey: ['readiness', 'calibration'],
    queryFn: api.calibrationReports,
    staleTime: 60_000
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
  const routeImpactSummary = useMemo(() => decision?.routeImpact ?? decisionRouteImpactSummary(effectiveResult), [decision?.routeImpact, effectiveResult]);
  const routeCandidateFeatures = useMemo(() => routeImpactFeaturesFromSummary(routeImpactSummary), [routeImpactSummary]);
  const mapFeatures = useMemo(() => mergeFeatureCollections(features.data, { type: 'FeatureCollection', features: routeCandidateFeatures }), [features.data, routeCandidateFeatures]);
  const selectedRouteFeatureId = selectedRouteCandidateId
    ? `route-candidate-${routeImpactSummary?.missionId ?? 'mission'}-${selectedRouteCandidateId}`
    : undefined;
  const sourceRefs = useMemo(() => {
    return [...new Set([
      ...decisionSourceReferences(trace, effectiveResult),
      ...(routeImpactSummary?.sourceRefs ?? [])
    ])];
  }, [trace, effectiveResult, routeImpactSummary?.sourceRefs]);
  const sourceLinks = useMemo(() => decisionSourceLinks(trace, effectiveResult), [trace, effectiveResult]);
  const engineAction = decision?.recommendedAction || decision?.action || '—';
  const contextualAction = routeImpactSummary?.recommendedAction || routeImpactSummary?.action || engineAction;
  const contextualAttention = isBlockingAction(contextualAction) || (routeImpactSummary?.candidateComparisons?.length ?? 0) > 0;
  const avoidanceCount = Math.max(avoidanceCandidates.length, routeImpactSummary?.candidateComparisons?.length ?? 0);
  useEffect(() => {
    if (!decision) return;
    const selection: WorkbenchSelection = {
      decisionId: decision.id,
      sourceFamily: 'DECISION',
      label: contextualAction,
      conflictCount: Math.max(blockingConstraints.length, routeImpactSummary?.blockingConstraintCount ?? 0),
      lockState: `engine ${engineAction} · ${Math.round(decision.confidence * 100)}% confidence`
    };
    writeWorkbenchJson('airspace.workbench.selection', selection);
    window.dispatchEvent(new Event('airspace-workbench-selection'));
  }, [decision, blockingConstraints.length, contextualAction, engineAction, routeImpactSummary?.blockingConstraintCount]);

  return (
    <section className="decision-workspace">
      <aside className="decision-queue">
        <header>Decision Queue</header>
        <button onClick={() => evaluate.mutate()}><Play size={14} /> Evaluate latest route</button>
        {decision ? (
          <button className="queue-item active">
            <span>{compactId(decision.id)}</span>
            <StatusBadge value={contextualAction} />
            <small>engine {engineAction} · {Math.round(decision.confidence * 100)}% confidence</small>
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
            <p>{routeImpactSummary?.rationale ?? decision?.rationale ?? 'USNS/CARF/NOTAM/weather/PIREP fusion result with replayable trace.'}</p>
          </div>
          {decision && <StatusBadge value={contextualAction} />}
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
          <ErrorNotice error={calibrationReports.error} title="Calibration reports unavailable" />
          <MutationNotice mutation={evaluate} label="Evaluate decision" />
          <MutationNotice mutation={replay} label="Replay verification" />
        </div>
        <AgentRouteContextPanel
          title="Decision Agent Evidence"
          description="Unsafe-guidance red-team and replay-integrity checks stay attached to the decision trace they evaluate."
          agentTypes={['UNSAFE_GUIDANCE_RED_TEAM', 'REPLAY_INTEGRITY_AGENT']}
          decisionId={decision?.id ?? (decisionId !== 'latest' ? decisionId : undefined)}
          sourceFamily="DECISION"
          taskRouteContains={decision?.id ? `/decisions/${decision.id}` : '/decisions'}
        />
        {tab === 'summary' && (
          <>
            <section className="safety-loop-grid">
              <DecisionSafetyCard title="Inputs fused" value={inputCoverage(effectiveResult)} detail="USNS, CARF/ALTRV, NOTAM, weather, PIREP, and route candidates are normalized into one decision request." />
              <DecisionSafetyCard title="Route guidance" value={contextualAction} detail="The operator sees a single action instead of raw products: clear, monitor, caution, delay, altitude change, avoid, reroute, or blocked." attention={contextualAttention} />
              <DecisionSafetyCard title="Constraints" value={`${Math.max(blockingConstraints.length, routeImpactSummary?.blockingConstraintCount ?? 0)} blocking`} detail="CARF conflicts, NOTAM restrictions, weather hazards, and route impacts remain linked to source IDs and map features." attention={blockingConstraints.length > 0 || (routeImpactSummary?.blockingConstraintCount ?? 0) > 0} />
              <DecisionSafetyCard title="Auditability" value={`${trace.length} trace steps`} detail="Rule IDs, thresholds, source refs, warnings, and replay envelopes explain why the action was selected." />
            </section>
            <section className="decision-summary-grid">
              <Metric label="Engine Action" value={engineAction} />
              <Metric label="Mission Route Action" value={contextualAction} attention={contextualAttention} />
              <Metric label="Confidence" value={decision ? `${Math.round(decision.confidence * 100)}%` : '—'} />
              <Metric label="Blocking Constraints" value={String(Math.max(blockingConstraints.length ?? 0, routeImpactSummary?.blockingConstraintCount ?? 0))} attention={(blockingConstraints.length ?? 0) > 0 || (routeImpactSummary?.blockingConstraintCount ?? 0) > 0} />
              <Metric label="Route Predictions" value={String(Math.max(routePredictions.length ?? 0, routeImpactSummary?.impactedSegmentCount ?? 0))} />
              <Metric label="Avoidance Candidates" value={String(avoidanceCount)} attention={contextualAttention && avoidanceCount === 0} />
              <Metric label="Source Refs" value={String(sourceRefs.length)} attention={contextualAttention && sourceRefs.length === 0} />
              <Metric
                label="Calibration"
                value={calibrationReports.data?.[0]?.routeImpactReport?.calibrationVersion ?? 'fixture pending'}
                attention={(calibrationReports.data?.[0]?.routeImpactReport?.uncalibratedCoefficientCount ?? 1) > 0}
              />
              {decision?.routeImpact && (
                <div className="panel wide contextual-decision-note">
                  <h3>Mission Context Overlay</h3>
                  <p>The engine result remains replayable as evaluated. This page also attaches the selected mission/reservation route-impact review so operators can compare the replayed decision with current weather, PIREP, NOTAM, and CARF route context.</p>
                  <div className="source-ref-grid">
                    <span>Mission {routeImpactSummary?.missionId ?? '—'}</span>
                    <span>Reservation {routeImpactSummary?.reservationId ?? '—'}</span>
                    <span>{routeImpactSummary?.candidateComparisons?.length ?? 0} route alternate(s)</span>
                    <span>{routeImpactSummary?.sourceRefs?.length ?? 0} route source ref(s)</span>
                  </div>
                </div>
              )}
              <div className="panel wide">
                <h3>Operational Rationale</h3>
                <p>{routeImpactSummary?.rationale ?? decision?.rationale ?? 'No decision has been evaluated yet.'}</p>
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
                <RouteCandidateComparisonPanel
                  routeImpact={routeImpactSummary}
                  title="Why This Reroute?"
                  selectedCandidateId={selectedRouteCandidateId}
                  onCandidateSelect={(candidateId) => {
                    setSelectedRouteCandidateId(candidateId);
                    setTab('map');
                  }}
                />
              </div>
              <div className="panel wide">
                <h3>Calibration Readiness</h3>
                <p>{calibrationReports.data?.[0]?.routeImpactReport?.summary ?? 'Calibration report unavailable. Treat route blockage scoring as deterministic prototype guidance until fixture and historical reports are loaded.'}</p>
                <div className="source-ref-grid">
                  {(calibrationReports.data?.[0]?.routeImpactReport?.uncalibratedCoefficients ?? ['historical CWAP-style/CWAF-like outcomes', 'sector demand model']).slice(0, 6).map((item) => (
                    <span key={item}>{item}</span>
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
        {tab === 'map' && (
          <OperationsMap
            features={mapFeatures}
            selectedFeatureId={selectedRouteFeatureId}
            onSelectedFeatureIdChange={(featureId) => {
              const prefix = `route-candidate-${routeImpactSummary?.missionId ?? 'mission'}-`;
              setSelectedRouteCandidateId(featureId?.startsWith(prefix) ? featureId.slice(prefix.length) : '');
            }}
            affectedMissionIds={routeImpactSummary?.missionId ? [routeImpactSummary.missionId] : undefined}
            affectedSourceRefs={sourceRefs}
          />
        )}
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
