import { useMutation, useQuery } from '@tanstack/react-query';
import { useParams } from 'react-router-dom';
import { api } from '../api/client';
import { OperationsMap } from '../components/OperationsMap';
import { StatusBadge } from '../components/StatusBadge';

export function DecisionPage() {
  const { decisionId = 'latest' } = useParams();
  const evaluate = useMutation({
    mutationFn: () => api.evaluateDecision({
      decisionTime: '2026-05-20T00:00:00Z',
      route: [[30, -150.5, 24000], [30, -148.5, 24000]]
    })
  });
  const replay = useMutation({
    mutationFn: () => api.replayDecision(decision!.id)
  });
  const savedDecision = useQuery({
    queryKey: ['decision', decisionId],
    queryFn: () => api.decision(decisionId),
    enabled: decisionId !== 'latest'
  });
  const decision = evaluate.data ?? savedDecision.data;
  const features = useQuery({
    queryKey: ['decision-features', decision?.id],
    queryFn: () => api.decisionFeatures(decision!.id),
    enabled: !!decision?.id
  });
  const result = decision?.result as any;
  const persistedResult = parseJson(decision?.resultJson);
  const persistedAudit = parseJson(decision?.auditJson);
  const persistedReplay = parseJson(decision?.replayJson);
  const effectiveResult = result ?? persistedResult;
  const trace = result?.trace?.steps ?? result?.decisionTrace?.steps ?? [];
  const persistedTrace = persistedResult?.trace?.steps ?? persistedResult?.decisionTrace?.steps ?? [];
  const blockingConstraints = effectiveResult?.blockingConstraints ?? effectiveResult?.constraints ?? [];
  const visibleTrace = trace.length ? trace : persistedTrace;
  return (
    <section className="workspace">
      <div className="toolbar">
        <h2>Operational Decision</h2>
        <button onClick={() => evaluate.mutate()}>Evaluate</button>
        <button className="secondary" disabled={!decision?.id || replay.isPending} onClick={() => replay.mutate()}>
          Verify Replay
        </button>
      </div>
      <div className="grid two">
        <div className="panel">
          <h3>Decision</h3>
          {decision && (
            <>
              <StatusBadge value={decision.action} />
              <p>{decision.rationale}</p>
              <p>Confidence {Math.round(decision.confidence * 100)}%</p>
            </>
          )}
          <h3>Blocking Constraints</h3>
          {(blockingConstraints.length ? blockingConstraints : ['No blocking constraints in current result']).slice(0, 8).map((item: any, index: number) => (
            <div className="event" key={index}>{typeof item === 'string' ? item : `${item.type ?? item.constraintType ?? 'constraint'} · ${item.sourceId ?? item.id ?? ''}`}</div>
          ))}
          <h3>Trace</h3>
          {(visibleTrace.length ? visibleTrace : ['Evaluate or open a persisted decision to inspect trace steps']).slice(0, 12).map((step: any, index: number) => (
            <div className="event" key={index}>{typeof step === 'string' ? step : `${step.stage ?? step.ruleId ?? step.id ?? 'step'} · ${step.message ?? step.rationale ?? ''}`}</div>
          ))}
          <h3>Replay Verification</h3>
          {replay.data ? (
            <div className={replay.data.accepted ? 'verification accepted' : 'verification rejected'}>
              <strong>{replay.data.accepted ? 'Accepted' : 'Rejected'}</strong>
              {(replay.data.errors.length ? replay.data.errors : replay.data.warnings.length ? replay.data.warnings : ['Replay verified without diagnostics']).map((item, index) => (
                <div key={index}>{item}</div>
              ))}
            </div>
          ) : (
            <p className="muted">Run replay verification to check persisted audit hashes and signature.</p>
          )}
          {replay.error ? <p className="error">{String(replay.error)}</p> : null}
          <h3>Audit Envelope</h3>
          {persistedAudit ? <pre className="raw-panel">{JSON.stringify(persistedAudit, null, 2)}</pre> : <p className="muted">No persisted audit envelope loaded.</p>}
          <h3>Replay Bundle</h3>
          {persistedReplay ? <pre className="raw-panel">{JSON.stringify(persistedReplay, null, 2)}</pre> : <p className="muted">No persisted replay bundle loaded.</p>}
        </div>
        <OperationsMap features={features.data} />
      </div>
    </section>
  );
}

function parseJson(value?: string): any {
  if (!value) return undefined;
  try {
    return JSON.parse(value);
  } catch {
    return undefined;
  }
}
