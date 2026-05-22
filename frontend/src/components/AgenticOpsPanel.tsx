import { useMemo, useState } from 'react';
import type { ReactNode } from 'react';
import { useMutation, useQuery, useQueryClient } from '@tanstack/react-query';
import { Bot, ClipboardCheck, FileWarning, Radio, Route, ShieldCheck } from 'lucide-react';
import { api } from '../api/client';
import type { AgentFinding, AgentRecommendation, AgentRunResult, AgentTask } from '../types';
import type { WorkbenchSelection } from '../lib/workbenchState';

export function AgenticOpsPanel({ selection }: { selection: WorkbenchSelection }) {
  const queryClient = useQueryClient();
  const [expanded, setExpanded] = useState(false);
  const [traceQuestion, setTraceQuestion] = useState('');
  const request = useMemo(() => ({
    missionId: selection.missionId,
    reservationId: selection.reservationId,
    decisionId: selection.decisionId,
    actor: 'planner'
  }), [selection.missionId, selection.reservationId, selection.decisionId]);
  const weatherImpact = useQuery({
    queryKey: ['agentic', 'weather-impact', request.missionId, request.reservationId, request.decisionId],
    queryFn: () => api.weatherImpactAgent(request),
    staleTime: 45_000
  });
  const runHistory = useQuery({
    queryKey: ['agentic', 'runs', request.missionId, request.reservationId, request.decisionId],
    queryFn: () => api.agentRuns(5, {
      missionId: request.missionId,
      reservationId: request.reservationId,
      decisionId: request.decisionId
    }),
    staleTime: 30_000
  });
  const taskHistory = useQuery({
    queryKey: ['agentic', 'tasks', request.missionId, request.reservationId, request.decisionId],
    queryFn: () => api.agentTasks(undefined, 8, { routeContains: request.reservationId ?? request.missionId ?? request.decisionId }),
    staleTime: 30_000
  });
  const storeStatus = useQuery({ queryKey: ['agentic', 'status'], queryFn: api.agentStatus, staleTime: 30_000 });
  const agentMetrics = useQuery({ queryKey: ['agentic', 'metrics'], queryFn: api.agentMetrics, staleTime: 30_000 });
  const runAll = useMutation({ mutationFn: () => api.runAgent({ ...request, agentType: 'ALL' }) });
  const missionRisk = useMutation({ mutationFn: () => api.missionRiskAgent(request) });
  const reroute = useMutation({ mutationFn: () => api.rerouteAnalysisAgent(request) });
  const coordination = useMutation({ mutationFn: () => api.coordinationDraftAgent(request) });
  const integrity = useMutation({ mutationFn: () => api.dataIntegrityAgent(request) });
  const replayAudit = useMutation({ mutationFn: () => api.replayAuditAgent({ ...request, question: traceQuestion }) });
  const transitionTask = useMutation({
    mutationFn: (id: string) => api.transitionAgentTask(id, { status: 'ACKNOWLEDGED', actor: 'planner', note: 'acknowledged from Agentic Ops panel' }),
    onSuccess: () => {
      queryClient.invalidateQueries({ queryKey: ['agentic', 'tasks'] });
      queryClient.invalidateQueries({ queryKey: ['agentic', 'runs'] });
    }
  });
  const result = replayAudit.data ?? runAll.data ?? missionRisk.data ?? reroute.data ?? coordination.data ?? integrity.data ?? weatherImpact.data;
  const findingCount = result?.findings?.length ?? 0;
  const taskCount = result?.tasks?.length ?? 0;
  const recommendationCount = result?.recommendations?.length ?? 0;
  return (
    <section className={expanded ? 'agentic-panel expanded' : 'agentic-panel'} aria-label="Agentic operations panel">
      <button className="agentic-panel-summary" onClick={() => setExpanded((value) => !value)}>
        <Bot size={15} />
        <strong>Agentic Ops</strong>
        <span>{weatherImpact.isLoading ? 'Analyzing...' : result?.summary ?? 'Weather impact watch ready'}</span>
        <small>{findingCount} findings</small>
        <small>{taskCount} tasks</small>
        <small>{recommendationCount} recommendations</small>
      </button>
      {expanded && (
        <div className="agentic-panel-body">
          <div className="toolbar wrap">
            <button onClick={() => runAll.mutate()} disabled={runAll.isPending}><ShieldCheck size={14} /> Run all agents</button>
            <button onClick={() => missionRisk.mutate()} disabled={missionRisk.isPending}><Radio size={14} /> Mission risk</button>
            <button onClick={() => reroute.mutate()} disabled={reroute.isPending}><Route size={14} /> Reroute analysis</button>
            <button onClick={() => coordination.mutate()} disabled={coordination.isPending}><ClipboardCheck size={14} /> Draft coordination</button>
            <button onClick={() => integrity.mutate()} disabled={integrity.isPending}><FileWarning size={14} /> Data integrity</button>
          </div>
          <div className="agentic-question">
            <input
              value={traceQuestion}
              onChange={(event) => setTraceQuestion(event.target.value)}
              placeholder="Ask against decision trace/source refs, e.g. why this reroute?"
            />
            <button onClick={() => replayAudit.mutate()} disabled={replayAudit.isPending}>Ask trace</button>
          </div>
          <AgentResult
            result={result}
            taskHistory={taskHistory.data}
            runHistory={runHistory.data}
            storeStatus={storeStatus.data}
            agentMetrics={agentMetrics.data}
            onAcknowledgeTask={(id) => transitionTask.mutate(id)}
            error={weatherImpact.error ?? runAll.error ?? missionRisk.error ?? reroute.error ?? coordination.error ?? integrity.error ?? replayAudit.error ?? taskHistory.error ?? runHistory.error ?? storeStatus.error ?? agentMetrics.error}
          />
        </div>
      )}
    </section>
  );
}

function AgentResult({
  result,
  taskHistory,
  runHistory,
  storeStatus,
  agentMetrics,
  onAcknowledgeTask,
  error
}: {
  result?: AgentRunResult;
  taskHistory?: AgentTask[];
  runHistory?: AgentRunResult[];
  storeStatus?: { mode?: string; durable?: boolean; path?: string; runCount?: number; taskCount?: number; latestRunAt?: string };
  agentMetrics?: Record<string, number>;
  onAcknowledgeTask: (id: string) => void;
  error?: Error | null;
}) {
  if (error) {
    return <p className="empty-state">Agentic analysis unavailable: {error.message}</p>;
  }
  if (!result) {
    return <p className="empty-state">No agent output yet. Run an agent to create cited analysis and review tasks.</p>;
  }
  return (
    <div className="agentic-grid">
      <AgentColumn title="Findings" items={result.findings} render={(item: AgentFinding) => (
        <>
          <span className={`status-badge ${String(item.severity ?? '').toLowerCase()}`}>{item.severity ?? 'INFO'}</span>
          <strong>{item.category}</strong>
          <p>{item.message}</p>
          <CitationList items={item.citations} />
        </>
      )} />
      <AgentColumn title="Recommendations" items={result.recommendations} render={(item: AgentRecommendation) => (
        <>
          <span className="status-badge">{item.action ?? 'REVIEW'}</span>
          <strong>{item.summary}</strong>
          <p>{item.rationale}</p>
          {item.humanApprovalRequired && <small className="warning-text">Human approval required before official action.</small>}
          <CitationList items={item.citations} />
        </>
      )} />
      {result.traceAnswer && (
        <div className="agent-card trace-answer-card">
          <h4>Trace Answer</h4>
          <span className="status-badge">Q&A</span>
          <strong>{result.traceAnswer.question ?? 'Trace question'}</strong>
          <p>{result.traceAnswer.answer ?? 'No evidence-grounded answer returned.'}</p>
          {!!result.traceAnswer.ruleIds?.length && (
            <small>Rules {result.traceAnswer.ruleIds.slice(0, 6).join(', ')}</small>
          )}
          {!!result.traceAnswer.sourceRefs?.length && (
            <small>Sources {result.traceAnswer.sourceRefs.slice(0, 6).join(', ')}</small>
          )}
          {result.traceAnswer.unsupportedClaims?.map((claim) => <small key={claim} className="warning-text">{claim}</small>)}
          <CitationList items={result.traceAnswer.citations} />
        </div>
      )}
      <div className="agent-card delta-card">
        <h4>What Changed</h4>
        {(result.deltas ?? []).slice(0, 6).map((delta) => (
          <div className="agent-item" key={delta.id}>
            <span className={`status-badge ${String(delta.severity ?? '').toLowerCase()}`}>{delta.changeType ?? 'DELTA'}</span>
            <strong>{delta.sourceFamily}:{delta.sourceId}</strong>
            <p>{delta.previousValue ?? 'previous'} → {delta.currentValue ?? 'current'}</p>
            <CitationList items={delta.citations} />
          </div>
        ))}
        {(!result.deltas || result.deltas.length === 0) && <p className="empty-state compact">No typed deltas returned.</p>}
      </div>
      <AgentColumn title="Tasks" items={(taskHistory?.length ? taskHistory : result.tasks)} render={(item: AgentTask) => (
        <>
          <span className={`status-badge ${String(item.priority ?? '').toLowerCase()}`}>{item.priority ?? 'INFO'}</span>
          <strong>{item.title}</strong>
          <p>{item.rationale}</p>
          {item.route && <a href={item.route}>Open workspace</a>}
          {item.status !== 'ACKNOWLEDGED' && <button className="mini-action" onClick={() => onAcknowledgeTask(item.id)}>Acknowledge</button>}
          <CitationList items={item.citations} />
        </>
      )} />
      <div className="agent-card loop-card">
        <h4>NextGen Loop</h4>
        {(result.operatingLoop ?? []).slice(0, 8).map((step) => (
          <div className="loop-step" key={step.stage}>
            <span className={`status-badge ${String(step.status ?? '').toLowerCase()}`}>{step.stage}</span>
            <strong>{step.status}</strong>
            <p>{step.summary}</p>
          </div>
        ))}
        {(!result.operatingLoop || result.operatingLoop.length === 0) && <p className="empty-state compact">No operating-loop snapshot returned.</p>}
      </div>
      <div className="agent-card audit-card">
        <h4>Audit</h4>
        <p>{result.accepted ? 'Citation validation accepted.' : 'Citation validation blocked operational display.'}</p>
        <p>
          Evaluation {Math.round((result.evaluation?.citationCoverage ?? 0) * 100)}% cited ·
          {` ${result.evaluation?.policyViolationCount ?? 0}`} policy issue(s)
        </p>
        <small>
          Claims {result.evaluation?.citedClaimCount ?? 0} cited / {result.evaluation?.uncitedClaimCount ?? 0} uncited ·
          Deltas {result.evaluation?.deltaCount ?? result.deltas?.length ?? 0}
        </small>
        <small>
          Reasoning {result.reasoningEnvelope?.reasoningMode ?? 'deterministic'} ·
          {` ${result.reasoningEnvelope?.modelId ?? 'no-model'}`}
        </small>
        <small>
          Prompt {result.reasoningEnvelope?.promptVersion ?? 'n/a'} · Draft {result.reasoningEnvelope?.draftHash?.slice(0, 12) ?? 'pending'}
        </small>
        {result.reasoningEnvelope?.inputSummary && <small>{result.reasoningEnvelope.inputSummary}</small>}
        <small>Policy {result.auditEnvelope?.policyVersion ?? 'default'} · Agent {result.auditEnvelope?.agentVersion ?? result.agentType}</small>
        <small>Input {result.auditEnvelope?.inputHash?.slice(0, 12) ?? 'pending'} · Output {result.auditEnvelope?.outputHash?.slice(0, 12) ?? 'pending'}</small>
        <small>
          Store {storeStatus?.mode ?? 'UNKNOWN'} · {storeStatus?.durable ? 'restart-safe' : 'memory-only'} ·
          {` ${storeStatus?.runCount ?? runHistory?.length ?? 0}`} run(s) / {storeStatus?.taskCount ?? taskHistory?.length ?? 0} task(s)
        </small>
        <small>
          Agent metrics accepted {agentMetrics?.['agentic.runs.accepted'] ?? 0} · rejected {agentMetrics?.['agentic.runs.rejected'] ?? 0} · policy {agentMetrics?.['agentic.policyViolations'] ?? 0}
        </small>
        {storeStatus?.path && <small>{storeStatus.path}</small>}
        {!storeStatus?.durable && <small className="warning-text">Agent history is memory-only unless `airspace.agentic.store.path` is configured.</small>}
        {result.diagnostics?.map((diagnostic) => <small key={diagnostic} className="warning-text">{diagnostic}</small>)}
      </div>
    </div>
  );
}

function AgentColumn<T extends { id: string }>({ title, items, render }: { title: string; items?: T[]; render: (item: T) => ReactNode }) {
  const safeItems = items ?? [];
  return (
    <div className="agent-card">
      <h4>{title}</h4>
      {safeItems.length === 0 && <p className="empty-state compact">None</p>}
      {safeItems.slice(0, 5).map((item) => (
        <div className="agent-item" key={item.id}>{render(item)}</div>
      ))}
    </div>
  );
}

function CitationList({ items }: { items?: { sourceFamily?: string; sourceId?: string; label?: string; route?: string }[] }) {
  if (!items?.length) return <small className="warning-text">No citations</small>;
  return (
    <div className="source-chip-row">
      {items.slice(0, 6).map((item, index) => (
        <a key={`${item.sourceFamily}-${item.sourceId}-${index}`} href={item.route ?? '#'} className="source-chip">
          {item.sourceFamily ?? 'SRC'}:{item.sourceId ?? item.label ?? 'unknown'}
        </a>
      ))}
    </div>
  );
}
