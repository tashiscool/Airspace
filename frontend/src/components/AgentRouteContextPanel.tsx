import { useMemo } from 'react';
import { useQuery } from '@tanstack/react-query';
import { Bot, ClipboardCheck, Gauge, ShieldCheck } from 'lucide-react';
import { api } from '../api/client';
import type { AgentRunResult, AgentTask } from '../types';
import { StatusBadge } from './StatusBadge';

type AgentRouteContextPanelProps = {
  title: string;
  description: string;
  agentTypes?: string[];
  missionId?: string;
  reservationId?: string;
  decisionId?: string;
  sourceFamily?: string;
  taskRouteContains?: string;
  limit?: number;
};

export function AgentRouteContextPanel({
  title,
  description,
  agentTypes = [],
  missionId,
  reservationId,
  decisionId,
  sourceFamily,
  taskRouteContains,
  limit = 4
}: AgentRouteContextPanelProps) {
  const normalizedAgentTypes = useMemo(() => [...new Set(agentTypes.filter(Boolean))], [agentTypes]);
  const runQuery = useQuery({
    queryKey: ['agent-route-context', 'runs', normalizedAgentTypes, missionId, reservationId, decisionId, sourceFamily, limit],
    queryFn: async () => {
      const baseFilters = { missionId, reservationId, decisionId };
      const batches = normalizedAgentTypes.length
        ? await Promise.all(normalizedAgentTypes.map((agentType) => api.agentRuns(limit, { ...baseFilters, agentType })))
        : [await api.agentRuns(limit, baseFilters)];
      return batches.flat().sort(compareRuns).slice(0, limit);
    },
    staleTime: 30_000
  });
  const taskQuery = useQuery({
    queryKey: ['agent-route-context', 'tasks', taskRouteContains, sourceFamily, limit],
    queryFn: () => api.agentTasks(undefined, limit, taskRouteContains ? { routeContains: taskRouteContains } : { sourceFamily }),
    staleTime: 30_000,
    enabled: Boolean(taskRouteContains || sourceFamily)
  });
  const runs = runQuery.data ?? [];
  const tasks = taskQuery.data ?? [];
  const latestRun = runs[0];
  const policyGuards = latestRun?.policyGuardDetails?.length
    ? latestRun.policyGuardDetails.map((guard) => guard.label ?? guard.id ?? 'policy guard')
    : latestRun?.policyGuards ?? [];
  const replayRefs = latestRun?.replayRefs ?? [];
  const approvals = latestRun?.approvalRequirements ?? [];
  return (
    <section className="agent-route-context" aria-label={title}>
      <div className="agent-route-context-header">
        <div>
          <span className="section-kicker"><Bot size={13} /> Safety Lab context</span>
          <h3>{title}</h3>
          <p>{description}</p>
        </div>
        <div className="agent-route-context-status">
          {latestRun ? <StatusBadge value={latestRun.accepted ? 'ACCEPTED' : 'REVIEW'} /> : <StatusBadge value={runQuery.isLoading ? 'LOADING' : 'NO RUN'} />}
          {latestRun?.executionTimeMs !== undefined && <small>{latestRun.executionTimeMs} ms</small>}
        </div>
      </div>
      <div className="agent-route-context-grid">
        <article className="agent-context-card primary">
          <span><Gauge size={13} /> Latest workload</span>
          <strong>{latestRun?.agentType ?? (normalizedAgentTypes.join(' / ') || 'Any Safety Lab agent')}</strong>
          <p>{latestRun?.summary ?? 'No matching agent run has been recorded yet. Use Agentic Ops or a page action to generate cited advisory evidence.'}</p>
          <div className="source-ref-grid compact">
            <span>{runs.length} run(s)</span>
            <span>{latestRun ? `${Math.round((latestRun.confidence ?? 0) * 100)}% confidence` : 'confidence pending'}</span>
            <span>${(latestRun?.costEstimate ?? 0).toFixed(4)} est.</span>
            <span>{latestRun?.evidenceReceipts?.length ?? 0} receipt(s)</span>
          </div>
          {runQuery.isError && <small className="warning-text">Agent run history is unavailable for this context.</small>}
        </article>
        <article className="agent-context-card">
          <span><ShieldCheck size={13} /> Policy and replay</span>
          <strong>{latestRun?.humanApprovalRequired ? 'Human approval required' : 'Advisory guardrails'}</strong>
          <ul className="compact-list">
            {(policyGuards.length ? policyGuards : ['NO_EXTERNAL_SEND', 'NO_OFFICIAL_MUTATION', 'HUMAN_APPROVAL_REQUIRED']).slice(0, 4).map((guard) => (
              <li key={guard}>{guard}</li>
            ))}
            {replayRefs.slice(0, 3).map((ref) => (
              <li key={`${ref.type}:${ref.id}`}>{ref.type ?? 'Replay'} · {ref.label ?? ref.id}</li>
            ))}
          </ul>
          {approvals.length > 0 && <small className="warning-text">{approvals.length} approval checkpoint(s) retained.</small>}
        </article>
        <article className="agent-context-card">
          <span><ClipboardCheck size={13} /> Review tasks</span>
          <strong>{tasks.length} open/context task(s)</strong>
          <ul className="compact-list">
            {(tasks.length ? tasks : emptyTasks()).slice(0, 4).map((task) => (
              <li key={task.id}>
                <StatusBadge value={task.priority ?? task.status ?? 'INFO'} /> {task.title ?? task.rationale ?? task.id}
              </li>
            ))}
          </ul>
          {taskQuery.isError && <small className="warning-text">Agent review task history is unavailable for this context.</small>}
        </article>
      </div>
    </section>
  );
}

function compareRuns(a: AgentRunResult, b: AgentRunResult) {
  const bTime = Date.parse(b.generatedAt ?? '');
  const aTime = Date.parse(a.generatedAt ?? '');
  return (Number.isFinite(bTime) ? bTime : 0) - (Number.isFinite(aTime) ? aTime : 0);
}

function emptyTasks(): AgentTask[] {
  return [{
    id: 'no-context-agent-task',
    title: 'No matching human-review tasks yet.',
    status: 'INFO',
    priority: 'INFO'
  }];
}
