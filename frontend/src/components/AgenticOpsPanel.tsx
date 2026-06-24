import { useMemo, useState } from 'react';
import type { ReactNode } from 'react';
import { useMutation, useQuery, useQueryClient } from '@tanstack/react-query';
import { BarChart3, Bot, ClipboardCheck, FileWarning, FlaskConical, Radio, Route, ShieldCheck } from 'lucide-react';
import { api } from '../api/client';
import type {
  AgentAssessment,
  AgentFinding,
  AgentJobResult,
  AgentRecommendation,
  AgentRunResult,
  AgenticRiskAssessment,
  AgentStabilityResult,
  AgentTask,
  AgentWorkloadDefinition,
  McpEvidenceReceipt,
  McpServerDefinition,
  McpToolDescriptor
} from '../types';
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
  const workloadCatalog = useQuery({ queryKey: ['agentic', 'workloads'], queryFn: api.agentWorkloads, staleTime: 60_000 });
  const agentMetrics = useQuery({ queryKey: ['agentic', 'metrics'], queryFn: api.agentMetrics, staleTime: 30_000 });
  const riskAssessments = useQuery({ queryKey: ['agentic', 'risk-assessments'], queryFn: api.agentRiskAssessments, staleTime: 60_000 });
  const mcpServers = useQuery({ queryKey: ['agentic', 'mcp', 'servers'], queryFn: api.mcpServers, staleTime: 60_000 });
  const mcpTools = useQuery({ queryKey: ['agentic', 'mcp', 'tools', 'airspace-first-party'], queryFn: () => api.mcpTools('airspace-first-party'), staleTime: 60_000 });
  const mcpReceipts = useQuery({ queryKey: ['agentic', 'mcp', 'receipts'], queryFn: () => api.mcpReceipts(6), staleTime: 15_000 });
  const agentJobs = useQuery({ queryKey: ['agentic', 'jobs'], queryFn: () => api.agentJobs(5), staleTime: 15_000 });
  const safetyDossier = useQuery({ queryKey: ['readiness', 'safety-dossier'], queryFn: api.safetyDossier, staleTime: 60_000 });
  const runAll = useMutation({ mutationFn: () => api.runAgent({ ...request, agentType: 'ALL' }) });
  const queueJob = useMutation({
    mutationFn: () => api.enqueueAgentJob({ actor: 'planner', agentRunRequest: { ...request, agentType: 'MISSION_RISK' } }),
    onSuccess: () => {
      queryClient.invalidateQueries({ queryKey: ['agentic', 'jobs'] });
      queryClient.invalidateQueries({ queryKey: ['agentic', 'mcp', 'receipts'] });
      queryClient.invalidateQueries({ queryKey: ['agentic', 'runs'] });
    }
  });
  const stability = useMutation({
    mutationFn: () => api.evaluateAgentStability({
      iterations: 3,
      agentRunRequest: { ...request, agentType: 'MISSION_RISK' }
    })
  });
  const safetyLab = useMutation({ mutationFn: () => api.safetyLabAgent({ ...request, agentType: 'SAFETY_LAB_ALL', scenarioId: 'oceanic-altrv-convection' }) });
  const redTeam = useMutation({ mutationFn: () => api.runAgent({ ...request, agentType: 'UNSAFE_GUIDANCE_RED_TEAM' }) });
  const outcomeAudit = useMutation({ mutationFn: () => api.runAgent({ ...request, agentType: 'OUTCOME_METRICS_AUDITOR', scenarioId: 'oceanic-altrv-convection' }) });
  const tmiAudit = useMutation({ mutationFn: () => api.runAgent({ ...request, agentType: 'TMI_RECOMMENDATION_AUDITOR' }) });
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
  const result = replayAudit.data ?? safetyLab.data ?? redTeam.data ?? outcomeAudit.data ?? tmiAudit.data ?? runAll.data ?? missionRisk.data ?? reroute.data ?? coordination.data ?? integrity.data ?? weatherImpact.data;
  const selectedResult = queueJob.data?.runResult ?? result;
  const findingCount = selectedResult?.findings?.length ?? 0;
  const taskCount = selectedResult?.tasks?.length ?? 0;
  const recommendationCount = selectedResult?.recommendations?.length ?? 0;
  return (
    <section className={expanded ? 'agentic-panel expanded' : 'agentic-panel'} aria-label="Agentic operations panel">
      <button className="agentic-panel-summary" onClick={() => setExpanded((value) => !value)}>
        <Bot size={15} />
        <strong>Agentic Ops</strong>
        <span>{weatherImpact.isLoading ? 'Analyzing...' : selectedResult?.summary ?? 'Weather impact watch ready'}</span>
        <small>{findingCount} findings</small>
        <small>{taskCount} tasks</small>
        <small>{recommendationCount} recommendations</small>
      </button>
      {expanded && (
        <div className="agentic-panel-body">
          <div className="toolbar wrap">
            <button onClick={() => runAll.mutate()} disabled={runAll.isPending}><ShieldCheck size={14} /> Run all agents</button>
            <button onClick={() => queueJob.mutate()} disabled={queueJob.isPending}><ShieldCheck size={14} /> Queue MCP evidence job</button>
            <button onClick={() => stability.mutate()} disabled={stability.isPending}><ShieldCheck size={14} /> Stability check</button>
            <button onClick={() => safetyLab.mutate()} disabled={safetyLab.isPending}><FlaskConical size={14} /> Safety Lab</button>
            <button onClick={() => redTeam.mutate()} disabled={redTeam.isPending}><ShieldCheck size={14} /> Red-team</button>
            <button onClick={() => outcomeAudit.mutate()} disabled={outcomeAudit.isPending}><BarChart3 size={14} /> Outcomes</button>
            <button onClick={() => tmiAudit.mutate()} disabled={tmiAudit.isPending}><Route size={14} /> TMI audit</button>
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
            result={selectedResult}
            taskHistory={taskHistory.data}
            runHistory={runHistory.data}
            storeStatus={storeStatus.data}
            workloads={workloadCatalog.data}
            agentMetrics={agentMetrics.data}
            riskAssessments={riskAssessments.data}
            stabilityResult={stability.data}
            mcpServers={mcpServers.data}
            mcpTools={mcpTools.data}
            mcpReceipts={mcpReceipts.data}
            agentJobs={agentJobs.data}
            latestJob={queueJob.data}
            safetySummary={safetyDossier.data?.summary}
            safetyCheckpoints={safetyDossier.data?.humanReviewCheckpoints}
            onAcknowledgeTask={(id) => transitionTask.mutate(id)}
            error={weatherImpact.error ?? runAll.error ?? queueJob.error ?? stability.error ?? safetyLab.error ?? redTeam.error ?? outcomeAudit.error ?? tmiAudit.error ?? missionRisk.error ?? reroute.error ?? coordination.error ?? integrity.error ?? replayAudit.error ?? taskHistory.error ?? runHistory.error ?? workloadCatalog.error ?? storeStatus.error ?? agentMetrics.error ?? riskAssessments.error ?? mcpServers.error ?? mcpTools.error ?? mcpReceipts.error ?? agentJobs.error ?? safetyDossier.error}
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
  workloads,
  agentMetrics,
  riskAssessments,
  stabilityResult,
  mcpServers,
  mcpTools,
  mcpReceipts,
  agentJobs,
  latestJob,
  safetySummary,
  safetyCheckpoints,
  onAcknowledgeTask,
  error
}: {
  result?: AgentRunResult;
  taskHistory?: AgentTask[];
  runHistory?: AgentRunResult[];
  storeStatus?: { mode?: string; durable?: boolean; path?: string; runCount?: number; taskCount?: number; latestRunAt?: string };
  workloads?: AgentWorkloadDefinition[];
  agentMetrics?: Record<string, number>;
  riskAssessments?: AgenticRiskAssessment[];
  stabilityResult?: AgentStabilityResult;
  mcpServers?: McpServerDefinition[];
  mcpTools?: McpToolDescriptor[];
  mcpReceipts?: McpEvidenceReceipt[];
  agentJobs?: AgentJobResult[];
  latestJob?: AgentJobResult;
  safetySummary?: string;
  safetyCheckpoints?: string[];
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
          <HumanReviewBadge mode={item.humanReviewMode} reason={item.humanReviewReason} approvalRequired={item.humanApprovalRequired} />
          <CitationList items={item.citations} />
        </>
      )} />
      <AgentColumn title="Assessments" items={result.assessments} render={(item: AgentAssessment) => (
        <>
          <span className={`status-badge ${String(item.humanReviewMode ?? '').toLowerCase()}`}>{item.verdict ?? 'ASSESSMENT'}</span>
          <strong>{item.claim}</strong>
          <p>{item.requiredHumanAction}</p>
          {item.uncertainty && <small>{item.uncertainty}</small>}
          {!!item.counterEvidence?.length && <small className="warning-text">{item.counterEvidence.slice(0, 2).join(' · ')}</small>}
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
          <HumanReviewBadge mode={item.humanReviewMode} reason={item.humanReviewReason} />
          {item.route && <a href={item.route}>Open workspace</a>}
          {item.status !== 'ACKNOWLEDGED' && <button className="mini-action" onClick={() => onAcknowledgeTask(item.id)}>Acknowledge</button>}
          <CitationList items={item.citations} />
        </>
      )} />
      <div className="agent-card">
        <h4>MCP Tool Plane</h4>
        <p>MCP tools provide cited evidence and drafts; they do not authorize operational action.</p>
        {(mcpServers ?? []).map((server) => (
          <div className="agent-item" key={server.id}>
            <span className={`status-badge ${server.enabled ? 'clear' : 'warning'}`}>{server.enabled ? 'ENABLED' : 'SETUP'}</span>
            <strong>{server.name ?? server.id}</strong>
            <p>{server.description}</p>
            {server.setupRequired && <small className="warning-text">Setup required before invocation.</small>}
          </div>
        ))}
        {(mcpTools ?? []).slice(0, 6).map((tool) => (
          <div className="agent-item" key={tool.id}>
            <span className={`status-badge ${String(tool.sideEffectLevel ?? '').toLowerCase()}`}>{tool.sideEffectLevel}</span>
            <strong>{tool.id}</strong>
            <p>{tool.description}</p>
            {!!tool.requiredArguments?.length && <small>Required {tool.requiredArguments.join(', ')}</small>}
            {tool.riskProfile?.worstCaseBlastRadius && <small>Blast radius: {tool.riskProfile.worstCaseBlastRadius}</small>}
            {tool.riskProfile?.dataEgress && <small>Egress: {tool.riskProfile.dataEgress}</small>}
            {tool.riskProfile?.rollbackPath && <small>Rollback: {tool.riskProfile.rollbackPath}</small>}
          </div>
        ))}
      </div>
      <div className="agent-card">
        <h4>Risk Checklist</h4>
        {(riskAssessments ?? []).slice(0, 6).map((risk) => (
          <div className="agent-item" key={risk.id}>
            <span className={`status-badge ${String(risk.riskProfile?.requiredHumanReviewMode ?? '').toLowerCase()}`}>{risk.riskProfile?.requiredHumanReviewMode ?? 'REVIEW'}</span>
            <strong>{risk.subjectId}</strong>
            <p>{risk.riskProfile?.worstCaseBlastRadius ?? risk.summary}</p>
            <small>{risk.riskProfile?.toolProvenance ?? 'Tool provenance unavailable'}</small>
            <small>{risk.riskProfile?.modelProvenance ?? 'Model provenance unavailable'}</small>
            {!!risk.diagnostics?.length && <small className="warning-text">{risk.diagnostics.join(' · ')}</small>}
          </div>
        ))}
        {(!riskAssessments || riskAssessments.length === 0) && <p className="empty-state compact">No risk assessments returned.</p>}
      </div>
      <div className="agent-card">
        <h4>Stability Harness</h4>
        {stabilityResult ? (
          <>
            <span className={`status-badge ${stabilityResult.accepted ? 'clear' : 'warning'}`}>{stabilityResult.accepted ? 'STABLE' : 'UNSTABLE'}</span>
            <p>{stabilityResult.iterations ?? 0} run(s), {(stabilityResult.metrics ?? []).filter((metric) => metric.accepted).length}/{stabilityResult.metrics?.length ?? 0} metric(s) accepted.</p>
            {(stabilityResult.metrics ?? []).slice(0, 6).map((metric) => (
              <small key={metric.id} className={metric.accepted ? undefined : 'warning-text'}>
                {metric.name}: {formatMetric(metric.value)} / {formatMetric(metric.threshold)}
              </small>
            ))}
            {stabilityResult.diagnostics?.map((diagnostic) => <small key={diagnostic} className="warning-text">{diagnostic}</small>)}
          </>
        ) : (
          <p className="empty-state compact">Run a stability check to compare repeated deterministic agent output.</p>
        )}
      </div>
      <div className="agent-card">
        <h4>Evidence Receipts</h4>
        {(mcpReceipts ?? []).slice(0, 5).map((receipt) => (
          <div className="agent-item" key={receipt.id}>
            <span className={`status-badge ${String(receipt.status ?? '').toLowerCase()}`}>{receipt.status ?? 'RECEIPT'}</span>
            <strong>{receipt.toolId}</strong>
            <p>{receipt.policyDecision} · {receipt.redactionStatus} · {receipt.durationMillis ?? 0} ms</p>
            <small>Input {receipt.inputHash?.slice(0, 12) ?? 'pending'} · Output {receipt.outputHash?.slice(0, 12) ?? 'pending'}</small>
            <CitationList items={receipt.sourceRefs} />
          </div>
        ))}
        {(!mcpReceipts || mcpReceipts.length === 0) && <p className="empty-state compact">No MCP evidence receipts yet.</p>}
      </div>
      <div className="agent-card">
        <h4>Agent Jobs</h4>
        {latestJob && (
          <div className="agent-item">
            <span className={`status-badge ${String(latestJob.status ?? '').toLowerCase()}`}>{latestJob.status}</span>
            <strong>{latestJob.request?.agentRunRequest?.agentType ?? 'Queued agent'}</strong>
            <p>{latestJob.toolCalls?.length ?? 0} tool call(s), {latestJob.receipts?.length ?? 0} receipt(s)</p>
          </div>
        )}
        {(agentJobs ?? []).slice(0, 5).map((job) => (
          <div className="agent-item" key={job.id}>
            <span className={`status-badge ${String(job.status ?? '').toLowerCase()}`}>{job.status}</span>
            <strong>{job.request?.agentRunRequest?.agentType ?? job.id}</strong>
            <p>{job.toolCalls?.length ?? 0} tool call(s), {job.receipts?.length ?? 0} receipt(s)</p>
          </div>
        ))}
        {(!agentJobs || agentJobs.length === 0) && <p className="empty-state compact">No agent jobs retained yet.</p>}
      </div>
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
        <p>{result.externalSendPerformed ? 'External send performed' : 'No external send'} · {result.officialStateMutationPerformed ? 'Official state mutation performed' : 'No official mutation'}</p>
        {result.humanApprovalRequired && <small className="warning-text">Human approval required before any official action or external delivery.</small>}
        {!!result.policyGuards?.length && <small>Guards {result.policyGuards.slice(0, 6).join(', ')}</small>}
        {result.costBudget && (
          <small>
            Cost ${Number(result.costBudget.estimatedCostUsd ?? 0).toFixed(3)} / ${Number(result.costBudget.maxCostUsd ?? 0).toFixed(2)} ·
            timeout {result.costBudget.timeoutMillis ?? 0}ms · retries {result.costBudget.retryCap ?? 0}
          </small>
        )}
        {!!result.evidenceReceipts?.length && <small>Evidence receipts {result.evidenceReceipts.length} · first {result.evidenceReceipts[0]?.receiptHash?.slice(0, 16)}</small>}
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
        {!!result.reasoningEnvelope?.availableTools?.length && (
          <small>Tools {result.reasoningEnvelope.availableTools.slice(0, 4).join(', ')}</small>
        )}
        {!!result.reasoningEnvelope?.blockedTools?.length && (
          <small className="warning-text">Blocked/setup-required {result.reasoningEnvelope.blockedTools.slice(0, 3).join(', ')}</small>
        )}
        {!!result.reasoningEnvelope?.toolReceiptIds?.length && (
          <small>Receipts {result.reasoningEnvelope.toolReceiptIds.slice(0, 4).join(', ')}</small>
        )}
        {result.reasoningEnvelope?.toolPolicySummary && <small>{result.reasoningEnvelope.toolPolicySummary}</small>}
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
        {safetySummary && <small className="warning-text">{safetySummary}</small>}
        {(safetyCheckpoints ?? []).slice(0, 4).map((checkpoint) => <small key={checkpoint}>{checkpoint}</small>)}
        {storeStatus?.path && <small>{storeStatus.path}</small>}
        {!storeStatus?.durable && <small className="warning-text">Agent history is memory-only unless `airspace.agentic.store.path` is configured.</small>}
        {!!result.toolCalls?.length && <small>{result.toolCalls.length} MCP/tool receipt-backed call(s) attached.</small>}
        {result.diagnostics?.map((diagnostic) => <small key={diagnostic} className="warning-text">{diagnostic}</small>)}
      </div>
      <div className="agent-card workload-card">
        <h4>Safety Lab Workloads</h4>
        {(workloads ?? []).filter((workload) => ['UNSAFE_GUIDANCE_RED_TEAM', 'OUTCOME_METRICS_AUDITOR', 'TMI_RECOMMENDATION_AUDITOR', 'REPLAY_INTEGRITY_AGENT', 'SCENARIO_GENERATION'].includes(workload.id)).slice(0, 6).map((workload) => (
          <div className="agent-item" key={workload.id}>
            <span className="status-badge">{workload.category ?? 'WORKLOAD'}</span>
            <strong>{workload.label ?? workload.id}</strong>
            <p>{workload.gapCoverage}</p>
            {workload.humanApprovalRequired && <small className="warning-text">Human approval required.</small>}
          </div>
        ))}
        {(!workloads || workloads.length === 0) && <p className="empty-state compact">Workload catalog unavailable.</p>}
      </div>
    </div>
  );
}

function HumanReviewBadge({ mode, reason, approvalRequired }: { mode?: string; reason?: string; approvalRequired?: boolean }) {
  const safeMode = mode ?? (approvalRequired ? 'PUSH_APPROVAL' : 'REVIEW_ONLY');
  return (
    <>
      <small className={safeMode === 'PUSH_APPROVAL' || safeMode === 'PULL_CLARIFICATION' ? 'warning-text' : undefined}>
        HITL {safeMode.replace(/_/g, ' ').toLowerCase()}
      </small>
      {reason && <small>{reason}</small>}
    </>
  );
}

function formatMetric(value?: number) {
  if (value == null || Number.isNaN(value)) return 'n/a';
  return value >= 0.99 ? value.toFixed(2) : value.toFixed(3);
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
