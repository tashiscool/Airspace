import { useEffect, useState } from 'react';
import { useMutation, useQuery, useQueryClient } from '@tanstack/react-query';
import { createColumnHelper } from '@tanstack/react-table';
import { Database } from 'lucide-react';
import { api } from '../api/client';
import { DataTable } from '../components/DataTable';
import { ErrorNotice, MutationNotice, QueryNotice } from '../components/Notices';
import { writeWorkbenchJson, type WorkbenchSelection } from '../lib/workbenchState';
import type { AirspaceGapStatus, CalibrationRunSummary, CollaborativeProposalSummary, CommonOperatingPictureSummary, ProviderHealthSummary, ReferencePointSummary, ReleaseGateSummary, SafetyCaseDossierSummary } from '../types';

type ConfigNode = 'navaids' | 'fixes' | 'airspace' | 'recipients' | 'users' | 'separation' | 'imports' | 'providers' | 'collaboration' | 'calibration' | 'safety' | 'system';

const AIRSPACE_GROUPS = [
  { id: 'AG-PAC-W', name: 'PACIFIC WEST', owner: '613 AOC', classification: 'OPERATIONAL', members: 14 },
  { id: 'AG-ATL-E', name: 'ATLANTIC EAST', owner: '618 AOC', classification: 'OPERATIONAL', members: 9 },
  { id: 'AG-GOM', name: 'GULF OF MEXICO', owner: 'CENTCOM', classification: 'OPERATIONAL', members: 6 },
  { id: 'AG-ARC', name: 'ARCTIC', owner: 'NORAD', classification: 'RESTRICTED', members: 4 }
];

const RECIPIENTS = [
  { id: 'RC-001', handle: 'FAA-ATCSCC', channel: 'USNS', address: 'atcscc@usns.faa', priority: 'ROUTINE' },
  { id: 'RC-002', handle: 'NORAD-CMOC', channel: 'USNS', address: 'cmoc@norad.usns', priority: 'PRIORITY' },
  { id: 'RC-003', handle: 'ZJX-CTR', channel: 'USNS', address: 'zjx@usns.faa', priority: 'ROUTINE' },
  { id: 'RC-004', handle: 'FLEET-W', channel: 'INTERNAL', address: 'fleet-w@carf', priority: 'PRIORITY' },
  { id: 'RC-005', handle: '618-AOC-DUTY', channel: 'INTERNAL', address: 'duty@618aoc', priority: 'IMMEDIATE' }
];

const USERS = [
  { id: 'U-001', username: 'j.planner', role: 'PLANNER', status: 'ACTIVE' },
  { id: 'U-002', username: 'k.controller', role: 'CONTROLLER', status: 'ACTIVE' },
  { id: 'U-003', username: 'm.approver', role: 'APPROVER', status: 'ACTIVE' },
  { id: 'U-004', username: 's.admin', role: 'ADMIN', status: 'ACTIVE' },
  { id: 'U-005', username: 't.observer', role: 'READONLY', status: 'SUSPENDED' }
];

const PERMISSIONS = [
  'reservation.create',
  'reservation.submit',
  'reservation.approve',
  'reservation.reject',
  'deconflict.run',
  'deconflict.force',
  'message.transmit',
  'refdata.edit',
  'users.manage',
  'system.configure'
];

export function ConfigPage() {
  const queryClient = useQueryClient();
  const [selected, setSelected] = useState<ConfigNode>('navaids');
  const config = useQuery({ queryKey: ['config'], queryFn: api.config });
  const metrics = useQuery({ queryKey: ['metrics'], queryFn: api.metrics });
  const agentStatus = useQuery({ queryKey: ['agentic', 'status'], queryFn: api.agentStatus });
  const agentMetrics = useQuery({ queryKey: ['agentic', 'metrics'], queryFn: api.agentMetrics });
  const gaps = useQuery({ queryKey: ['readiness', 'gaps'], queryFn: api.gaps });
  const releaseGates = useQuery({ queryKey: ['readiness', 'release-gates'], queryFn: api.releaseGates });
  const providers = useQuery({ queryKey: ['readiness', 'providers'], queryFn: api.providersStatus });
  const commonOperatingPicture = useQuery({ queryKey: ['collaboration', 'common-operating-picture'], queryFn: api.commonOperatingPicture });
  const calibrationReports = useQuery({ queryKey: ['readiness', 'calibration'], queryFn: api.calibrationReports });
  const safetyDossier = useQuery({ queryKey: ['readiness', 'safety-dossier'], queryFn: api.safetyDossier });
  const points = useQuery({ queryKey: ['reference-points'], queryFn: () => api.referencePoints() });
  const [referenceImport, setReferenceImport] = useState('type,identifier,latitude,longitude,altitudeFeet,source,version\nFIX,DEMOFIX,39.0,-76.0,0,local,v1');
  const previewImport = useMutation({ mutationFn: () => api.previewReferenceImport(referenceImport) });
  const applyImport = useMutation({
    mutationFn: () => api.applyReferenceImport(referenceImport),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['reference-points'] })
  });
  const addPoint = useMutation({
    mutationFn: () => api.createReferencePoint({
      identifier: `LOCAL${Math.floor(Math.random() * 1000)}`,
      pointType: 'FIX',
      latitude: 38,
      longitude: -77,
      altitudeFeet: 0,
      source: 'operator'
    }),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['reference-points'] })
  });
  const pollProviderWeather = useMutation({
    mutationFn: () => api.pollProviderWeather({ products: ['metar', 'taf', 'airsigmet'], hoursBeforeNow: 2, maxResults: 25 }),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['readiness', 'providers'] })
  });
  const runCalibration = useMutation({
    mutationFn: () => api.runCalibration({ datasetId: 'fixture-weather-route-outcomes', includeSyntheticScale: true, actor: 'planner' }),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['readiness', 'calibration'] })
  });
  const createCollaborationProposal = useMutation({
    mutationFn: () => api.createCollaborativeProposal({
      proposalType: 'WEATHER_ROUTE_COORDINATION',
      recommendedAction: 'REROUTE',
      summary: 'Review weather route guidance with ATCSCC, facility TMU, airline dispatch, and mission owner.',
      rationale: 'Local common operating picture proposal for human-reviewed stakeholder coordination.',
      actor: 'planner',
      role: 'PLANNER',
      sourceRefs: ['COP:LOCAL', 'GUIDANCE:WORKBENCH']
    }),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['collaboration', 'common-operating-picture'] })
  });
  useEffect(() => {
    const selection: WorkbenchSelection = {
      sourceFamily: 'REFERENCE',
      label: `Config · ${selected}`,
      lockState: `${points.data?.length ?? 0} reference points`
    };
    writeWorkbenchJson('airspace.workbench.selection', selection);
    window.dispatchEvent(new Event('airspace-workbench-selection'));
  }, [selected, points.data?.length]);
  return (
    <section className="config-workspace">
      <aside className="config-tree">
        <header><Database size={14} /> Configuration</header>
        {([
          ['navaids', 'NAVAIDs'],
          ['fixes', 'Fixes / Reference Points'],
          ['airspace', 'Airspace Groups'],
          ['recipients', 'Recipients'],
          ['users', 'Users / RBAC'],
          ['separation', 'Flight Path Separation'],
          ['imports', 'Reference Imports'],
          ['providers', 'Providers / Freshness'],
          ['collaboration', 'Collaboration / COP'],
          ['calibration', 'Calibration / Release Gates'],
          ['safety', 'Safety Dossier'],
          ['system', 'System Configuration']
        ] as Array<[ConfigNode, string]>).map(([id, label]) => <button key={id} className={selected === id ? 'active' : ''} onClick={() => setSelected(id)}>{label}</button>)}
      </aside>
      <main className="panel">
        <div className="notice-stack">
          <ErrorNotice error={config.error} title="Config unavailable" />
          <ErrorNotice error={metrics.error} title="Metrics unavailable" />
          <ErrorNotice error={agentStatus.error} title="Agentic status unavailable" />
          <ErrorNotice error={agentMetrics.error} title="Agentic metrics unavailable" />
          <ErrorNotice error={gaps.error} title="Gap registry unavailable" />
          <ErrorNotice error={releaseGates.error} title="Release gates unavailable" />
          <ErrorNotice error={providers.error} title="Provider status unavailable" />
          <ErrorNotice error={commonOperatingPicture.error} title="Common operating picture unavailable" />
          <ErrorNotice error={calibrationReports.error} title="Calibration reports unavailable" />
          <ErrorNotice error={safetyDossier.error} title="Safety dossier unavailable" />
          <QueryNotice query={points} label="Reference points" />
          <MutationNotice mutation={previewImport} label="Preview import" />
          <MutationNotice mutation={applyImport} label="Apply import" />
          <MutationNotice mutation={addPoint} label="Add reference point" />
          <MutationNotice mutation={pollProviderWeather} label="Provider weather poll" />
          <MutationNotice mutation={runCalibration} label="Calibration run" />
          <MutationNotice mutation={createCollaborationProposal} label="Create collaboration proposal" />
        </div>
        {selected === 'navaids' && <ReferenceTable rows={(points.data ?? []).filter((point) => point.pointType === 'NAVAID')} onAdd={() => addPoint.mutate()} />}
        {selected === 'fixes' && <ReferenceTable rows={(points.data ?? []).filter((point) => point.pointType !== 'NAVAID')} onAdd={() => addPoint.mutate()} />}
        {selected === 'airspace' && <AirspaceGroupsPanel />}
        {selected === 'recipients' && <RecipientsPanel />}
        {selected === 'users' && <UsersPanel />}
        {selected === 'separation' && <SeparationPanel />}
        {selected === 'imports' && (
          <>
            <h3>Reference Import</h3>
            <textarea className="compact-textarea" value={referenceImport} onChange={(event) => setReferenceImport(event.target.value)} />
            <div className="toolbar compact">
              <button className="secondary" onClick={() => previewImport.mutate()}>Preview Import</button>
              <button onClick={() => applyImport.mutate()}>Apply Import</button>
            </div>
            {(previewImport.data || applyImport.data) && <pre className="raw-panel">{JSON.stringify(previewImport.data ?? applyImport.data, null, 2)}</pre>}
          </>
        )}
        {selected === 'providers' && (
          <ProvidersPanel
            providers={providers.data}
            gates={releaseGates.data}
            onPollWeather={() => pollProviderWeather.mutate()}
            pollPending={pollProviderWeather.isPending}
          />
        )}
        {selected === 'collaboration' && (
          <CollaborationPanel
            commonOperatingPicture={commonOperatingPicture.data}
            onCreateProposal={() => createCollaborationProposal.mutate()}
            createPending={createCollaborationProposal.isPending}
          />
        )}
        {selected === 'calibration' && (
          <CalibrationPanel
            reports={calibrationReports.data}
            gates={releaseGates.data}
            gaps={gaps.data}
            onRunCalibration={() => runCalibration.mutate()}
            runPending={runCalibration.isPending}
          />
        )}
        {selected === 'safety' && <SafetyDossierPanel dossier={safetyDossier.data} gaps={gaps.data} />}
        {selected === 'system' && (
          <SystemConfigurationPanel
            config={config.data}
            metrics={metrics.data}
            agentStatus={agentStatus.data}
            agentMetrics={agentMetrics.data}
          />
        )}
      </main>
    </section>
  );
}

function SystemConfigurationPanel({
  config,
  metrics,
  agentStatus,
  agentMetrics
}: {
  config?: Record<string, unknown>;
  metrics?: Record<string, number>;
  agentStatus?: { mode?: string; durable?: boolean; path?: string; runCount?: number; taskCount?: number; latestRunAt?: string };
  agentMetrics?: Record<string, number>;
}) {
  const agentStoreDurable = agentStatus?.durable ?? config?.agenticStoreDurable === true;
  const productMetricRows = Object.entries(metrics ?? {})
    .filter(([key]) => key.startsWith('product.') || key.startsWith('agentic.'))
    .sort(([a], [b]) => a.localeCompare(b))
    .map(([key, value]) => [key, formatNumber(value)]);
  const agentMetricRows = Object.entries(agentMetrics ?? {})
    .sort(([a], [b]) => a.localeCompare(b))
    .map(([key, value]) => [key, formatNumber(value)]);
  return (
    <section>
      <div className="panel-heading"><h3>System Configuration</h3><span>runtime + observability</span></div>
      <div className="config-system-grid">
        <div className="metric-card">
          <span>Agentic Store</span>
          <strong>{agentStatus?.mode ?? String(config?.agenticStore ?? 'UNKNOWN')}</strong>
          <small>{agentStoreDurable ? 'Restart-safe history' : 'Memory-only history'}</small>
          {agentStatus?.path && <small>{agentStatus.path}</small>}
        </div>
        <div className="metric-card">
          <span>Agent Runs</span>
          <strong>{agentStatus?.runCount ?? agentMetrics?.['agentic.runs'] ?? 0}</strong>
          <small>{agentMetrics?.['agentic.runs.accepted'] ?? 0} accepted · {agentMetrics?.['agentic.runs.rejected'] ?? 0} rejected</small>
        </div>
        <div className="metric-card">
          <span>Agent Tasks</span>
          <strong>{agentStatus?.taskCount ?? agentMetrics?.['agentic.tasks'] ?? 0}</strong>
          <small>{agentMetrics?.['agentic.tasks.status.acknowledged'] ?? 0} acknowledged · {agentMetrics?.['agentic.tasks.status.open'] ?? 0} open</small>
        </div>
        <div className="metric-card">
          <span>Guidance Quality</span>
          <strong>{formatNumber(metrics?.['product.weather.guidanceTargetRate'] ?? 0)}</strong>
          <small>{formatNumber(metrics?.['product.weather.averageGuidanceLatencySeconds'] ?? 0)} sec avg latency</small>
        </div>
      </div>
      {!agentStoreDurable && (
        <p className="warning-text">Agent history is memory-only. Configure <code>airspace.agentic.store.path</code> for restart-safe local/demo agent audit history.</p>
      )}
      <div className="config-split">
        <ConfigTable title="Product + Agent Metrics" head={['Metric', 'Value']} rows={productMetricRows} />
        <ConfigTable title="Agent Metrics" head={['Metric', 'Value']} rows={agentMetricRows} />
      </div>
      <details>
        <summary>Raw runtime config</summary>
        <pre className="raw-panel">{JSON.stringify(config ?? {}, null, 2)}</pre>
      </details>
    </section>
  );
}

function CollaborationPanel({
  commonOperatingPicture,
  onCreateProposal,
  createPending
}: {
  commonOperatingPicture?: CommonOperatingPictureSummary;
  onCreateProposal: () => void;
  createPending: boolean;
}) {
  const proposals = commonOperatingPicture?.proposals ?? [];
  return (
    <section>
      <div className="panel-heading">
        <h3>Collaborative Decision / Common Operating Picture</h3>
        <button onClick={onCreateProposal} disabled={createPending}>Create local proposal</button>
      </div>
      <p className="muted">Local CDM-style workflow for proposal review, stakeholder comments, approval, and delivery receipts. It does not synchronize official FAA CDM/SWIM state or transmit external messages.</p>
      <div className="config-system-grid">
        <div className="metric-card">
          <span>Sync Status</span>
          <strong>{commonOperatingPicture?.syncStatus ?? 'UNKNOWN'}</strong>
          <small>{commonOperatingPicture?.sourceMode ?? 'LOCAL'}</small>
        </div>
        <div className="metric-card">
          <span>Affected Missions</span>
          <strong>{commonOperatingPicture?.affectedMissionCount ?? 0}</strong>
          <small>{commonOperatingPicture?.activeMissionCount ?? 0} active missions</small>
        </div>
        <div className="metric-card">
          <span>Pending Approvals</span>
          <strong>{commonOperatingPicture?.pendingApprovalCount ?? 0}</strong>
          <small>{commonOperatingPicture?.activeProposalCount ?? 0} active proposal(s)</small>
        </div>
        <div className="metric-card">
          <span>Receipts</span>
          <strong>{commonOperatingPicture?.deliveredReceiptCount ?? 0}</strong>
          <small>{commonOperatingPicture?.providerCount ?? 0} provider seams · {commonOperatingPicture?.staleProviderCount ?? 0} stale</small>
        </div>
      </div>
      <div className="config-split">
        <div>
          <h4>Participants</h4>
          {(commonOperatingPicture?.participants ?? []).map((participant) => (
            <article className="event supplement" key={participant.participantId}>
              <div className="panel-heading compact">
                <h4>{participant.displayName ?? participant.participantId}</h4>
                <span className="status-badge monitor">{participant.role}</span>
              </div>
              <p>{participant.organization} · {participant.facility} · {participant.channel}</p>
              <small>{participant.canApprove ? 'Can approve' : 'Review only'} · {participant.canDeliver ? 'Can record receipt' : 'No delivery receipt'}</small>
            </article>
          ))}
        </div>
        <div>
          <h4>Proposals</h4>
          {proposals.map((proposal) => <CollaborationProposalCard key={proposal.id} proposal={proposal} />)}
          {proposals.length === 0 && <p className="empty-state">No collaboration proposals yet. Create a local proposal to exercise the review flow.</p>}
        </div>
      </div>
      {!!commonOperatingPicture?.diagnostics?.length && (
        <div className="notice-strip">
          {commonOperatingPicture.diagnostics.join(' ')}
        </div>
      )}
    </section>
  );
}

function CollaborationProposalCard({ proposal }: { proposal: CollaborativeProposalSummary }) {
  return (
    <article className="event supplement">
      <div className="panel-heading compact">
        <h4>{proposal.summary ?? proposal.id}</h4>
        <span className={`status-badge ${proposal.state === 'DELIVERED_BY_OPERATOR' ? 'clear' : proposal.state === 'REJECTED' ? 'blocked' : 'monitor'}`}>{proposal.state}</span>
      </div>
      <p>{proposal.proposalType} · {proposal.recommendedAction}</p>
      <small>{proposal.rationale}</small>
      <small>Recipients: {proposal.recipientParticipantIds?.join(', ') || 'none'}</small>
      <small>Comments {proposal.comments?.length ?? 0} · approvals {proposal.approvals?.length ?? 0} · receipts {proposal.deliveryReceipts?.length ?? 0}</small>
      {!!proposal.sourceRefs?.length && <small>Sources: {proposal.sourceRefs.slice(0, 5).join(', ')}</small>}
    </article>
  );
}

function ProvidersPanel({
  providers,
  gates,
  onPollWeather,
  pollPending
}: {
  providers?: ProviderHealthSummary[];
  gates?: ReleaseGateSummary[];
  onPollWeather: () => void;
  pollPending: boolean;
}) {
  const operationalGate = gates?.find((gate) => gate.id === 'operational-evaluation-ready');
  return (
    <section>
      <div className="panel-heading">
        <h3>Provider Status / Freshness</h3>
        <button onClick={onPollWeather} disabled={pollPending}>Poll AWC seam</button>
      </div>
      <p className="muted">Live operational providers stay disabled until credentials, consent scopes, egress policy, and reviewer approval exist. AWC public weather is configuration-gated and non-certified.</p>
      {operationalGate && (
        <div className="metric-card">
          <span>Operational Evaluation Gate</span>
          <strong>{operationalGate.status}</strong>
          <small>{operationalGate.summary}</small>
        </div>
      )}
      <div className="provider-grid">
        {(providers ?? []).map((provider) => (
          <article className="event supplement" key={provider.id}>
            <div className="panel-heading compact">
              <h4>{provider.label ?? provider.id}</h4>
              <span className={`status-badge ${provider.enabled ? 'clear' : 'monitor'}`}>{provider.enabled ? 'ENABLED' : 'DISABLED'}</span>
            </div>
            <p>{provider.sourceMode} · {provider.providerType} · {provider.egressPolicy}</p>
            <small>{provider.credentialRequirement} · {provider.consentScope}</small>
            <small>Freshness {provider.freshness?.status ?? 'UNKNOWN'}{provider.freshness?.ageSeconds !== undefined ? ` · ${provider.freshness.ageSeconds}s old` : ''}</small>
            {provider.endpoint && <small>{provider.endpoint}</small>}
            {provider.diagnostics?.map((diagnostic) => <small className="warning-text" key={diagnostic}>{diagnostic}</small>)}
          </article>
        ))}
        {(!providers || providers.length === 0) && <p className="empty-state">Provider status unavailable.</p>}
      </div>
    </section>
  );
}

function CalibrationPanel({
  reports,
  gates,
  gaps,
  onRunCalibration,
  runPending
}: {
  reports?: CalibrationRunSummary[];
  gates?: ReleaseGateSummary[];
  gaps?: AirspaceGapStatus[];
  onRunCalibration: () => void;
  runPending: boolean;
}) {
  return (
    <section>
      <div className="panel-heading">
        <h3>Calibration / Release Gates</h3>
        <button onClick={onRunCalibration} disabled={runPending}>Run fixture calibration</button>
      </div>
      <div className="config-split">
        <div>
          <h4>Release Gates</h4>
          {(gates ?? []).map((gate) => (
            <article className="event supplement" key={gate.id}>
              <span className={`status-badge ${gate.passed ? 'clear' : 'blocked'}`}>{gate.status}</span>
              <strong>{gate.label ?? gate.id}</strong>
              <p>{gate.summary}</p>
              {!!gate.blockingGapIds?.length && <small>Blocking gaps: {gate.blockingGapIds.join(', ')}</small>}
              {!!gate.excludedClaims?.length && <small>Excluded claims: {gate.excludedClaims.slice(0, 3).join('; ')}</small>}
            </article>
          ))}
        </div>
        <div>
          <h4>Open Calibration Gaps</h4>
          {(gaps ?? []).filter((gap) => gap.id === 'calibration' || gap.id === 'route-avoidance' || gap.id === 'scale').map((gap) => (
            <article className="event supplement" key={gap.id}>
              <span className="status-badge monitor">{gap.status}</span>
              <strong>{gap.category}</strong>
              <p>{gap.summary}</p>
              <small>{gap.nextStep}</small>
            </article>
          ))}
        </div>
      </div>
      {(reports ?? []).map((run) => (
        <article className="panel" key={run.id}>
          <div className="panel-heading">
            <h4>{run.datasetId}</h4>
            <span>{run.routeImpactReport?.calibrationVersion ?? 'fixture'}</span>
          </div>
          <p>{run.routeImpactReport?.summary}</p>
          <div className="source-ref-grid">
            <span>{run.routeImpactReport?.routeOutcomeCount ?? 0} route outcomes</span>
            <span>{run.routeImpactReport?.weatherOutcomeCount ?? 0} weather outcomes</span>
            <span>{run.routeImpactReport?.pirepOutcomeCount ?? 0} PIREP outcomes</span>
            <span>{run.routeImpactReport?.uncalibratedCoefficientCount ?? 0} uncalibrated coefficient(s)</span>
          </div>
          {run.routeImpactReport?.uncalibratedCoefficients?.map((item) => <small className="warning-text" key={item}>{item}</small>)}
        </article>
      ))}
    </section>
  );
}

function SafetyDossierPanel({ dossier, gaps }: { dossier?: SafetyCaseDossierSummary; gaps?: AirspaceGapStatus[] }) {
  return (
    <section>
      <div className="panel-heading"><h3>Safety Dossier</h3><span>{dossier?.releaseGate ?? 'loading'}</span></div>
      <p className="warning-text">{dossier?.summary ?? 'Safety dossier unavailable.'}</p>
      <div className="config-split">
        <ConfigList title="Scenarios Tested" items={dossier?.scenariosTested} />
        <ConfigList title="Human Review Checkpoints" items={dossier?.humanReviewCheckpoints} />
        <ConfigList title="Rejected Overclaims" items={dossier?.rejectedOverclaims} />
        <ConfigList title="Known External Blocks" items={(gaps ?? []).filter((gap) => gap.externallyBlocked).map((gap) => `${gap.category}: ${gap.nextStep}`)} />
      </div>
    </section>
  );
}

function ConfigList({ title, items }: { title: string; items?: string[] }) {
  return (
    <div>
      <h4>{title}</h4>
      <ul className="compact-list">
        {(items ?? []).map((item) => <li key={item}>{item}</li>)}
        {(!items || items.length === 0) && <li className="muted">None</li>}
      </ul>
    </div>
  );
}

function formatNumber(value: number) {
  if (!Number.isFinite(value)) return '0';
  return Math.abs(value) >= 100 || Number.isInteger(value) ? String(Math.round(value)) : value.toFixed(2);
}

function AirspaceGroupsPanel() {
  return (
    <ConfigTable
      title="Airspace Groups"
      head={['Group', 'Name', 'Owner', 'Class', 'Members']}
      rows={AIRSPACE_GROUPS.map((group) => [group.id, group.name, group.owner, group.classification, String(group.members)])}
      note="Local operational grouping surface. Backend persistence can replace these reference rows when an airspace-group endpoint is added."
    />
  );
}

function RecipientsPanel() {
  return (
    <ConfigTable
      title="Recipients"
      head={['ID', 'Handle', 'Channel', 'Address', 'Priority']}
      rows={RECIPIENTS.map((recipient) => [recipient.id, recipient.handle, recipient.channel, recipient.address, recipient.priority])}
      note="Recipient directory mirrors the legacy CARF workflow shape; message send/reply/forward still uses the real Airspace message API."
    />
  );
}

function UsersPanel() {
  return (
    <section>
      <div className="panel-heading"><h3>Users / RBAC</h3><span>local role matrix</span></div>
      <div className="config-split">
        <ConfigTable title="Users" head={['ID', 'Username', 'Role', 'Status']} rows={USERS.map((user) => [user.id, user.username, user.role, user.status])} />
        <div>
          <h4>Permission Matrix</h4>
          <table className="data-table">
            <thead><tr><th>Permission</th><th>Planner</th><th>Controller</th><th>Approver</th><th>Admin</th></tr></thead>
            <tbody>
              {PERMISSIONS.map((permission) => (
                <tr key={permission}>
                  <td>{permission}</td>
                  <td>{permission.startsWith('reservation.') ? 'Y' : '-'}</td>
                  <td>{permission.startsWith('deconflict') || permission === 'message.transmit' ? 'Y' : '-'}</td>
                  <td>{permission.includes('approve') || permission.includes('reject') || permission === 'deconflict.force' ? 'Y' : '-'}</td>
                  <td>Y</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      </div>
    </section>
  );
}

function SeparationPanel() {
  const [lateral, setLateral] = useState(120);
  const [vertical, setVertical] = useState(2000);
  const [time, setTime] = useState(15);
  return (
    <section>
      <div className="panel-heading"><h3>Flight Path Separation</h3><span>operator policy preview</span></div>
      <div className="policy-grid">
        <label>Lateral NM<input type="number" value={lateral} onChange={(event) => setLateral(Number(event.target.value))} /></label>
        <label>Vertical FT<input type="number" value={vertical} onChange={(event) => setVertical(Number(event.target.value))} /></label>
        <label>Minimum Duration MIN<input type="number" value={time} onChange={(event) => setTime(Number(event.target.value))} /></label>
      </div>
      <p className="muted">These controls mirror the legacy CARF configuration workflow. The authoritative deconfliction thresholds still come from the backend engine/config APIs.</p>
    </section>
  );
}

function ConfigTable({ title, head, rows, note }: { title: string; head: string[]; rows: string[][]; note?: string }) {
  return (
    <section>
      <div className="panel-heading"><h3>{title}</h3><span>{rows.length}</span></div>
      {note && <p className="muted">{note}</p>}
      <table className="data-table">
        <thead><tr>{head.map((item) => <th key={item}>{item}</th>)}</tr></thead>
        <tbody>
          {rows.map((row) => (
            <tr key={row.join(':')}>{row.map((cell, index) => <td key={`${cell}:${index}`}>{cell}</td>)}</tr>
          ))}
        </tbody>
      </table>
    </section>
  );
}

function ReferenceTable({ rows, onAdd }: { rows: ReferencePointSummary[]; onAdd: () => void }) {
  const column = createColumnHelper<ReferencePointSummary>();
  const columns = [
    column.accessor('identifier', { header: 'Identifier' }),
    column.accessor('pointType', { header: 'Type' }),
    column.accessor('latitude', { header: 'Lat' }),
    column.accessor('longitude', { header: 'Lon' }),
    column.accessor('altitudeFeet', { header: 'Alt Ft' }),
    column.accessor('source', { header: 'Source' })
  ];
  return (
    <>
      <div className="panel-heading"><h3>Reference Points</h3><button onClick={onAdd}>Add Local Fix</button></div>
      <DataTable data={rows} columns={columns} />
    </>
  );
}
