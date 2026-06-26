import { useEffect, useState, type ReactNode } from 'react';
import { useQuery } from '@tanstack/react-query';
import { BarChart3, Calculator, Clock, Fuel, Play, Route, ShieldCheck, TriangleAlert } from 'lucide-react';
import { api } from '../api/client';
import { AgentRouteContextPanel } from '../components/AgentRouteContextPanel';
import { ErrorNotice, QueryNotice } from '../components/Notices';
import { StatusBadge } from '../components/StatusBadge';
import { writeWorkbenchJson, type WorkbenchSelection } from '../lib/workbenchState';
import type { OutcomeMetricSummary, OutcomeMetricsReport, OutcomeMetricsRequest } from '../types';

type OutcomeDraft = {
  scenarioId: string;
  flightCount: number;
  airportCount: number;
  sectorCount: number;
  runSimulation: boolean;
};

const DEFAULT_DRAFT: OutcomeDraft = {
  scenarioId: 'oceanic-altrv-convection',
  flightCount: 900,
  airportCount: 10,
  sectorCount: 18,
  runSimulation: true
};

export function OutcomeMetricsPage() {
  const [draft, setDraft] = useState(DEFAULT_DRAFT);
  const [request, setRequest] = useState<OutcomeMetricsRequest>(() => buildRequest(DEFAULT_DRAFT));
  const report = useQuery({ queryKey: ['outcome-metrics', request], queryFn: () => api.outcomeMetrics(request) });
  const data = report.data;

  useEffect(() => {
    if (!data) return;
    const selection: WorkbenchSelection = {
      sourceFamily: 'OUTCOME',
      label: `Outcome metrics · ${data.scope ?? 'local'} · ${formatNumber(data.delayMinutesSaved)} min saved`,
      conflictCount: data.falseClearCount + data.falseBlockCount + data.overloadedSectorCount + data.overloadedAirportCount,
      lockState: `${data.sourceMode ?? 'LOCAL'} · ${data.sourceRefCompletenessRate.toFixed(2)} source-ref completeness`
    };
    writeWorkbenchJson('airspace.workbench.selection', selection);
    window.dispatchEvent(new Event('airspace-workbench-selection'));
  }, [data]);

  return (
    <section className="decision-workspace">
      <aside className="decision-queue">
        <header>Outcome Metrics</header>
        <div className="toolbar compact">
          <button onClick={() => setRequest(buildRequest(draft))} disabled={report.isFetching}>
            <Play size={14} /> Generate Report
          </button>
        </div>
        <div className="source-ref-grid">
          <label>
            <span>Scenario</span>
            <select value={draft.scenarioId} onChange={(event) => setDraft({ ...draft, scenarioId: event.target.value })}>
              <option value="oceanic-altrv-convection">Oceanic ALTRV + Convection</option>
              <option value="low-vis-rvr-smgcs">JFK Low Visibility</option>
              <option value="pirep-safety-override">PIREP Safety Override</option>
              <option value="blocked-no-viable-reroute">Blocked Route</option>
            </select>
          </label>
          <NumberField label="Flights" value={draft.flightCount} onChange={(flightCount) => setDraft({ ...draft, flightCount })} />
          <NumberField label="Airports" value={draft.airportCount} onChange={(airportCount) => setDraft({ ...draft, airportCount })} />
          <NumberField label="Sectors" value={draft.sectorCount} onChange={(sectorCount) => setDraft({ ...draft, sectorCount })} />
          <label>
            <span>Simulation</span>
            <select value={draft.runSimulation ? 'run' : 'board'} onChange={(event) => setDraft({ ...draft, runSimulation: event.target.value === 'run' })}>
              <option value="run">Run scenario + TFM board</option>
              <option value="board">TFM board only</option>
            </select>
          </label>
        </div>
        <article className="event supplement">
          <strong>Interpretation</strong>
          <p>These are local modeled outcomes for simulation, replay, and TFM review. They are not calibrated FAA ASPM/TFMS post-event measurements.</p>
        </article>
        <article className="event supplement">
          <strong>Evidence posture</strong>
          <p>{data?.sourceMode ?? 'LOCAL_SIMULATION'}</p>
          <small>{data?.sourceRefs.length ?? 0} source refs preserved</small>
        </article>
      </aside>

      <main className="decision-detail">
        <div className="page-header">
          <div>
            <h2><Calculator size={18} /> Operational Outcome Metrics</h2>
            <p>Delay, fuel, reroute, sector-overload, false-clear/false-block, source-ref, and time-to-decision metrics for local simulation and TFM review.</p>
          </div>
          <div className="toolbar compact">
            <StatusBadge value={data?.scope ?? 'LOCAL'} />
            <StatusBadge value={data?.sourceMode ?? 'FIXTURE'} />
          </div>
        </div>
        <div className="notice-stack">
          <QueryNotice query={report} label="outcome metrics" />
          <ErrorNotice error={report.error} title="Outcome metrics unavailable" />
        </div>
        <AgentRouteContextPanel
          title="Outcome Metrics Agent Evidence"
          description="Outcome-metrics audit runs track delay, fuel, reroute, false-clear/false-block, source-ref, and operator time-to-decision claims."
          agentTypes={['OUTCOME_METRICS_AUDITOR']}
          sourceFamily="OUTCOME"
          taskRouteContains="/outcomes"
        />

        <section className="safety-loop-grid">
          <MetricCard icon={<Clock size={16} />} title="Delay Saved" value={`${formatNumber(data?.delayMinutesSaved)}m`} detail={`${formatNumber(data?.baselineDelayMinutes)}m baseline · ${formatNumber(data?.mitigatedDelayMinutes)}m mitigated`} attention={(data?.delayMinutesSaved ?? 0) > 0} />
          <MetricCard icon={<Fuel size={16} />} title="Fuel Impact" value={`${formatNumber(data?.fuelImpactPounds)} lb`} detail={`${formatNumber(data?.additionalFuelPounds)} added · ${formatNumber(data?.holdingFuelSavedPounds)} avoided`} attention={(data?.fuelImpactPounds ?? 0) > 0} />
          <MetricCard icon={<Route size={16} />} title="Reroute Miles" value={`${formatNumber(data?.rerouteMiles)} NM`} detail={`${data?.routeAlternativeCount ?? 0} route alternative(s)`} />
          <MetricCard icon={<BarChart3 size={16} />} title="Overload Avoided" value={String(data?.sectorOverloadAvoidedCount ?? '—')} detail={`${data?.overloadedSectorCount ?? 0} sector · ${data?.overloadedAirportCount ?? 0} airport overload(s)`} attention={(data?.sectorOverloadAvoidedCount ?? 0) > 0} />
          <MetricCard icon={<ShieldCheck size={16} />} title="Source Refs" value={ratio(data?.sourceRefCompletenessRate)} detail={`${data?.sourceRefs.length ?? 0} evidence reference(s)`} attention={(data?.sourceRefCompletenessRate ?? 1) < 0.95} />
          <MetricCard icon={<TriangleAlert size={16} />} title="False Clear / Block" value={`${data?.falseClearCount ?? 0}/${data?.falseBlockCount ?? 0}`} detail="Scenario-labeled safety outcomes" attention={(data?.falseClearCount ?? 0) + (data?.falseBlockCount ?? 0) > 0} />
          <MetricCard icon={<Clock size={16} />} title="Time To Decision" value={seconds(data?.operatorTimeToDecisionSeconds)} detail="Guidance plus communication/review latency" attention={(data?.operatorTimeToDecisionSeconds ?? 0) > 300} />
          <MetricCard icon={<BarChart3 size={16} />} title="TMIs" value={String(data?.proposedTmiCount ?? '—')} detail={`${data?.affectedFlightCount ?? 0} affected flight(s)`} />
        </section>

        <div className="simulation-grid">
          <section className="panel wide">
            <div className="panel-heading">
              <h3><BarChart3 size={15} /> Metric Cards</h3>
              <span>{data?.metrics.length ?? 0} measured outcome(s)</span>
            </div>
            <OutcomeMetricTable rows={data?.metrics ?? []} />
          </section>
          <section className="panel">
            <div className="panel-heading">
              <h3><ShieldCheck size={15} /> Source References</h3>
              <span>{data?.sourceRefs.length ?? 0} refs</span>
            </div>
            <SourceRefs refs={data?.sourceRefs ?? []} />
          </section>
          <section className="panel">
            <div className="panel-heading">
              <h3><TriangleAlert size={15} /> Assumptions / Diagnostics</h3>
              <span>{(data?.assumptions.length ?? 0) + (data?.diagnostics.length ?? 0)} note(s)</span>
            </div>
            <ul className="compact-list">
              {(data?.assumptions ?? []).map((item) => <li key={item}>{item}</li>)}
              {(data?.diagnostics ?? []).map((item) => <li key={item}>{item}</li>)}
            </ul>
          </section>
        </div>
      </main>
    </section>
  );
}

function buildRequest(draft: OutcomeDraft): OutcomeMetricsRequest {
  return {
    scenarioId: draft.scenarioId,
    runSimulation: draft.runSimulation,
    includeTfmBoard: true,
    demandCapacityConfig: {
      id: 'outcome-workbench',
      flightCount: draft.flightCount,
      airportCount: draft.airportCount,
      sectorCount: draft.sectorCount,
      durationMinutes: 90,
      tickIntervalMinutes: 10,
      demandSpikeFactor: 1.5,
      capacityReductionFactor: 0.66,
      includeWeatherCapacityReduction: true,
      sourceMode: 'LOCAL_SYNTHETIC_NAS_SCALE'
    }
  };
}

function NumberField({ label, value, onChange }: { label: string; value: number; onChange: (value: number) => void }) {
  return (
    <label>
      <span>{label}</span>
      <input type="number" min={1} value={value} onChange={(event) => onChange(Number(event.target.value))} />
    </label>
  );
}

function MetricCard({ icon, title, value, detail, attention = false }: { icon: ReactNode; title: string; value: string; detail: string; attention?: boolean }) {
  return (
    <article className={`metric-card ${attention ? 'attention-card' : ''}`}>
      <span>{icon} {title}</span>
      <strong>{value}</strong>
      <small>{detail}</small>
    </article>
  );
}

function OutcomeMetricTable({ rows }: { rows: OutcomeMetricSummary[] }) {
  return (
    <div className="data-table-wrap">
      <table className="data-table">
        <thead>
          <tr>
            <th>Metric</th>
            <th>Value</th>
            <th>Status</th>
            <th>Rationale</th>
          </tr>
        </thead>
        <tbody>
          {rows.length ? rows.map((row) => (
            <tr key={row.id ?? row.label}>
              <td>{row.label ?? row.id}</td>
              <td>{formatNumber(row.value)} {row.unit}</td>
              <td><StatusBadge value={row.status ?? 'LOCAL'} /></td>
              <td>{row.rationale ?? 'Local modeled outcome.'}</td>
            </tr>
          )) : (
            <tr><td colSpan={4} className="muted">No outcome metrics loaded.</td></tr>
          )}
        </tbody>
      </table>
    </div>
  );
}

function SourceRefs({ refs }: { refs: string[] }) {
  if (!refs.length) return <p className="muted">No source references in this report.</p>;
  return (
    <div className="chip-row">
      {refs.slice(0, 24).map((ref) => <span className="source-chip" key={ref}>{ref}</span>)}
      {refs.length > 24 ? <span className="source-chip">+{refs.length - 24} more</span> : null}
    </div>
  );
}

function formatNumber(value?: number) {
  if (value === undefined || Number.isNaN(value)) return '—';
  return new Intl.NumberFormat(undefined, { maximumFractionDigits: Math.abs(value) < 10 ? 1 : 0 }).format(value);
}

function ratio(value?: number) {
  if (value === undefined || Number.isNaN(value)) return '—';
  return `${Math.round(value * 100)}%`;
}

function seconds(value?: number) {
  if (value === undefined || value < 0 || Number.isNaN(value)) return '—';
  return `${Math.round(value)}s`;
}
