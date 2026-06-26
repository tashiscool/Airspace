import { useEffect, useMemo, useState, type ReactNode } from 'react';
import { useQuery } from '@tanstack/react-query';
import { BarChart3, Clock, Database, FileText, MapPinned, Play, Route, ShieldAlert } from 'lucide-react';
import { api } from '../api/client';
import { AgentRouteContextPanel } from '../components/AgentRouteContextPanel';
import { ErrorNotice, QueryNotice } from '../components/Notices';
import { StatusBadge } from '../components/StatusBadge';
import { writeWorkbenchJson, type WorkbenchSelection } from '../lib/workbenchState';
import type {
  TfmActiveConstraintSummary,
  TfmAirportDemandSummary,
  TfmCommandCenterRequest,
  TfmProposedTmiSummary,
  TfmRouteAlternativeSummary,
  TfmSectorLoadSummary
} from '../types';

type DraftConfig = {
  flightCount: number;
  airportCount: number;
  sectorCount: number;
  durationMinutes: number;
  tickIntervalMinutes: number;
  demandSpikeFactor: number;
  capacityReductionFactor: number;
  focusMinute: number;
};

const DEFAULT_DRAFT: DraftConfig = {
  flightCount: 1200,
  airportCount: 12,
  sectorCount: 18,
  durationMinutes: 180,
  tickIntervalMinutes: 15,
  demandSpikeFactor: 1.45,
  capacityReductionFactor: 0.70,
  focusMinute: -1
};

export function TfmBoardPage() {
  const [draft, setDraft] = useState(DEFAULT_DRAFT);
  const [request, setRequest] = useState<TfmCommandCenterRequest>(() => buildRequest(DEFAULT_DRAFT));
  const board = useQuery({ queryKey: ['tfm-board', request], queryFn: () => api.tfmBoard(request) });
  const data = board.data;
  const selectedMinute = data?.selectedMinute ?? 0;
  const activeTmiTypes = useMemo(() => {
    const values = new Set((data?.proposedTmis ?? []).map((item) => item.type).filter(Boolean));
    return [...values].slice(0, 6);
  }, [data?.proposedTmis]);

  useEffect(() => {
    if (!data) return;
    const selection: WorkbenchSelection = {
      sourceFamily: 'TFM',
      label: `TFM board T+${data.selectedMinute} · ${data.impactTotals.activeFlightCount} active flights`,
      conflictCount: data.impactTotals.overloadedAirportCount + data.impactTotals.overloadedSectorCount,
      lockState: `${data.sourceMode ?? 'LOCAL'} · ${data.impactTotals.commonOperatingPictureStatus ?? 'review'}`
    };
    writeWorkbenchJson('airspace.workbench.selection', selection);
    window.dispatchEvent(new Event('airspace-workbench-selection'));
  }, [data]);

  return (
    <section className="decision-workspace">
      <aside className="decision-queue">
        <header>TFM Board</header>
        <div className="toolbar compact">
          <button onClick={() => setRequest(buildRequest(draft))} disabled={board.isFetching}>
            <Play size={14} /> Refresh Board
          </button>
        </div>
        <div className="source-ref-grid">
          <NumberField label="Flights" value={draft.flightCount} onChange={(flightCount) => setDraft({ ...draft, flightCount })} />
          <NumberField label="Airports" value={draft.airportCount} onChange={(airportCount) => setDraft({ ...draft, airportCount })} />
          <NumberField label="Sectors" value={draft.sectorCount} onChange={(sectorCount) => setDraft({ ...draft, sectorCount })} />
          <NumberField label="Duration" value={draft.durationMinutes} onChange={(durationMinutes) => setDraft({ ...draft, durationMinutes })} />
          <NumberField label="Tick min" value={draft.tickIntervalMinutes} onChange={(tickIntervalMinutes) => setDraft({ ...draft, tickIntervalMinutes })} />
          <NumberField label="Focus min" value={draft.focusMinute} onChange={(focusMinute) => setDraft({ ...draft, focusMinute })} />
          <NumberField label="Spike" value={draft.demandSpikeFactor} step="0.05" onChange={(demandSpikeFactor) => setDraft({ ...draft, demandSpikeFactor })} />
          <NumberField label="Capacity" value={draft.capacityReductionFactor} step="0.05" onChange={(capacityReductionFactor) => setDraft({ ...draft, capacityReductionFactor })} />
        </div>
        <article className="event supplement">
          <strong>Source posture</strong>
          <p>{data?.authorizationMode ?? 'LOCAL_SYNTHETIC_ONLY'}</p>
          <small>{data?.sourceMode ?? 'Local synthetic demand/capacity preview'}</small>
        </article>
        <article className="event supplement">
          <strong>Active proposal families</strong>
          <div className="chip-row">
            {activeTmiTypes.length ? activeTmiTypes.map((type) => <span className="source-chip" key={type}>{type}</span>) : <span className="source-chip">No proposals loaded</span>}
          </div>
        </article>
      </aside>

      <main className="decision-detail">
        <div className="page-header">
          <div>
            <h2><MapPinned size={18} /> TFM Command Center Board</h2>
            <p>Airport demand, sector load, active constraints, proposed TMIs, reroute alternatives, and impact totals from local SWIM/TFMS-like replay concepts.</p>
          </div>
          <div className="toolbar compact">
            <StatusBadge value={data?.impactTotals.commonOperatingPictureStatus ?? 'LOCAL_PREVIEW'} />
            <StatusBadge value={`T+${selectedMinute}M`} />
          </div>
        </div>
        <div className="notice-stack">
          <QueryNotice query={board} label="TFM command-center board" />
          <ErrorNotice error={board.error} title="TFM board unavailable" />
        </div>
        <AgentRouteContextPanel
          title="TFM Agent Evidence"
          description="TMI recommendation audits and collaborative-decision facilitation stay visible next to the common operating picture they review."
          agentTypes={['TMI_RECOMMENDATION_AUDITOR', 'COLLABORATIVE_DECISION_FACILITATOR']}
          sourceFamily="TFM"
          taskRouteContains="/tfm"
        />

        <section className="safety-loop-grid">
          <MetricCard title="Active Flights" value={String(data?.impactTotals.activeFlightCount ?? '—')} detail={`${data?.impactTotals.flightCount ?? 0} total replay flights`} />
          <MetricCard title="Airport Load" value={ratio(data?.impactTotals.maxAirportDemandCapacityRatio)} detail={`${data?.impactTotals.overloadedAirportCount ?? 0} overloaded airport(s)`} attention={(data?.impactTotals.overloadedAirportCount ?? 0) > 0} />
          <MetricCard title="Sector Load" value={ratio(data?.impactTotals.maxSectorDemandCapacityRatio)} detail={`${data?.impactTotals.overloadedSectorCount ?? 0} overloaded sector(s)`} attention={(data?.impactTotals.overloadedSectorCount ?? 0) > 0} />
          <MetricCard title="Delay" value={`${data?.impactTotals.totalExpectedDelayMinutes ?? 0}m`} detail="Expected delay in selected snapshot" />
          <MetricCard title="TMIs" value={String(data?.impactTotals.proposedTmiCount ?? '—')} detail={`${data?.impactTotals.activeConstraintCount ?? 0} active constraint(s)`} />
          <MetricCard title="Affected Flights" value={String(data?.impactTotals.affectedFlightCount ?? '—')} detail="Preserved in proposal source refs" />
          <MetricCard title="Route Options" value={String(data?.impactTotals.routeAlternativeCount ?? '—')} detail="Human-reviewed alternatives only" />
          <MetricCard title="Freshness" value={data?.impactTotals.sourceFreshnessStatus ?? '—'} detail={data?.sourceMode ?? 'local'} />
        </section>

        <div className="simulation-grid">
          <section className="panel">
            <div className="panel-heading">
              <h3><BarChart3 size={15} /> Airport Demand</h3>
              <span>{data?.airportDemand.length ?? 0} row(s)</span>
            </div>
            <AirportTable rows={data?.airportDemand ?? []} />
          </section>
          <section className="panel">
            <div className="panel-heading">
              <h3><Clock size={15} /> Sector Load</h3>
              <span>{data?.sectorLoad.length ?? 0} row(s)</span>
            </div>
            <SectorTable rows={data?.sectorLoad ?? []} />
          </section>
          <section className="panel">
            <div className="panel-heading">
              <h3><ShieldAlert size={15} /> Active Constraints</h3>
              <span>{data?.activeConstraints.length ?? 0} active</span>
            </div>
            <ConstraintTable rows={data?.activeConstraints ?? []} />
          </section>
          <section className="panel">
            <div className="panel-heading">
              <h3><FileText size={15} /> Proposed TMIs</h3>
              <span>review required</span>
            </div>
            <ProposalTable rows={data?.proposedTmis ?? []} />
          </section>
          <section className="panel wide">
            <div className="panel-heading">
              <h3><Route size={15} /> Route Alternatives</h3>
              <span>{data?.routeAlternatives.length ?? 0} candidate(s)</span>
            </div>
            <RouteAlternativeTable rows={data?.routeAlternatives ?? []} />
          </section>
          <section className="panel">
            <div className="panel-heading">
              <h3><Database size={15} /> Human Factors / Assumptions</h3>
              <span>{data?.sourceRefs.length ?? 0} source ref(s)</span>
            </div>
            <ul className="compact-list">
              {(data?.humanFactorsNotes ?? []).map((note) => <li key={note}>{note}</li>)}
              {(data?.assumptions ?? []).map((note) => <li key={note}>{note}</li>)}
            </ul>
            <SourceRefs refs={data?.sourceRefs ?? []} />
          </section>
        </div>
      </main>
    </section>
  );
}

function buildRequest(draft: DraftConfig): TfmCommandCenterRequest {
  return {
    focusMinute: draft.focusMinute >= 0 ? draft.focusMinute : undefined,
    maxAirportRows: 12,
    maxSectorRows: 12,
    maxConstraintRows: 20,
    maxProposalRows: 20,
    maxRouteAlternatives: 12,
    demandCapacityConfig: {
      id: 'tfm-board',
      flightCount: draft.flightCount,
      airportCount: draft.airportCount,
      sectorCount: draft.sectorCount,
      durationMinutes: draft.durationMinutes,
      tickIntervalMinutes: draft.tickIntervalMinutes,
      demandSpikeFactor: draft.demandSpikeFactor,
      capacityReductionFactor: draft.capacityReductionFactor,
      includeWeatherCapacityReduction: true,
      sourceMode: 'LOCAL_SYNTHETIC_NAS_SCALE'
    }
  };
}

function NumberField({ label, value, step = '1', onChange }: { label: string; value: number; step?: string; onChange: (value: number) => void }) {
  return (
    <label>
      <span>{label}</span>
      <input
        type="number"
        step={step}
        value={value}
        onChange={(event) => onChange(Number(event.target.value))}
      />
    </label>
  );
}

function MetricCard({ title, value, detail, attention = false }: { title: string; value: string; detail: string; attention?: boolean }) {
  return (
    <article className={`metric-card ${attention ? 'attention-card' : ''}`}>
      <span>{title}</span>
      <strong>{value}</strong>
      <small>{detail}</small>
    </article>
  );
}

function AirportTable({ rows }: { rows: TfmAirportDemandSummary[] }) {
  return (
    <Table headers={['Airport', 'Demand/Cap', 'Queue', 'Delay', 'Runway', 'Sources']}>
      {rows.map((row) => (
        <tr key={`${row.airportId}-${row.offsetMinutes}`}>
          <td><strong>{row.airportId}</strong><br /><StatusBadge value={row.status ?? 'UNKNOWN'} /></td>
          <td>{row.departureDemandPerHour}/{row.departureCapacityPerHour} dep<br />{row.arrivalDemandPerHour}/{row.arrivalCapacityPerHour} arr<br /><small>{ratio(row.demandCapacityRatio)}</small></td>
          <td>{row.departureQueueDepth} dep<br />{row.arrivalQueueDepth} arr</td>
          <td>{row.averageDelayMinutes}m</td>
          <td>{row.runwayConfiguration ?? '—'}</td>
          <td><SourceRefs refs={row.sourceRefs} compact /></td>
        </tr>
      ))}
    </Table>
  );
}

function SectorTable({ rows }: { rows: TfmSectorLoadSummary[] }) {
  return (
    <Table headers={['Sector', 'Load', 'Queue', 'Frequency', 'Handoff', 'Sources']}>
      {rows.map((row) => (
        <tr key={`${row.sectorId}-${row.offsetMinutes}`}>
          <td><strong>{row.sectorId}</strong><br /><StatusBadge value={row.status ?? 'UNKNOWN'} /></td>
          <td>{row.activeAircraft}/{row.baselineCapacity}<br /><small>{ratio(row.workloadRatio)}</small></td>
          <td>{row.handoffQueueDepth}</td>
          <td>{percent(row.frequencyUtilization)}</td>
          <td>{row.estimatedHandoffDelaySeconds}s</td>
          <td><SourceRefs refs={row.sourceRefs} compact /></td>
        </tr>
      ))}
    </Table>
  );
}

function ConstraintTable({ rows }: { rows: TfmActiveConstraintSummary[] }) {
  return (
    <Table headers={['Type', 'Target', 'Window', 'Delay', 'Confidence', 'Sources']}>
      {rows.map((row) => (
        <tr key={row.id}>
          <td><StatusBadge value={row.type ?? 'TMI'} /><br /><small>{row.status}</small></td>
          <td><strong>{row.targetResourceId ?? 'network'}</strong><br /><small>{row.scope}</small><br />{row.reason}</td>
          <td>T+{row.startOffsetMinutes} to T+{row.endOffsetMinutes}</td>
          <td>{row.expectedDelayMinutes}m<br /><small>{row.affectedFlightCount} flight(s)</small></td>
          <td>{percent(row.confidence)}</td>
          <td><SourceRefs refs={row.sourceRefs} compact /></td>
        </tr>
      ))}
    </Table>
  );
}

function ProposalTable({ rows }: { rows: TfmProposedTmiSummary[] }) {
  return (
    <Table headers={['Proposal', 'Target', 'Trigger', 'Impact', 'Confidence', 'Sources']}>
      {rows.map((row) => (
        <tr key={row.id}>
          <td><StatusBadge value={row.type ?? 'TMI'} /><br /><small>{row.action}</small><br /><small>{row.status}</small></td>
          <td><strong>{row.targetResourceId ?? 'network'}</strong><br /><small>{row.targetResourceType}</small></td>
          <td>{row.trigger ?? row.rationale}</td>
          <td>{row.expectedDelayMinutes}m<br /><small>{row.affectedFlightCount} flight(s)</small><br /><small>{row.requiresHumanApproval ? 'Human approval required' : 'Advisory'}</small></td>
          <td>{percent(row.confidence)}<br /><small>{row.severity}</small></td>
          <td><SourceRefs refs={row.sourceRefs} compact /></td>
        </tr>
      ))}
    </Table>
  );
}

function RouteAlternativeTable({ rows }: { rows: TfmRouteAlternativeSummary[] }) {
  return (
    <Table headers={['Route', 'Reason', 'Window', 'Impact', 'Residual Risk', 'Sources']}>
      {rows.map((row) => (
        <tr key={row.id}>
          <td><strong>{row.routeName ?? row.id}</strong><br /><small>{row.advisoryType}</small><br />{row.routeText}</td>
          <td>{row.reason ?? 'Review alternate route against current constraints.'}</td>
          <td>T+{row.startOffsetMinutes} to T+{row.endOffsetMinutes}</td>
          <td>{row.expectedDelayMinutes}m<br /><small>{row.affectedFlightCount} flight(s)</small><br /><small>{row.routePoints.length} waypoint(s)</small></td>
          <td><StatusBadge value={row.residualRisk ?? 'REVIEW'} /><br /><small>{percent(row.confidence)}</small></td>
          <td><SourceRefs refs={row.sourceRefs} compact /></td>
        </tr>
      ))}
    </Table>
  );
}

function Table({ headers, children }: { headers: string[]; children: ReactNode }) {
  return (
    <div className="data-table-wrap">
      <table className="data-table">
        <thead>
          <tr>{headers.map((header) => <th key={header}>{header}</th>)}</tr>
        </thead>
        <tbody>
          {children}
          {!Array.isArray(children) || children.length ? null : (
            <tr><td colSpan={headers.length} className="empty-cell">No records.</td></tr>
          )}
        </tbody>
      </table>
    </div>
  );
}

function SourceRefs({ refs, compact = false }: { refs: string[]; compact?: boolean }) {
  const visible = compact ? refs.slice(0, 2) : refs.slice(0, 12);
  if (!visible.length) return <span className="muted">No source refs</span>;
  return (
    <div className="chip-row">
      {visible.map((ref) => <span className="source-chip" key={ref}>{ref}</span>)}
      {refs.length > visible.length && <span className="source-chip">+{refs.length - visible.length}</span>}
    </div>
  );
}

function ratio(value?: number) {
  if (value === undefined || Number.isNaN(value)) return '—';
  return `${value.toFixed(2)}x`;
}

function percent(value?: number) {
  if (value === undefined || Number.isNaN(value)) return '—';
  return `${Math.round(value * 100)}%`;
}
