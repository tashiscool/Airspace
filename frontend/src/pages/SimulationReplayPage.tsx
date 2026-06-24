import { useMemo, useState } from 'react';
import { Link, useParams } from 'react-router-dom';
import { useQuery } from '@tanstack/react-query';
import { Clock3, MapPinned, Plane, RadioTower, ShieldAlert } from 'lucide-react';
import { api } from '../api/client';
import { OperationsMap } from '../components/OperationsMap';
import { ErrorNotice, QueryNotice } from '../components/Notices';
import { StatusBadge } from '../components/StatusBadge';
import { fmtZ } from '../lib/viewModels';
import type { SimulationStepResult, SimulationTick } from '../types';

export function SimulationReplayPage() {
  const { runId = '' } = useParams();
  const replay = useQuery({ queryKey: ['simulation', 'replay', runId], queryFn: () => api.simulationReplay(runId), enabled: !!runId });
  const worldState = useQuery({ queryKey: ['simulation', 'world-state', runId], queryFn: () => api.simulationWorldState(runId), enabled: !!runId });
  const [selectedMinute, setSelectedMinute] = useState<number | undefined>();
  const steps = replay.data?.steps ?? [];
  const ticks = worldState.data?.ticks ?? replay.data?.worldState?.ticks ?? [];
  const selectedStep = useMemo(() => {
    if (!steps.length) return undefined;
    if (selectedMinute == null) return steps[steps.length - 1];
    return steps.find((step) => step.offsetMinutes === selectedMinute) ?? steps[steps.length - 1];
  }, [steps, selectedMinute]);
  const selectedTick = useMemo(() => {
    if (!ticks.length) return undefined;
    const minute = selectedStep?.offsetMinutes ?? selectedMinute;
    if (minute == null) return ticks[ticks.length - 1];
    return ticks.find((tick) => tick.minute === minute) ?? ticks[ticks.length - 1];
  }, [ticks, selectedStep?.offsetMinutes, selectedMinute]);
  const mapFeatures = selectedStep?.features;

  return (
    <section className="simulation-workbench">
      <aside className="decision-queue">
        <header>Replay Ticks</header>
        <Link className="secondary action-link" to="/simulation">Back to Simulation</Link>
        {steps.map((step) => (
          <button
            key={step.id}
            type="button"
            className={step.offsetMinutes === selectedStep?.offsetMinutes ? 'queue-item active' : 'queue-item'}
            onClick={() => setSelectedMinute(step.offsetMinutes)}
          >
            <span>T+{step.offsetMinutes} · {step.injectedEvent.family}</span>
            <StatusBadge value={step.engineAction} />
            <small>{step.injectedEvent.label}</small>
          </button>
        ))}
        {!steps.length && <p className="empty-state">Run a scenario first, then open its replay.</p>}
      </aside>
      <main className="decision-detail">
        <div className="page-header">
          <div>
            <h2><Clock3 size={18} /> Simulation Replay Viewer</h2>
            <p>Tick-by-tick world-state playback for route impacts, aircraft positions, runway state, sector workload, behavior, and source artifacts.</p>
          </div>
          <StatusBadge value={replay.data?.finalAction ?? 'REPLAY'} />
        </div>
        <div className="notice-stack">
          <QueryNotice query={replay} label="Replay bundle" />
          <QueryNotice query={worldState} label="World state" />
          <ErrorNotice error={replay.error ?? worldState.error} title="Replay load failed" />
        </div>
        <section className="safety-loop-grid">
          <ReplayMetric title="Run" value={runId || '—'} detail={replay.data?.id ?? 'No replay bundle loaded.'} />
          <ReplayMetric title="Ticks" value={String(ticks.length)} detail={`${worldState.data?.tickIntervalSeconds ?? replay.data?.worldState?.tickIntervalSeconds ?? 0}s interval`} />
          <ReplayMetric title="Seed" value={String(worldState.data?.randomSeed ?? replay.data?.worldState?.randomSeed ?? '—')} detail="Deterministic local replay seed." />
          <ReplayMetric title="Sources" value={String(selectedTick?.sourceRefs.length ?? 0)} detail="Source refs preserved for selected tick." />
        </section>
        <div className="simulation-grid">
          <section className="panel simulation-map-panel">
            <div className="panel-heading"><h3><MapPinned size={15} /> Replay Map</h3><span>{selectedStep ? `T+${selectedStep.offsetMinutes}` : 'not loaded'}</span></div>
            <OperationsMap features={mapFeatures} selectedFeatureId={selectedStep ? `simulation-event-${selectedStep.injectedEvent.id}` : undefined} />
          </section>
          <section className="panel">
            <div className="panel-heading"><h3><Plane size={15} /> Aircraft State</h3><span>{selectedTick?.aircraft.length ?? 0} aircraft</span></div>
            <TickAircraft tick={selectedTick} />
          </section>
          <section className="panel">
            <div className="panel-heading"><h3><RadioTower size={15} /> ATC / Airport State</h3><span>{selectedTick?.sectorWorkload?.capacityState ?? '—'}</span></div>
            <TickOps tick={selectedTick} />
          </section>
          <section className="panel wide">
            <div className="panel-heading"><h3><ShieldAlert size={15} /> Guidance And Evidence</h3><span>{selectedStep?.engineAction ?? '—'}</span></div>
            <ReplayEvidence step={selectedStep} tick={selectedTick} />
          </section>
        </div>
      </main>
    </section>
  );
}

function ReplayMetric({ title, value, detail }: { title: string; value: string; detail: string }) {
  return (
    <article className="metric-card">
      <span>{title}</span>
      <strong>{value}</strong>
      <small>{detail}</small>
    </article>
  );
}

function TickAircraft({ tick }: { tick?: SimulationTick }) {
  if (!tick?.aircraft.length) return <p className="empty-state">No aircraft state in this tick.</p>;
  return (
    <div className="timeline-list">
      {tick.aircraft.map((aircraft) => (
        <article className="event supplement" key={aircraft.id ?? aircraft.callsign}>
          <strong>{aircraft.callsign ?? aircraft.id}</strong>
          <p>{aircraft.aircraftClass} · {aircraft.trajectory?.phase ?? 'SIM'} · {Math.round(aircraft.trajectory?.altitudeFeet ?? 0)} ft · {Math.round(aircraft.trajectory?.groundSpeedKnots ?? 0)} kt</p>
          <small>{aircraft.rerouteAssignment ?? 'No reroute'} · {aircraft.impacted ? 'impacted' : 'clear'}</small>
        </article>
      ))}
    </div>
  );
}

function TickOps({ tick }: { tick?: SimulationTick }) {
  if (!tick) return <p className="empty-state">No world state selected.</p>;
  const runway = tick.airportOps?.runwayStates?.[0];
  const procedure = tick.airportOps?.surfaceProcedures?.[0];
  const sector = tick.sectorWorkload;
  const frequency = sector?.frequencyCongestion;
  return (
    <div className="source-ref-grid">
      <span>Runway {runway?.runwayId ?? '—'} · RVR {runway?.runwayVisualRangeFeet ?? '—'} ft · queue {runway?.queueDepth ?? 0}</span>
      <span>{procedure?.localProcedureName ?? 'Procedure'} · {procedure?.confirmationStatus ?? 'simulated'}</span>
      <span>Sector {sector?.sectorId ?? '—'} · {Math.round((sector?.workloadRatio ?? 0) * 100)}% · {sector?.capacityState ?? 'unknown'}</span>
      <span>Frequency {frequency?.frequencyId ?? '—'} · {Math.round((frequency?.utilization ?? 0) * 100)}% · {frequency?.congestionState ?? 'unknown'}</span>
    </div>
  );
}

function ReplayEvidence({ step, tick }: { step?: SimulationStepResult; tick?: SimulationTick }) {
  if (!step || !tick) return <p className="empty-state">No replay evidence selected.</p>;
  return (
    <>
      <div className="source-ref-grid">
        <span>Action {step.engineAction}</span>
        <span>Recommended {step.recommendedAction ?? 'review'}</span>
        <span>Confidence {Math.round(step.confidence * 100)}%</span>
        <span>Simulated {fmtZ(step.simulatedTime)}</span>
      </div>
      <h4>Source refs</h4>
      <div className="map-source-ref-row">
        {tick.sourceRefs.map((ref) => <span key={ref}>{ref}</span>)}
      </div>
      <h4>Injected event</h4>
      <pre className="raw-panel compact">{step.injectedEvent.payload}</pre>
      <h4>Replay hashes</h4>
      <pre className="raw-panel compact">{JSON.stringify(tick.replayHashes, null, 2)}</pre>
    </>
  );
}
