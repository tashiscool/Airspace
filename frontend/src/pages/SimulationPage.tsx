import { useEffect, useMemo, useState } from 'react';
import { Link } from 'react-router-dom';
import { useMutation, useQuery } from '@tanstack/react-query';
import { BarChart3, Clock, Database, FileText, MapPinned, Play, Route, ShieldAlert } from 'lucide-react';
import { api } from '../api/client';
import { OperationsMap } from '../components/OperationsMap';
import { ErrorNotice, MutationNotice, QueryNotice } from '../components/Notices';
import { RouteCandidateComparisonPanel } from '../components/RouteCandidateComparisonPanel';
import { StatusBadge } from '../components/StatusBadge';
import { compactId, fmtZ } from '../lib/viewModels';
import { writeWorkbenchJson, type WorkbenchSelection } from '../lib/workbenchState';
import type { FeatureCollection, HistoricalReplayCalibrationReport, HistoricalReplayDay, HistoricalReplayLoadResult, SimulationRunResult, SimulationScenario, SimulationStepResult } from '../types';

const STORY_PRESETS = [
  'Weather Avoidance',
  'Low Visibility Procedure Ambiguity',
  'PIREP Safety Override',
  'NOTAM + Weather Compound Constraint',
  'Blocked Route With No Viable Reroute',
  'Viable Reroute With Residual Risk'
];

export function SimulationPage() {
  const scenarios = useQuery({ queryKey: ['simulation', 'scenarios'], queryFn: api.simulationScenarios });
  const historicalDays = useQuery({ queryKey: ['simulation', 'historical-replay-days'], queryFn: api.historicalReplayDays });
  const historicalCalibration = useQuery({ queryKey: ['simulation', 'historical-replay-calibration'], queryFn: () => api.historicalReplayCalibration({}) });
  const [selectedScenarioId, setSelectedScenarioId] = useState('low-vis-rvr-smgcs');
  const [selectedHistoricalDayId, setSelectedHistoricalDayId] = useState('public-like-jfk-lowvis-opsnet-bts-awc');
  const [selectedStepId, setSelectedStepId] = useState<string | undefined>();
  const [selectedFeatureId, setSelectedFeatureId] = useState<string | undefined>();
  const [campaignIterations, setCampaignIterations] = useState(3);
  const [campaignDays, setCampaignDays] = useState(5);
  const runSimulation = useMutation({
    mutationFn: () => api.runSimulation({ scenarioId: selectedScenarioId, actor: 'simulation-operator', includeSensitivity: true })
  });
  const runCampaign = useMutation({
    mutationFn: () => api.runSimulationCampaign({
      actor: 'simulation-campaign',
      includeSensitivity: true,
      iterationsPerScenario: campaignIterations,
      simulatedDayCount: campaignDays
    })
  });
  const loadHistoricalReplay = useMutation({
    mutationFn: () => api.loadHistoricalReplay({
      dayId: selectedHistoricalDayId,
      actor: 'historical-replay-operator',
      runSimulation: true,
      includeCalibrationReport: true
    })
  });
  const loadedRun = useQuery({
    queryKey: ['simulation', 'historical-replay-run', loadHistoricalReplay.data?.runId],
    queryFn: () => api.simulationRun(loadHistoricalReplay.data?.runId ?? ''),
    enabled: Boolean(loadHistoricalReplay.data?.runId)
  });
  const run = runSimulation.data ?? loadedRun.data;
  const selectedScenario = scenarios.data?.find((scenario) => scenario.id === selectedScenarioId) ?? scenarios.data?.[0];
  const selectedHistoricalDay = historicalDays.data?.find((day) => day.id === selectedHistoricalDayId) ?? historicalDays.data?.[0];
  const selectedStep = useMemo(() => {
    if (!run?.steps.length) return undefined;
    return run.steps.find((step) => step.id === selectedStepId) ?? run.steps[run.steps.length - 1];
  }, [run, selectedStepId]);
  const mapFeatures = useMemo(() => selectedStep?.features ?? emptyFeatures(), [selectedStep]);

  useEffect(() => {
    if (!run) return;
    const selection: WorkbenchSelection = {
      sourceFamily: 'SIMULATION',
      label: `${run.scenarioName ?? run.scenarioId} · ${run.finalAction}`,
      missionId: run.missionId,
      reservationId: run.reservationId,
      conflictCount: run.kpiSummary.falseClearCount + run.kpiSummary.falseBlockCount,
      lockState: `T+${selectedStep?.offsetMinutes ?? 0} · ${Math.round((selectedStep?.confidence ?? 0) * 100)}%`
    };
    writeWorkbenchJson('airspace.workbench.selection', selection);
    window.dispatchEvent(new Event('airspace-workbench-selection'));
  }, [run, selectedStep?.offsetMinutes, selectedStep?.confidence]);

  return (
    <section className="simulation-workbench">
      <aside className="decision-queue">
        <header>Simulation Library</header>
        <div className="toolbar compact">
          <button onClick={() => runSimulation.mutate()} disabled={runSimulation.isPending}><Play size={14} /> Run Scenario</button>
          <button className="secondary" onClick={() => runCampaign.mutate()} disabled={runCampaign.isPending}><BarChart3 size={14} /> Run Campaign</button>
          <Link className="secondary action-link" to="/simulation/author">Author</Link>
        </div>
        <div className="source-ref-grid">
          <label><span>Iterations</span><input value={campaignIterations} inputMode="numeric" onChange={(event) => setCampaignIterations(Math.max(1, Number(event.target.value) || 1))} /></label>
          <label><span>Days</span><input value={campaignDays} inputMode="numeric" onChange={(event) => setCampaignDays(Math.max(1, Number(event.target.value) || 1))} /></label>
        </div>
        <div className="story-preset-list">
          {STORY_PRESETS.map((story) => (
            <button
              key={story}
              className={selectedScenario?.capabilityStory === story ? 'queue-item active' : 'queue-item'}
              onClick={() => {
                const match = scenarios.data?.find((scenario) => scenario.capabilityStory === story);
                if (match) setSelectedScenarioId(match.id);
              }}
            >
              <span>{story}</span>
              <small>Capability preset</small>
            </button>
          ))}
        </div>
        {(scenarios.data ?? []).map((scenario) => (
          <button
            key={scenario.id}
            className={scenario.id === selectedScenarioId ? 'queue-item active' : 'queue-item'}
            onClick={() => setSelectedScenarioId(scenario.id)}
          >
            <span>{scenario.name}</span>
            <StatusBadge value={scenario.expectedFinalAction ?? 'SIM'} />
            <small>{scenario.narrative}</small>
          </button>
        ))}
      </aside>
      <main className="decision-detail">
        <div className="page-header">
          <div>
            <h2><MapPinned size={18} /> Aerospace Simulation Workbench</h2>
            <p>{selectedScenario?.narrative ?? 'Run a local, time-stepped operational scenario to watch source artifacts become guidance.'}</p>
          </div>
          <div className="toolbar compact">
            {run && <Link className="secondary action-link" to={`/simulation/runs/${run.id}/replay`}>Replay</Link>}
            {run && <StatusBadge value={run.finalAction} />}
          </div>
        </div>
        <div className="notice-stack">
          <QueryNotice query={scenarios} label="Simulation scenarios" />
          <QueryNotice query={historicalDays} label="Historical replay corpus" />
          <QueryNotice query={historicalCalibration} label="Historical calibration report" />
          <ErrorNotice error={runSimulation.error} title="Simulation run failed" />
          <ErrorNotice error={runCampaign.error} title="Simulation campaign failed" />
          <ErrorNotice error={historicalDays.error} title="Historical replay corpus unavailable" />
          <ErrorNotice error={historicalCalibration.error} title="Historical calibration report unavailable" />
          <ErrorNotice error={loadHistoricalReplay.error} title="Historical replay load failed" />
          <MutationNotice mutation={runSimulation} label="Run simulation" />
          <MutationNotice mutation={runCampaign} label="Run campaign" />
          <MutationNotice mutation={loadHistoricalReplay} label="Load historical replay" />
        </div>
        <section className="safety-loop-grid">
          <SimulationMetric title="Time To Guidance" value={run ? `${run.kpiSummary.timeToGuidanceSeconds}s` : '—'} detail="First non-clear/monitor advisory in the local timeline." />
          <SimulationMetric title="Source Refs" value={run ? `${Math.round(run.kpiSummary.sourceRefPreservationRate * 100)}%` : '—'} detail="Timestep source refs preserved into guidance snapshots." />
          <SimulationMetric title="Reroute Found" value={run ? `${Math.round(run.kpiSummary.rerouteFoundRate * 100)}%` : '—'} detail="Steps with at least one avoidance candidate." />
          <SimulationMetric title="Replay" value={run ? `${Math.round(run.kpiSummary.replayVerificationPassRate * 100)}%` : '—'} detail="Replay/audit IDs retained in every timestep." />
          <SimulationMetric title="Minute Steps" value={run ? `${run.kpiSummary.minuteStepCount}` : '—'} detail="True minute-by-minute dynamics snapshots in this run." />
          <SimulationMetric title="Peak Workload" value={run ? `${Math.round(run.kpiSummary.peakSectorWorkloadRatio * 100)}%` : '—'} detail="Highest simulated sector load versus baseline capacity." />
        </section>
        <div className="simulation-grid">
          <section className="panel">
            <div className="panel-heading">
              <h3><Clock size={15} /> Timeline Playback</h3>
              <span>{run?.steps.length ?? selectedScenario?.events.length ?? 0} step(s)</span>
            </div>
            {run ? (
              <Timeline steps={run.steps} selectedStep={selectedStep} onSelect={(step) => {
                setSelectedStepId(step.id);
                setSelectedFeatureId(`simulation-event-${step.injectedEvent.id}`);
              }} />
            ) : (
              <ScenarioPreview scenario={selectedScenario} />
            )}
          </section>
          <section className="panel simulation-map-panel">
            <div className="panel-heading">
              <h3>Map Playback</h3>
              <span>{selectedStep ? `T+${selectedStep.offsetMinutes}` : 'not run'}</span>
            </div>
            <OperationsMap features={mapFeatures} selectedFeatureId={selectedFeatureId} onSelectedFeatureIdChange={setSelectedFeatureId} />
          </section>
          <section className="panel">
            <div className="panel-heading"><h3><Database size={15} /> Historical Replay Corpus</h3><span>{historicalDays.data?.length ?? 0} day(s)</span></div>
            <HistoricalReplayPanel
              days={historicalDays.data ?? []}
              selectedDay={selectedHistoricalDay}
              selectedDayId={selectedHistoricalDayId}
              loadResult={loadHistoricalReplay.data}
              calibration={historicalCalibration.data}
              loading={loadHistoricalReplay.isPending}
              onSelectDay={(dayId) => setSelectedHistoricalDayId(dayId)}
              onLoad={() => loadHistoricalReplay.mutate()}
            />
          </section>
          <section className="panel">
            <div className="panel-heading"><h3><Route size={15} /> Route Impact</h3><span>{selectedStep?.engineAction ?? '—'}</span></div>
            {selectedStep?.routeImpact ? (
              <>
                <p>{selectedStep.routeImpact.rationale}</p>
                <div className="source-ref-grid">
                  <span>{selectedStep.routeImpact.impactedSegmentCount} impacted segment(s)</span>
                  <span>{selectedStep.routeImpact.blockingConstraintCount} blocking constraint(s)</span>
                  <span>{selectedStep.routeImpact.avoidanceCandidates.length} candidate(s)</span>
                  <span>{Math.round(selectedStep.routeImpact.confidence * 100)}% confidence</span>
                </div>
                <RouteCandidateComparisonPanel routeImpact={selectedStep.routeImpact} title="Simulation Reroute Candidates" />
              </>
            ) : <p className="empty-state">Run a scenario to generate route-impact playback.</p>}
          </section>
          <section className="panel wide">
            <div className="panel-heading"><h3><ShieldAlert size={15} /> Dynamics Snapshot</h3><span>{selectedStep ? `T+${selectedStep.offsetMinutes} min` : 'not run'}</span></div>
            <DynamicsSnapshot step={selectedStep} />
          </section>
          <section className="panel">
            <div className="panel-heading"><h3><FileText size={15} /> Coordination / Pilot Brief</h3><span>human reviewed</span></div>
            {selectedStep ? (
              <>
                <h4>Injected Event</h4>
                <p>{selectedStep.injectedEvent.label}</p>
                <pre className="raw-panel compact">{selectedStep.injectedEvent.payload}</pre>
                <h4>Coordination Draft</h4>
                <pre className="raw-panel compact">{selectedStep.coordinationDraft?.rawText ?? 'No draft generated.'}</pre>
                <h4>Pilot Brief</h4>
                <pre className="raw-panel compact">{selectedStep.pilotBrief?.printableText ?? 'No pilot brief generated.'}</pre>
              </>
            ) : <p className="empty-state">Run a scenario to generate draft and brief artifacts.</p>}
          </section>
          <section className="panel wide">
            <div className="panel-heading"><h3><ShieldAlert size={15} /> KPI Report</h3><span>{runCampaign.data ? 'campaign loaded' : 'single run'}</span></div>
            <KpiReport run={run} campaign={runCampaign.data} />
          </section>
        </div>
      </main>
    </section>
  );
}

function Timeline({ steps, selectedStep, onSelect }: { steps: SimulationStepResult[]; selectedStep?: SimulationStepResult; onSelect: (step: SimulationStepResult) => void }) {
  return (
    <div className="timeline-list">
      {steps.map((step) => (
        <button key={step.id} className={step.id === selectedStep?.id ? 'queue-item active' : 'queue-item'} onClick={() => onSelect(step)}>
          <span>T+{step.offsetMinutes} min · {step.injectedEvent.family}</span>
          <StatusBadge value={step.engineAction} />
          <small>{step.injectedEvent.label}</small>
          <small>{fmtZ(step.simulatedTime)}</small>
        </button>
      ))}
    </div>
  );
}

function ScenarioPreview({ scenario }: { scenario?: SimulationScenario }) {
  if (!scenario) return <p className="empty-state">No simulation scenario selected.</p>;
  return (
    <div className="timeline-list">
      {scenario.events.map((event) => (
        <article className="event supplement" key={event.id}>
          <span className="status-badge">T+{event.offsetMinutes}</span>
          <strong>{event.label}</strong>
          <p>{event.payload}</p>
          <small>{event.family} · expected {event.expectedAction}</small>
        </article>
      ))}
    </div>
  );
}

function SimulationMetric({ title, value, detail }: { title: string; value: string; detail: string }) {
  return (
    <article className="metric-card">
      <span>{title}</span>
      <strong>{value}</strong>
      <small>{detail}</small>
    </article>
  );
}

function HistoricalReplayPanel({
  days,
  selectedDay,
  selectedDayId,
  loadResult,
  calibration,
  loading,
  onSelectDay,
  onLoad
}: {
  days: HistoricalReplayDay[];
  selectedDay?: HistoricalReplayDay;
  selectedDayId: string;
  loadResult?: HistoricalReplayLoadResult;
  calibration?: HistoricalReplayCalibrationReport;
  loading: boolean;
  onSelectDay: (dayId: string) => void;
  onLoad: () => void;
}) {
  if (!days.length) {
    return <p className="empty-state">No historical replay days are available. Local fixtures are expected in development mode.</p>;
  }
  return (
    <div className="timeline-list">
      <label className="field-row">
        <span>Replay day</span>
        <select value={selectedDayId} onChange={(event) => onSelectDay(event.target.value)}>
          {days.map((day) => <option key={day.id} value={day.id}>{day.name ?? day.id}</option>)}
        </select>
      </label>
      {selectedDay && (
        <article className="event supplement">
          <div className="panel-heading inline">
            <strong>{selectedDay.name}</strong>
            <StatusBadge value={selectedDay.sourceMode ?? 'REPLAY'} />
          </div>
          <p>{selectedDay.expectedFinalAction} · {selectedDay.authorizationMode}</p>
          <div className="source-ref-grid">
            <span>{selectedDay.trafficReplay?.flightPlans.length ?? 0} flights</span>
            <span>{selectedDay.trafficReplay?.positions.length ?? 0} positions</span>
            <span>{selectedDay.expectedOutcomes.length} labels</span>
            <span>{selectedDay.airportIds.join(', ') || 'no airport'}</span>
          </div>
          <div className="chip-row">
            {selectedDay.tags.slice(0, 8).map((tag) => <span className="source-chip" key={tag}>{tag}</span>)}
          </div>
          {selectedDay.dataQualityWarnings.slice(0, 2).map((warning) => <small key={warning}>{warning}</small>)}
        </article>
      )}
      <button onClick={onLoad} disabled={loading}>
        <Play size={14} /> Load + Run Replay Day
      </button>
      {loadResult && (
        <article className="event supplement">
          <strong>{loadResult.accepted ? 'Replay accepted' : 'Replay rejected'}</strong>
          <p>{loadResult.finalAction ? `Generated ${loadResult.finalAction} run ${compactId(loadResult.runId)}` : 'Validation only.'}</p>
          <div className="source-ref-grid">
            <span>{loadResult.flightPlanCount} flight plans</span>
            <span>{loadResult.positionCount} positions</span>
            <span>{loadResult.tmiCount} TMIs</span>
            <span>{loadResult.expectedOutcomeCount} outcomes</span>
          </div>
          {loadResult.warnings.slice(0, 3).map((warning) => <small key={warning}>{warning}</small>)}
        </article>
      )}
      {calibration && (
        <article className="event supplement">
          <strong>Calibration readiness</strong>
          <p>{calibration.calibrationVersion} · {calibration.corpusDayCount} day(s) · {calibration.expectedOutcomeCount} outcome label(s)</p>
          <div className="source-ref-grid">
            <span>{Math.round(calibration.deterministicAgreementRate * 100)}% agreement labels</span>
            <span>{Math.round(calibration.sourceRefPreservationRate * 100)}% source refs</span>
            <span>{calibration.routeOutcomeCount} route outcomes</span>
            <span>{calibration.sectorCapacityOutcomeCount} capacity outcomes</span>
          </div>
          {calibration.uncalibratedCoefficients.slice(0, 3).map((gap) => <small key={gap}>{gap}</small>)}
        </article>
      )}
    </div>
  );
}

function DynamicsSnapshot({ step }: { step?: SimulationStepResult }) {
  const dynamics = step?.dynamics;
  if (!dynamics) return <p className="empty-state">Run a scenario and select a timestep to inspect dynamics.</p>;
  const aircraft = dynamics.aircraft;
  const airport = dynamics.airportSurface;
  const sector = dynamics.sectorWorkload;
  const behavior = dynamics.pilotOperator;
  const weather = dynamics.weatherEvolution;
  const traffic = dynamics.trafficReplay;
  const national = dynamics.nationalDemandCapacity;
  const tfmRecommendations = dynamics.trafficManagementRecommendations ?? [];
  return (
    <>
      <div className="source-ref-grid">
        {aircraft && <span>Aircraft {Math.round(aircraft.altitudeFeet)} ft · {Math.round(aircraft.groundSpeedKnots)} kt · fuel {Math.round(aircraft.fuelRemainingPounds)} lb</span>}
        {airport && <span>RVR {airport.runwayVisualRangeFeet} ft · SMGCS {airport.smgcsActive ? 'active' : 'review'} · delay {airport.surfaceDelaySeconds}s</span>}
        {sector && <span>Sector {Math.round(sector.workloadRatio * 100)}% · handoff {sector.estimatedHandoffDelaySeconds}s · {sector.capacityState}</span>}
        {behavior && <span>Pilot {behavior.pilotAction} · approval {behavior.humanApprovalRequired ? 'required' : 'not required'}</span>}
        {weather && <span>Weather {weather.stormPhase} · members {weather.ensembleMemberCount} · spread {Math.round(weather.probabilitySpread * 100)}%</span>}
        {traffic && <span>Traffic {traffic.sourceMode} · {traffic.replayedAircraftCount} aircraft · {traffic.airportDemandSnapshotCount} airport snapshots · {traffic.activeTmiRecommendationCount} TFM recs · live SWIM/NAS {traffic.liveSwimNasDataUsed ? 'yes' : 'no'}</span>}
        {national && <span>National {national.activeFlightCount}/{national.totalFlightCount} active · airport overloads {national.overloadedAirportCount} · sector overloads {national.overloadedSectorCount}</span>}
      </div>
      <div className="simulation-dynamics-grid">
        <DynamicsCard title="Aircraft Performance" rows={[
          ['Phase', aircraft?.performancePhase],
          ['Runway required', aircraft ? `${Math.round(aircraft.runwayRequiredFeet)} ft` : undefined],
          ['Runway available', aircraft ? `${Math.round(aircraft.runwayAvailableFeet)} ft` : undefined],
          ['Takeoff performance', aircraft?.takeoffPerformanceAcceptable ? 'acceptable' : 'review required'],
          ['Fuel burn', aircraft ? `${Math.round(aircraft.estimatedFuelBurnPounds)} lb` : undefined]
        ]} rationale={aircraft?.rationale} />
        <DynamicsCard title="Airport Surface" rows={[
          ['Airport/runway', airport ? `${airport.airportId} ${airport.runwayId}` : undefined],
          ['RVR', airport ? `${airport.runwayVisualRangeFeet} ft` : undefined],
          ['Braking', airport?.brakingAction],
          ['LVP', airport?.lowVisibilityProcedureActive ? 'active' : 'not active/review'],
          ['Terminology ambiguity', airport?.terminologyAmbiguity ? 'yes' : 'no']
        ]} rationale={airport?.rationale} />
        <DynamicsCard title="ATC Workload" rows={[
          ['Sector', sector?.sectorId],
          ['Active aircraft', sector?.activeAircraft],
          ['Baseline capacity', sector?.baselineCapacity],
          ['Frequency congestion', sector ? `${Math.round(sector.frequencyCongestion * 100)}%` : undefined],
          ['Handoff queue', sector?.handoffQueueDepth]
        ]} rationale={sector?.rationale} />
        <DynamicsCard title="Pilot / Operator Behavior" rows={[
          ['Model', behavior?.behaviorModel],
          ['Controller action', behavior?.controllerAction],
          ['Operator action', behavior?.operatorAction],
          ['Comms delay', behavior ? `${behavior.communicationDelaySeconds}s` : undefined],
          ['Acceptance', behavior ? `${Math.round(behavior.acceptanceProbability * 100)}%` : undefined]
        ]} rationale={behavior?.rationale} />
        <DynamicsCard title="Weather Ensemble" rows={[
          ['Model', weather?.evolutionModel],
          ['Movement', weather ? `${Math.round(weather.movementSpeedKnots)} kt` : undefined],
          ['Growth', weather ? `${Math.round(weather.growthRate * 100)}%` : undefined],
          ['Decay', weather ? `${Math.round(weather.decayRate * 100)}%` : undefined],
          ['Blocked mean', weather ? `${Math.round(weather.meanBlockedProbability * 100)}%` : undefined]
        ]} rationale={weather?.rationale} />
        <DynamicsCard title="Traffic Replay" rows={[
          ['Source', traffic?.replaySourceId],
          ['Mode', traffic?.sourceMode],
          ['Provider family', traffic?.providerFamily],
          ['Authorization', traffic?.authorizationMode],
          ['Aircraft', traffic?.replayedAircraftCount],
          ['Flight plans', traffic?.replayedFlightPlanCount],
          ['Positions', traffic?.replayedPositionCount],
          ['Airport demand', traffic?.airportDemandSnapshotCount],
          ['Sector demand', traffic?.sectorDemandSnapshotCount],
          ['Active TMIs', traffic?.activeTrafficManagementInitiativeCount],
          ['TMI types', traffic?.activeTmiTypes?.join(', ')],
          ['TFM recommendations', traffic?.activeTmiRecommendationCount],
          ['Fixture backed', traffic?.fixtureBacked ? 'yes' : 'no']
        ]} rationale={traffic?.rationale} />
        <DynamicsCard title="National Demand / Capacity" rows={[
          ['Active flights', national ? `${national.activeFlightCount}/${national.totalFlightCount}` : undefined],
          ['Airports / sectors', national ? `${national.airportCount} / ${national.sectorCount}` : undefined],
          ['Overloaded airports', national?.overloadedAirportCount],
          ['Overloaded sectors', national?.overloadedSectorCount],
          ['Busiest airport', national?.busiestAirportId],
          ['Busiest sector', national?.busiestSectorId],
          ['Airport peak ratio', national ? `${Math.round(national.maxAirportDemandCapacityRatio * 100)}%` : undefined],
          ['Sector peak ratio', national ? `${Math.round(national.maxSectorDemandCapacityRatio * 100)}%` : undefined],
          ['Generated TFM recs', national?.generatedTmiRecommendationCount]
        ]} rationale={national?.diagnostics?.[0]} />
      </div>
      {tfmRecommendations.length > 0 && (
        <div className="event-list compact">
          {tfmRecommendations.map((recommendation) => (
            <article className="event supplement" key={recommendation.id ?? `${recommendation.recommendedType}-${recommendation.action}`}>
              <strong>{recommendation.recommendedType ?? 'TFM'} · {recommendation.action ?? 'Review'}</strong>
              <dl className="compact-definition-list">
                <div>
                  <dt>Target</dt>
                  <dd>{[recommendation.targetResourceType, recommendation.targetResourceId].filter(Boolean).join(' ') || 'Traffic flow'}</dd>
                </div>
                <div>
                  <dt>Severity</dt>
                  <dd>{recommendation.severity ?? 'REVIEW'}</dd>
                </div>
                <div>
                  <dt>Expected delay</dt>
                  <dd>{recommendation.expectedDelayMinutes} min</dd>
                </div>
                <div>
                  <dt>Confidence</dt>
                  <dd>{Math.round((recommendation.confidence ?? 0) * 100)}%</dd>
                </div>
              </dl>
              <small>{recommendation.trigger ?? recommendation.rationale}</small>
            </article>
          ))}
        </div>
      )}
      <div className="source-ref-grid">
        {dynamics.assumptions.map((assumption) => <span key={assumption}>{assumption}</span>)}
      </div>
    </>
  );
}

function DynamicsCard({ title, rows, rationale }: { title: string; rows: Array<[string, string | number | undefined]>; rationale?: string }) {
  return (
    <article className="event supplement">
      <strong>{title}</strong>
      <dl className="compact-definition-list">
        {rows.filter(([, value]) => value != null).map(([label, value]) => (
          <div key={label}>
            <dt>{label}</dt>
            <dd>{String(value)}</dd>
          </div>
        ))}
      </dl>
      {rationale && <small>{rationale}</small>}
    </article>
  );
}

function KpiReport({ run, campaign }: { run?: SimulationRunResult; campaign?: { scenarioCount: number; passedScenarioCount: number; aggregateKpis: SimulationRunResult['kpiSummary']; runs: SimulationRunResult[] } }) {
  const kpi = campaign?.aggregateKpis ?? run?.kpiSummary;
  if (!kpi) return <p className="empty-state">Run a scenario or campaign to calculate KPIs.</p>;
  return (
    <>
      {campaign && <p>{campaign.passedScenarioCount}/{campaign.scenarioCount} scenario(s) matched expected final guidance.</p>}
      <div className="source-ref-grid">
        <span>Time to guidance {kpi.timeToGuidanceSeconds}s</span>
        <span>False clear {kpi.falseClearCount}</span>
        <span>False block {kpi.falseBlockCount}</span>
        <span>Source refs {Math.round(kpi.sourceRefPreservationRate * 100)}%</span>
        <span>Reroute found {Math.round(kpi.rerouteFoundRate * 100)}%</span>
        <span>Replay {Math.round(kpi.replayVerificationPassRate * 100)}%</span>
        <span>Minute steps {kpi.minuteStepCount}</span>
        <span>Aircraft updates {kpi.aircraftStateUpdateCount}</span>
        <span>Peak workload {Math.round(kpi.peakSectorWorkloadRatio * 100)}%</span>
        <span>Handoff max {kpi.maxHandoffDelaySeconds}s</span>
        <span>Surface max {kpi.maxSurfaceDelaySeconds}s</span>
        <span>Ensemble members {kpi.stochasticEnsembleMemberCount}</span>
        <span>Traffic replay aircraft {kpi.trafficReplayAircraftCount}</span>
        <span>National flights {kpi.nationalFlightCount}</span>
        <span>Airport overloads {kpi.overloadedAirportCount}</span>
        <span>Sector overloads {kpi.overloadedSectorCount}</span>
        <span>Airport D/C peak {Math.round(kpi.peakAirportDemandCapacityRatio * 100)}%</span>
        <span>Sector D/C peak {Math.round(kpi.peakNationalSectorDemandCapacityRatio * 100)}%</span>
        <span>National TFM recs {kpi.nationalTmiRecommendationCount}</span>
      </div>
      {campaign?.runs?.map((item) => (
        <div className="event supplement" key={item.id}>
          <strong>{item.scenarioName}</strong>
          <StatusBadge value={item.finalAction} />
          <small>{compactId(item.id)} · expected {item.expectedFinalAction}</small>
        </div>
      ))}
      {kpi.diagnostics.map((diagnostic) => <small className="warning-text" key={diagnostic}>{diagnostic}</small>)}
    </>
  );
}

function emptyFeatures(): FeatureCollection {
  return { type: 'FeatureCollection', features: [] };
}
