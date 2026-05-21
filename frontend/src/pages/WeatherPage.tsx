import { useEffect, useMemo, useState } from 'react';
import type { ReactNode } from 'react';
import { useQuery } from '@tanstack/react-query';
import { createColumnHelper } from '@tanstack/react-table';
import { useNavigate } from 'react-router-dom';
import { AlertTriangle, CloudSun, Radio, Route, ShieldAlert } from 'lucide-react';
import { api } from '../api/client';
import { DataTable } from '../components/DataTable';
import { QueryNotice } from '../components/Notices';
import { OperationsMap } from '../components/OperationsMap';
import { StatusBadge } from '../components/StatusBadge';
import type { MessageSummary } from '../types';
import { fmtZ, referencePointToFeature, sourceRefLabel, sourceRefRoute, weatherChangeFeed, weatherFeatureIdForMessageId, weatherFeaturesFromMessages, weatherGuidanceFromMessage, weatherRowsFromMessages, type WeatherGuidanceItem } from '../lib/viewModels';
import { writeWorkbenchJson, type WorkbenchSelection } from '../lib/workbenchState';

export function WeatherPage() {
  const navigate = useNavigate();
  const [selectedWeatherMessageId, setSelectedWeatherMessageId] = useState<string | undefined>();
  const messages = useQuery({ queryKey: ['messages'], queryFn: api.messages });
  const reference = useQuery({ queryKey: ['reference-points'], queryFn: () => api.referencePoints() });
  const affectedMissions = useQuery({ queryKey: ['weather-affected-missions'], queryFn: () => api.affectedMissions(undefined, 25) });
  const metrics = useQuery({ queryKey: ['metrics', 'weather'], queryFn: api.metrics });
  const weatherMessages = weatherRowsFromMessages(messages.data ?? []);
  const guidance = weatherMessages.map(weatherGuidanceFromMessage);
  const changes = weatherChangeFeed(messages.data ?? [], 6);
  const severeCount = guidance.filter((item) => item.priority === 'HIGH').length;
  const pirepCount = weatherMessages.filter((message) => message.family.toUpperCase().includes('PIREP')).length;
  const routeBlockageCount = guidance.filter((item) => item.action === 'REROUTE' || item.action === 'AVOID').length;
  const targetRate = metrics.data?.['product.weather.guidanceTargetRate'];
  const averageLatency = metrics.data?.['product.weather.averageGuidanceLatencySeconds'];
  const missedTargets = metrics.data?.['product.weather.guidanceTargetMissed'] ?? 0;
  useEffect(() => {
    const selection: WorkbenchSelection = {
      sourceFamily: 'WEATHER',
      label: 'Weather & PIREP constraints',
      lockState: `${weatherMessages.length} products`
    };
    writeWorkbenchJson('airspace.workbench.selection', selection);
    window.dispatchEvent(new Event('airspace-workbench-selection'));
  }, [weatherMessages.length]);
  const weatherFeatures = useMemo(() => weatherFeaturesFromMessages(messages.data ?? []), [messages.data]);
  const selectedWeatherHasFeature = selectedWeatherMessageId
    ? weatherFeatures.some((feature) => feature.id === weatherFeatureIdForMessageId(selectedWeatherMessageId))
    : false;
  const selectedWeatherMessage = selectedWeatherMessageId
    ? weatherMessages.find((message) => message.id === selectedWeatherMessageId)
    : undefined;
  const featureCollection = {
    type: 'FeatureCollection' as const,
    features: [
      ...weatherFeatures,
      ...(reference.data ?? []).slice(0, 100).map(referencePointToFeature)
    ]
  };
  const selectedMapFeatureId = weatherFeatureIdForMessageId(selectedWeatherMessageId);
  const column = createColumnHelper<MessageSummary>();
  const columns = [
    column.accessor('family', { header: 'Product' }),
    column.accessor('status', { header: 'Status', cell: (info) => <StatusBadge value={info.getValue()} /> }),
    column.accessor('subject', { header: 'Subject' }),
    column.display({ header: 'Guidance', cell: (info) => <StatusBadge value={weatherGuidanceFromMessage(info.row.original).action} /> }),
    column.accessor('createdAt', { header: 'Received', cell: (info) => fmtZ(info.getValue()) }),
    column.accessor('rawText', { header: 'Raw' })
  ];
  return (
    <section className="workspace operational-page">
      <div className="page-header">
        <div>
          <h2><CloudSun size={18} /> Weather & PIREPs</h2>
          <p>Turns weather products, aircraft reports, and route constraints into operational guidance for avoid, reroute, delay, altitude change, monitor, or clear decisions.</p>
        </div>
        <StatusBadge value={`${weatherMessages.length} PRODUCTS`} />
      </div>
      <section className="safety-loop-grid">
        <SafetyCard
          icon={<Radio size={16} />}
          title="Real-time inputs"
          value={`${weatherMessages.length} products`}
          detail="USNS/feed/message traffic is retained with source family, status, raw text, and received time."
        />
        <SafetyCard
          icon={<Route size={16} />}
          title="Route blockage"
          value={`${routeBlockageCount} route actions`}
          detail="Convective SIGMET/CWAP-style products promote reroute or avoid guidance before they become route constraints."
          attention={routeBlockageCount > 0}
        />
        <SafetyCard
          icon={<ShieldAlert size={16} />}
          title="PIREP workflow"
          value={`${pirepCount} aircraft reports`}
          detail="PIREPs are separated from forecast products so urgent turbulence/icing reports can drive review priority."
          attention={pirepCount > 0}
        />
        <SafetyCard
          icon={<AlertTriangle size={16} />}
          title="Coordination queue"
          value={`${severeCount} high priority`}
          detail="Severe, urgent, stale, or low-confidence products are highlighted for controller/weather-desk review."
          attention={severeCount > 0}
        />
        <SafetyCard
          icon={<ShieldAlert size={16} />}
          title="Time-to-guidance"
          value={targetRate == null ? 'tracking' : `${Math.round(targetRate * 100)}% <5s`}
          detail={`Average guidance latency ${averageLatency == null ? 'unknown' : `${Math.round(averageLatency)}s`}; ${Math.round(missedTargets)} affected mission(s) missed the local target.`}
          attention={missedTargets > 0}
        />
      </section>
      <div className="workspace-grid-main">
        <section className="panel">
          <div className="panel-heading">
            <h3>Operational Guidance</h3>
            <span>hazard to action to rationale</span>
          </div>
          <div className="guidance-stack">
            {(guidance.length ? guidance : [emptyGuidance()]).map((item) => (
              <article
                key={item.id}
                className={item.priority === 'HIGH' ? 'guidance-card attention-card' : 'guidance-card'}
                onClick={() => setSelectedWeatherMessageId(item.id === 'empty' ? undefined : item.id)}
              >
                <div>
                  <strong>{item.hazard}</strong>
                  <p>{item.rationale}</p>
                </div>
                <StatusBadge value={item.action} />
                <small>{item.priority} priority · {item.coordination}</small>
              </article>
            ))}
          </div>
          <div className="panel-heading compact-heading">
            <h3>Affected Active Missions</h3>
            <span>{affectedMissions.data?.length ?? 0}</span>
          </div>
          <div className="change-feed">
            {(affectedMissions.data?.length ? affectedMissions.data : []).map((mission) => (
              <article key={mission.missionId} className="affected-mission-card">
                <button className="affected-mission-open" onClick={() => navigate(mission.route ?? `/missions/${mission.missionId}`)}>
                  <StatusBadge value={mission.action} />
                  <span>{mission.missionNumber}</span>
                  <small>
                    {Math.round(mission.confidence * 100)}% · {mission.sourceCount} source(s) · {freshnessLabel(mission.ageSeconds, mission.stale)}
                    {' · '}
                    {mission.guidanceTargetMet ? '<5s target met' : 'guidance delayed'}
                    {' · '}
                    {mission.rationale}
                  </small>
                </button>
                <div className="affected-mission-source-row" aria-label={`Sources for ${mission.missionNumber}`}>
                  {(mission.sourceRefs?.length ? mission.sourceRefs : ['No explicit source refs']).slice(0, 8).map((ref) => (
                    <button
                      key={ref}
                      type="button"
                      className={`route-source route-source-${sourceRefLabel(ref).family.toLowerCase().replace(/[^a-z0-9]+/g, '-')}`}
                      disabled={!sourceRefRoute(ref)}
                      onClick={() => {
                        const route = sourceRefRoute(ref);
                        if (route) navigate(route);
                      }}
                    >
                      <strong>{sourceRefLabel(ref).family}</strong>
                      {sourceRefLabel(ref).id || ref}
                    </button>
                  ))}
                  <button className="secondary" type="button" onClick={() => navigate(coordinationHref(mission))}>Coordinate</button>
                </div>
              </article>
            ))}
            {!affectedMissions.data?.length && <p className="muted">No active missions are currently affected by retained weather or PIREP traffic.</p>}
          </div>
          <div className="panel-heading compact-heading">
            <h3>What Changed Since Last Brief</h3>
            <span>latest weather / PIREP deltas</span>
          </div>
          <div className="change-feed">
            {(changes.length ? changes : [emptyGuidance()]).map((item) => (
              <button key={`change-${item.id}`} className="change-row" onClick={() => item.id !== 'empty' && navigate(`/messages/${item.id}`)}>
                <StatusBadge value={item.action} />
                <span>{item.hazard}</span>
                <small>{fmtZ(item.createdAt)} · {item.rationale}</small>
              </button>
            ))}
          </div>
          <QueryNotice query={messages} label="Weather messages" />
          <QueryNotice query={affectedMissions} label="Affected missions" />
          <QueryNotice query={metrics} label="Guidance metrics" />
          {selectedWeatherMessage && !selectedWeatherHasFeature && (
            <div className="notice">
              <strong>No map geometry</strong>
              <span>{selectedWeatherMessage.family} {selectedWeatherMessage.subject || selectedWeatherMessage.id} has no coordinate-bearing text; it remains available in guidance, trace, and retained raw text.</span>
            </div>
          )}
          <DataTable
            data={weatherMessages}
            columns={columns}
            onRowClick={(message) => setSelectedWeatherMessageId(message.id)}
            onRowDoubleClick={(message) => navigate(`/messages/${message.id}`)}
            isRowSelected={(message) => message.id === selectedWeatherMessageId}
          />
        </section>
        <OperationsMap
          features={featureCollection}
          selectedFeatureId={selectedMapFeatureId}
          onSelectedFeatureIdChange={(featureId) => {
            const match = /^weather-message-(.+)$/.exec(featureId ?? '');
            if (match) setSelectedWeatherMessageId(match[1]);
          }}
        />
      </div>
    </section>
  );
}

function freshnessLabel(ageSeconds?: number, stale?: boolean) {
  if (ageSeconds == null || ageSeconds < 0) return stale ? 'stale' : 'freshness unknown';
  if (ageSeconds < 60) return `${ageSeconds}s old`;
  const minutes = Math.round(ageSeconds / 60);
  return `${minutes}m old${stale ? ' · stale' : ''}`;
}

function emptyGuidance(): WeatherGuidanceItem {
  return {
    id: 'empty',
    hazard: 'No weather products loaded',
    action: 'MONITOR',
    priority: 'LOW',
    coordination: 'Ingest USNS/weather/PIREP traffic',
    sourceFamily: 'WEATHER',
    sourceLabel: 'No product',
    rationale: 'The engine needs current weather products and aircraft reports before it can issue route guidance.'
  };
}

function SafetyCard({ icon, title, value, detail, attention }: { icon: ReactNode; title: string; value: string; detail: string; attention?: boolean }) {
  return (
    <article className={attention ? 'safety-card attention-card' : 'safety-card'}>
      <header>{icon}<span>{title}</span></header>
      <strong>{value}</strong>
      <p>{detail}</p>
    </article>
  );
}

function coordinationHref(mission: { missionId: string; missionNumber: string; action: string; rationale?: string; sourceRefs?: string[] }) {
  const params = new URLSearchParams({
    missionId: mission.missionId,
    family: 'USNS',
    direction: 'OUTBOUND',
    subject: `WX COORD ${mission.missionNumber} ${mission.action}`,
    body: [
      'USNS WEATHER COORDINATION',
      `MISSION: ${mission.missionNumber}`,
      `ACTION: ${mission.action}`,
      `IMPACT: ${mission.rationale ?? 'Review weather impact and route release guidance.'}`,
      `SOURCES: ${(mission.sourceRefs ?? []).join(', ')}`,
      'REQUEST: WEATHER DESK / TRAFFIC MANAGER REVIEW.'
    ].join('\n')
  });
  return `/messages?${params.toString()}`;
}
