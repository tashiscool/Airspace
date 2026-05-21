import { useEffect } from 'react';
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
import { fmtZ, referencePointToFeature, weatherRowsFromMessages } from '../lib/viewModels';
import { writeWorkbenchJson, type WorkbenchSelection } from '../lib/workbenchState';

export function WeatherPage() {
  const navigate = useNavigate();
  const messages = useQuery({ queryKey: ['messages'], queryFn: api.messages });
  const reference = useQuery({ queryKey: ['reference-points'], queryFn: () => api.referencePoints() });
  const weatherMessages = weatherRowsFromMessages(messages.data ?? []);
  const guidance = weatherMessages.map(guidanceFromMessage);
  const severeCount = guidance.filter((item) => item.priority === 'HIGH').length;
  const pirepCount = weatherMessages.filter((message) => message.family.toUpperCase().includes('PIREP')).length;
  const routeBlockageCount = guidance.filter((item) => item.action === 'REROUTE' || item.action === 'AVOID').length;
  useEffect(() => {
    const selection: WorkbenchSelection = {
      sourceFamily: 'WEATHER',
      label: 'Weather & PIREP constraints',
      lockState: `${weatherMessages.length} products`
    };
    writeWorkbenchJson('airspace.workbench.selection', selection);
    window.dispatchEvent(new Event('airspace-workbench-selection'));
  }, [weatherMessages.length]);
  const featureCollection = {
    type: 'FeatureCollection' as const,
    features: (reference.data ?? []).slice(0, 100).map(referencePointToFeature)
  };
  const column = createColumnHelper<MessageSummary>();
  const columns = [
    column.accessor('family', { header: 'Product' }),
    column.accessor('status', { header: 'Status', cell: (info) => <StatusBadge value={info.getValue()} /> }),
    column.accessor('subject', { header: 'Subject' }),
    column.display({ header: 'Guidance', cell: (info) => <StatusBadge value={guidanceFromMessage(info.row.original).action} /> }),
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
      </section>
      <div className="workspace-grid-main">
        <section className="panel">
          <div className="panel-heading">
            <h3>Operational Guidance</h3>
            <span>hazard to action to rationale</span>
          </div>
          <div className="guidance-stack">
            {(guidance.length ? guidance : [emptyGuidance()]).map((item) => (
              <article key={item.id} className={item.priority === 'HIGH' ? 'guidance-card attention-card' : 'guidance-card'}>
                <div>
                  <strong>{item.hazard}</strong>
                  <p>{item.rationale}</p>
                </div>
                <StatusBadge value={item.action} />
                <small>{item.priority} priority · {item.coordination}</small>
              </article>
            ))}
          </div>
          <QueryNotice query={messages} label="Weather messages" />
          <DataTable data={weatherMessages} columns={columns} onRowDoubleClick={(message) => navigate(`/messages/${message.id}`)} />
        </section>
        <OperationsMap features={featureCollection} />
      </div>
    </section>
  );
}

type GuidanceItem = {
  id: string;
  hazard: string;
  action: string;
  priority: 'HIGH' | 'MEDIUM' | 'LOW';
  coordination: string;
  rationale: string;
};

function guidanceFromMessage(message: MessageSummary): GuidanceItem {
  const family = message.family.toUpperCase();
  const raw = `${message.subject ?? ''} ${message.rawText ?? ''}`.toUpperCase();
  if (family.includes('SIGMET') || family.includes('CWAP') || family.includes('CWAF') || raw.includes('EMBD TS') || raw.includes('CONV')) {
    return {
      id: message.id,
      hazard: family.includes('SIGMET') ? 'Convective SIGMET' : 'Convective weather',
      action: raw.includes('TOP FL') || raw.includes('INTSF') ? 'REROUTE' : 'AVOID',
      priority: 'HIGH',
      coordination: 'Weather desk + traffic manager review',
      rationale: 'Embedded or intensifying convection can block route segments and reduce sector capacity.'
    };
  }
  if (family.includes('PIREP') || raw.includes('/TB') || raw.includes('/IC')) {
    const severe = raw.includes('SEV') || raw.includes('URGENT');
    return {
      id: message.id,
      hazard: raw.includes('/IC') ? 'Aircraft icing report' : 'Aircraft turbulence report',
      action: severe ? 'ALTITUDE CHANGE' : 'CAUTION',
      priority: severe ? 'HIGH' : 'MEDIUM',
      coordination: 'Solicit/verify PIREP and disseminate',
      rationale: 'Aircraft reports close the gap between forecasts and observed hazards along the active route.'
    };
  }
  if (family.includes('METAR') || family.includes('TAF') || raw.includes('BKN00') || raw.includes('OVC00') || raw.includes(' 1/2SM')) {
    return {
      id: message.id,
      hazard: family.includes('TAF') ? 'Forecast ceiling/visibility' : 'Observed ceiling/visibility',
      action: raw.includes('1/2SM') || raw.includes('BKN004') ? 'DELAY' : 'MONITOR',
      priority: raw.includes('1/2SM') || raw.includes('BKN004') ? 'MEDIUM' : 'LOW',
      coordination: 'Terminal weather and route-release check',
      rationale: 'Low ceiling or visibility affects departure/arrival decisions more than enroute lateral avoidance.'
    };
  }
  if (family.includes('AIRMET') || raw.includes('TURB') || raw.includes('ICE')) {
    return {
      id: message.id,
      hazard: 'AIRMET turbulence/icing',
      action: raw.includes('ICE') ? 'ALTITUDE CHANGE' : 'CAUTION',
      priority: 'MEDIUM',
      coordination: 'Monitor forecast confidence and pilot reports',
      rationale: 'AIRMET hazards usually require altitude or route-risk management rather than an automatic block.'
    };
  }
  return {
    id: message.id,
    hazard: message.family,
    action: 'MONITOR',
    priority: 'LOW',
    coordination: 'Retain and classify',
    rationale: 'Product is preserved for fusion, review, and later replay even when no immediate block is detected.'
  };
}

function emptyGuidance(): GuidanceItem {
  return {
    id: 'empty',
    hazard: 'No weather products loaded',
    action: 'MONITOR',
    priority: 'LOW',
    coordination: 'Ingest USNS/weather/PIREP traffic',
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
