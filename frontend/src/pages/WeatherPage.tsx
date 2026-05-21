import { useEffect } from 'react';
import { useQuery } from '@tanstack/react-query';
import { createColumnHelper } from '@tanstack/react-table';
import { useNavigate } from 'react-router-dom';
import { CloudSun } from 'lucide-react';
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
    column.accessor('createdAt', { header: 'Received', cell: (info) => fmtZ(info.getValue()) }),
    column.accessor('rawText', { header: 'Raw' })
  ];
  return (
    <section className="workspace operational-page">
      <div className="page-header">
        <div>
          <h2><CloudSun size={18} /> Weather & PIREPs</h2>
          <p>Weather/PIREP products enter through USNS/feed/message paths and are fused as operational constraints.</p>
        </div>
        <StatusBadge value={`${weatherMessages.length} PRODUCTS`} />
      </div>
      <div className="workspace-grid-main">
        <section className="panel">
          <QueryNotice query={messages} label="Weather messages" />
          <DataTable data={weatherMessages} columns={columns} onRowDoubleClick={(message) => navigate(`/messages/${message.id}`)} />
        </section>
        <OperationsMap features={featureCollection} />
      </div>
    </section>
  );
}
