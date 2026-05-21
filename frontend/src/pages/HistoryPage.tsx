import { useQuery } from '@tanstack/react-query';
import { createColumnHelper } from '@tanstack/react-table';
import { useNavigate } from 'react-router-dom';
import { FileText } from 'lucide-react';
import { api } from '../api/client';
import { DataTable } from '../components/DataTable';
import { QueryNotice } from '../components/Notices';
import type { HistoryEventSummary } from '../types';
import { fmtZ } from '../lib/viewModels';
import { writeWorkbenchJson, type WorkbenchSelection } from '../lib/workbenchState';

export function HistoryPage() {
  const navigate = useNavigate();
  const history = useQuery({ queryKey: ['history'], queryFn: api.history });
  const column = createColumnHelper<HistoryEventSummary>();
  const columns = [
    column.accessor('createdAt', { header: 'Time', cell: (info) => fmtZ(info.getValue()) }),
    column.accessor('aggregateType', { header: 'Aggregate' }),
    column.accessor('eventType', { header: 'Event' }),
    column.accessor('actor', { header: 'Actor' }),
    column.accessor('note', { header: 'Note' })
  ];
  return (
    <section className="workspace operational-page">
      <div className="page-header"><div><h2><FileText size={18} /> History & Audit</h2><p>Operator-facing history events for missions, messages, feed artifacts, decisions, and reference changes.</p></div></div>
      <QueryNotice query={history} label="History" />
      <DataTable
        data={history.data ?? []}
        columns={columns}
        onRowClick={(event) => {
          writeWorkbenchJson('airspace.workbench.selection', selectionForHistory(event));
          window.dispatchEvent(new Event('airspace-workbench-selection'));
        }}
        onRowDoubleClick={(event) => {
          const route = routeForHistory(event);
          if (route) navigate(route);
        }}
      />
    </section>
  );
}

function selectionForHistory(event: HistoryEventSummary): WorkbenchSelection {
  const aggregate = event.aggregateType.toLowerCase();
  return {
    missionId: aggregate === 'mission' ? event.aggregateId : undefined,
    messageId: aggregate === 'message' ? event.aggregateId : undefined,
    feedArtifactId: aggregate === 'feed' ? event.aggregateId : undefined,
    decisionId: aggregate === 'decision' ? event.aggregateId : undefined,
    sourceFamily: aggregate === 'decision' ? 'DECISION' : aggregate === 'feed' ? 'USNS' : 'UNKNOWN',
    label: `${event.eventType} · ${event.aggregateId}`,
    lockState: event.actor ?? 'audit'
  };
}

function routeForHistory(event: HistoryEventSummary): string | undefined {
  if (event.aggregateType === 'mission') return `/missions/${event.aggregateId}`;
  if (event.aggregateType === 'message') return `/messages/${event.aggregateId}`;
  if (event.aggregateType === 'feed') return `/feed/${event.aggregateId}`;
  if (event.aggregateType === 'decision') return `/decisions/${event.aggregateId}`;
  return undefined;
}
