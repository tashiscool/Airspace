import { useMutation, useQuery, useQueryClient } from '@tanstack/react-query';
import { createColumnHelper } from '@tanstack/react-table';
import { useNavigate } from 'react-router-dom';
import { api } from '../api/client';
import { DataTable } from '../components/DataTable';
import { StatusBadge } from '../components/StatusBadge';
import type { MessageSummary, MissionSummary } from '../types';

const column = createColumnHelper<MissionSummary>();
const columns = [
  column.accessor('missionNumber', { header: 'Mission' }),
  column.accessor('title', { header: 'Title' }),
  column.accessor('status', { header: 'Status', cell: (info) => <StatusBadge value={info.getValue()} /> }),
  column.accessor('reservationCount', { header: 'Reservations' }),
  column.accessor('lockedBy', { header: 'Locked By' })
];
const messageColumn = createColumnHelper<MessageSummary>();
const messageColumns = [
  messageColumn.accessor('direction', { header: 'Dir' }),
  messageColumn.accessor('family', { header: 'Family' }),
  messageColumn.accessor('subject', { header: 'Subject' }),
  messageColumn.accessor('status', { header: 'Status', cell: (info) => <StatusBadge value={info.getValue()} /> })
];

export function ExplorerPage() {
  const navigate = useNavigate();
  const queryClient = useQueryClient();
  const missions = useQuery({ queryKey: ['missions'], queryFn: api.missions });
  const messages = useQuery({ queryKey: ['messages'], queryFn: api.messages });
  const metrics = useQuery({ queryKey: ['metrics'], queryFn: api.metrics });
  const create = useMutation({
    mutationFn: () => api.createMission({ title: 'New Mission', actor: 'planner' }),
    onSuccess: (mission) => {
      queryClient.invalidateQueries({ queryKey: ['missions'] });
      navigate(`/missions/${mission.id}`);
    }
  });
  return (
    <section className="workspace">
      <div className="toolbar">
        <h2>Mission Explorer</h2>
        <button onClick={() => create.mutate()}>New Mission</button>
      </div>
      <div className="metric-strip">
        {Object.entries(metrics.data ?? {}).filter(([name]) => [
          'product.missions',
          'product.messages',
          'product.feedArtifacts',
          'product.decisions',
          'product.historyEvents'
        ].includes(name)).map(([name, value]) => (
          <div className="metric-tile" key={name}>
            <span>{name.replace('product.', '')}</span>
            <strong>{Math.round(value)}</strong>
          </div>
        ))}
      </div>
      <div className="explorer-grid">
        <aside className="panel explorer-tree">
          <h3>Mission Tree</h3>
          {(missions.data ?? []).map((mission) => (
            <button className="tree-row" key={mission.id} onClick={() => navigate(`/missions/${mission.id}`)}>
              <span>{mission.missionNumber}</span>
              <StatusBadge value={mission.status} />
            </button>
          ))}
        </aside>
        <aside className="panel explorer-tree">
          <h3>Message Tree</h3>
          {(messages.data ?? []).map((message) => (
            <button className="tree-row" key={message.id} onClick={() => navigate(`/messages/${message.id}`)}>
              <span>{message.subject || message.family}</span>
              <StatusBadge value={message.status} />
            </button>
          ))}
        </aside>
        <section className="panel explorer-main">
          <h3>Missions / Reservations / NOTAMs</h3>
          <DataTable data={missions.data ?? []} columns={columns} onRowClick={(row) => navigate(`/missions/${row.id}`)} />
        </section>
        <section className="panel explorer-preview">
          <h3>Messages</h3>
          <DataTable data={messages.data ?? []} columns={messageColumns} onRowClick={(row) => navigate(`/messages/${row.id}`)} />
          <p className="preview-copy">
            Select a mission, reservation, message, NOTAM, APREQ, or approval record to open the operational workspace.
          </p>
        </section>
      </div>
    </section>
  );
}
