import { useEffect, useMemo, useState } from 'react';
import { useMutation, useQuery, useQueryClient } from '@tanstack/react-query';
import { createColumnHelper } from '@tanstack/react-table';
import { Link, useNavigate, useParams } from 'react-router-dom';
import { Lock, Map, Plus, Unlock } from 'lucide-react';
import { api } from '../api/client';
import { DataTable } from '../components/DataTable';
import { ErrorNotice, MutationNotice, QueryNotice } from '../components/Notices';
import { OperationsMap } from '../components/OperationsMap';
import { StatusBadge } from '../components/StatusBadge';
import { compactId, fmtZ } from '../lib/viewModels';
import { writeWorkbenchJson, type WorkbenchSelection } from '../lib/workbenchState';
import type { MessageSummary, ReservationSummary } from '../types';

const reservationColumn = createColumnHelper<ReservationSummary>();
const reservationColumns = [
  reservationColumn.accessor('id', { header: 'Reservation', cell: (info) => compactId(info.getValue()) }),
  reservationColumn.accessor('state', { header: 'State', cell: (info) => <StatusBadge value={info.getValue()} /> }),
  reservationColumn.accessor('conflictCount', { header: 'Conflicts' }),
  reservationColumn.accessor('lockedBy', { header: 'Locked By', cell: (info) => info.getValue() ?? '—' }),
  reservationColumn.accessor((row) => row.diagnostics.length, { id: 'diagnostics', header: 'Diagnostics' })
];

const messageColumn = createColumnHelper<MessageSummary>();
const messageColumns = [
  messageColumn.accessor('family', { header: 'Family' }),
  messageColumn.accessor('direction', { header: 'Direction' }),
  messageColumn.accessor('status', { header: 'Status', cell: (info) => <StatusBadge value={info.getValue()} /> }),
  messageColumn.accessor('subject', { header: 'Subject' })
];

const defaultRouteReservationText = `A. TEST02 KZNY
B. 1F22/I
C. KZNY
D. FL240B260 3000N 15000W 0000 3000N 15100W 0100
E. TEST02
F. ETD TEST02 021200 MAR 2010 AVANA 021300
G. TAS: 300 KTAS`;

export function MissionPage() {
  const { missionId = '' } = useParams();
  const navigate = useNavigate();
  const queryClient = useQueryClient();
  const [selectedReservationId, setSelectedReservationId] = useState('');
  const [selectedMessageId, setSelectedMessageId] = useState('');
  const mission = useQuery({ queryKey: ['mission', missionId], queryFn: () => api.mission(missionId), enabled: !!missionId });
  const firstReservationId = mission.data?.reservations[0]?.id;
  const features = useQuery({
    queryKey: ['reservation-features', firstReservationId, 'mission-preview'],
    queryFn: () => api.reservationFeatures(firstReservationId!),
    enabled: !!firstReservationId
  });
  const createReservation = useMutation({
    mutationFn: () => api.createReservation(missionId, defaultRouteReservationText, 'planner'),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['mission', missionId] })
  });
  const lock = useMutation({ mutationFn: () => api.lockMission(missionId, 'planner'), onSuccess: () => queryClient.invalidateQueries({ queryKey: ['mission', missionId] }) });
  const unlock = useMutation({ mutationFn: () => api.unlockMission(missionId, 'planner'), onSuccess: () => queryClient.invalidateQueries({ queryKey: ['mission', missionId] }) });
  const conflictCount = useMemo(() => (mission.data?.reservations ?? []).reduce((sum, reservation) => sum + reservation.conflictCount, 0), [mission.data]);
  const selectedReservation = mission.data?.reservations.find((reservation) => reservation.id === selectedReservationId) ?? mission.data?.reservations[0];
  const selectedMessage = mission.data?.messages.find((message) => message.id === selectedMessageId) ?? mission.data?.messages[0];
  useEffect(() => {
    if (!mission.data?.mission) return;
    const selection: WorkbenchSelection = {
      missionId,
      reservationId: selectedReservation?.id,
      messageId: selectedMessage?.id,
      sourceFamily: 'CARF_ALTRV',
      label: mission.data.mission.missionNumber,
      lockState: mission.data.mission.lockedBy ? `Locked by ${mission.data.mission.lockedBy}` : 'Unlocked',
      conflictCount
    };
    writeWorkbenchJson('airspace.workbench.selection', selection);
    window.dispatchEvent(new Event('airspace-workbench-selection'));
  }, [mission.data?.mission, missionId, selectedReservation?.id, selectedMessage?.id, conflictCount]);
  return (
    <section className="workspace operational-page">
      <div className="page-header">
        <div>
          <h2>{mission.data?.mission.missionNumber ?? 'Mission Workspace'}</h2>
          <p>{mission.data?.mission.title ?? 'Mission detail, reservations, messages, history, and map preview.'}</p>
        </div>
        {mission.data?.mission.status && <StatusBadge value={mission.data.mission.status} />}
      </div>
      <div className="toolbar wrap">
        <button onClick={() => lock.mutate()}><Lock size={14} /> Lock</button>
        <button className="secondary" onClick={() => unlock.mutate()}><Unlock size={14} /> Unlock</button>
        <button onClick={() => createReservation.mutate()}><Plus size={14} /> Add Reservation</button>
        {firstReservationId && <button className="secondary" onClick={() => navigate(`/deconfliction/${firstReservationId}`)}><Map size={14} /> Deconfliction</button>}
      </div>
      <div className="notice-stack">
        <QueryNotice query={mission} label="Mission detail" />
        <ErrorNotice error={features.error} title="Mission map preview unavailable" />
        <MutationNotice mutation={lock} label="Lock mission" />
        <MutationNotice mutation={unlock} label="Unlock mission" />
        <MutationNotice mutation={createReservation} label="Create reservation" />
      </div>
      <section className="mission-summary-grid">
        <Metric label="Reservations" value={String(mission.data?.reservations.length ?? 0)} />
        <Metric label="Conflicts" value={String(conflictCount)} attention={conflictCount > 0} />
        <Metric label="Messages" value={String(mission.data?.messages.length ?? 0)} />
        <Metric label="Locked By" value={mission.data?.mission.lockedBy ?? 'Unlocked'} />
        <Metric label="Updated" value={fmtZ(mission.data?.mission.updatedAt)} />
      </section>
      <div className="workspace-grid-main">
        <section className="panel">
          <div className="panel-heading">
            <h3>Reservations</h3>
            <span>{mission.data?.reservations.length ?? 0}</span>
          </div>
          <DataTable
            data={mission.data?.reservations ?? []}
            columns={reservationColumns}
            onRowClick={(row) => setSelectedReservationId(row.id)}
            onRowDoubleClick={(row) => navigate(`/missions/${missionId}/reservations/${row.id}`)}
            isRowSelected={(row) => row.id === selectedReservation?.id}
          />
          {selectedReservation && (
            <div className="selection-console">
              <strong>{compactId(selectedReservation.id)}</strong>
              <StatusBadge value={selectedReservation.state} />
              <span>{selectedReservation.conflictCount} conflict(s)</span>
              <button className="secondary" onClick={() => navigate(`/missions/${missionId}/reservations/${selectedReservation.id}`)}>Open</button>
              <button className="secondary" onClick={() => navigate(`/deconfliction/${selectedReservation.id}`)}>Review</button>
            </div>
          )}
        </section>
        <section className="panel">
          <div className="panel-heading"><h3>Messages</h3><span>{mission.data?.messages.length ?? 0}</span></div>
          <DataTable
            data={mission.data?.messages ?? []}
            columns={messageColumns}
            onRowClick={(row) => setSelectedMessageId(row.id)}
            onRowDoubleClick={(row) => navigate(`/messages/${row.id}`)}
            isRowSelected={(row) => row.id === selectedMessage?.id}
          />
          {selectedMessage && (
            <div className="selection-console">
              <strong>{selectedMessage.subject || selectedMessage.family}</strong>
              <StatusBadge value={selectedMessage.status} />
              <span>{selectedMessage.direction}</span>
              <button className="secondary" onClick={() => navigate(`/messages/${selectedMessage.id}`)}>Open Message</button>
            </div>
          )}
        </section>
        <section className="panel">
          <div className="panel-heading"><h3>History</h3><Link to="/history">Open history</Link></div>
          <div className="event-list">
            {(mission.data?.history ?? []).map((event) => (
              <div className="event" key={event.id}>
                <strong>{event.eventType}</strong>
                <span>{event.actor ?? 'system'} · {fmtZ(event.createdAt)}</span>
                {event.note && <p>{event.note}</p>}
              </div>
            ))}
            {!mission.data?.history?.length && <p className="muted">No history has been recorded for this mission.</p>}
          </div>
        </section>
        <OperationsMap features={features.data} />
      </div>
    </section>
  );
}

function Metric({ label, value, attention }: { label: string; value: string; attention?: boolean }) {
  return (
    <div className={attention ? 'metric-tile attention' : 'metric-tile'}>
      <span>{label}</span>
      <strong>{value}</strong>
    </div>
  );
}
