import { useMutation, useQuery, useQueryClient } from '@tanstack/react-query';
import { createColumnHelper } from '@tanstack/react-table';
import { useNavigate, useParams } from 'react-router-dom';
import { api } from '../api/client';
import { DataTable } from '../components/DataTable';
import { StatusBadge } from '../components/StatusBadge';
import type { ReservationSummary } from '../types';

const column = createColumnHelper<ReservationSummary>();
const reservationColumns = [
  column.accessor('id', { header: 'Reservation' }),
  column.accessor('state', { header: 'State', cell: (info) => <StatusBadge value={info.getValue()} /> }),
  column.accessor('conflictCount', { header: 'Conflicts' }),
  column.accessor('lockedBy', { header: 'Locked By' })
];

const sampleCarf = `A. TEST02 KZNY
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
  const mission = useQuery({ queryKey: ['mission', missionId], queryFn: () => api.mission(missionId), enabled: !!missionId });
  const createReservation = useMutation({
    mutationFn: () => api.createReservation(missionId, sampleCarf, 'planner'),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['mission', missionId] })
  });
  const lock = useMutation({ mutationFn: () => api.lockMission(missionId, 'planner'), onSuccess: () => queryClient.invalidateQueries({ queryKey: ['mission', missionId] }) });
  const unlock = useMutation({ mutationFn: () => api.unlockMission(missionId, 'planner'), onSuccess: () => queryClient.invalidateQueries({ queryKey: ['mission', missionId] }) });
  return (
    <section className="workspace">
      <div className="toolbar">
        <h2>{mission.data?.mission.missionNumber ?? 'Mission'}</h2>
        <button onClick={() => lock.mutate()}>Lock</button>
        <button className="secondary" onClick={() => unlock.mutate()}>Unlock</button>
        <button onClick={() => createReservation.mutate()}>Add Reservation</button>
      </div>
      <div className="grid two">
        <div className="panel">
          <h3>Reservations</h3>
          <DataTable
            data={mission.data?.reservations ?? []}
            columns={reservationColumns}
            onRowClick={(row) => navigate(`/missions/${missionId}/reservations/${row.id}`)}
          />
        </div>
        <div className="panel">
          <h3>History</h3>
          {(mission.data?.history ?? []).map((event) => (
            <div className="event" key={event.id}>{event.eventType} · {event.actor}</div>
          ))}
        </div>
      </div>
    </section>
  );
}
