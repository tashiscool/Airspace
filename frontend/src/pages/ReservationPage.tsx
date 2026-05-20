import { useEffect, useMemo, useState } from 'react';
import { useMutation, useQuery, useQueryClient } from '@tanstack/react-query';
import { useNavigate, useParams } from 'react-router-dom';
import { api } from '../api/client';
import { StatusBadge } from '../components/StatusBadge';

export function ReservationPage() {
  const { missionId = '', reservationId = '' } = useParams();
  const queryClient = useQueryClient();
  const navigate = useNavigate();
  const mission = useQuery({ queryKey: ['mission', missionId], queryFn: () => api.mission(missionId), enabled: !!missionId });
  const supplements = useQuery({
    queryKey: ['reservation-supplements', reservationId],
    queryFn: () => api.reservationSupplements(reservationId),
    enabled: !!reservationId
  });
  const reservation = useMemo(
    () => mission.data?.reservations.find((item) => item.id === reservationId),
    [mission.data, reservationId]
  );
  const [rawText, setRawText] = useState<string>('');
  useEffect(() => {
    if (reservation?.rawText) {
      setRawText(reservation.rawText);
    }
  }, [reservation?.rawText]);
  const invalidate = () => queryClient.invalidateQueries({ queryKey: ['mission', missionId] });
  const save = useMutation({ mutationFn: () => api.updateReservation(reservationId, rawText, 'planner'), onSuccess: invalidate });
  const parse = useMutation({ mutationFn: () => api.parseReservation(reservationId, 'planner'), onSuccess: invalidate });
  const deconflict = useMutation({ mutationFn: () => api.deconflictReservation(reservationId, 'planner'), onSuccess: invalidate });
  const forceParse = useMutation({ mutationFn: () => api.forceParseReservation(reservationId, 'planner', 'Force parse from operator workspace'), onSuccess: invalidate });
  const forceDeconflict = useMutation({ mutationFn: () => api.forceDeconflictReservation(reservationId, 'planner', 'Force deconflict from operator workspace'), onSuccess: invalidate });
  const submit = useMutation({ mutationFn: () => api.submitReservation(reservationId, 'planner'), onSuccess: invalidate });
  const approve = useMutation({ mutationFn: () => api.approveReservation(reservationId, 'supervisor'), onSuccess: invalidate });
  const reject = useMutation({ mutationFn: () => api.rejectReservation(reservationId, 'supervisor', 'Rejected from workspace'), onSuccess: invalidate });
  const cancel = useMutation({ mutationFn: () => api.cancelReservation(reservationId, 'planner', 'Cancelled from workspace'), onSuccess: invalidate });
  const complete = useMutation({ mutationFn: () => api.completeReservation(reservationId, 'planner'), onSuccess: invalidate });
  const lock = useMutation({ mutationFn: () => api.lockReservation(reservationId, 'planner'), onSuccess: invalidate });
  const unlock = useMutation({ mutationFn: () => api.unlockReservation(reservationId, 'planner'), onSuccess: invalidate });
  const addNotam = useMutation({
    mutationFn: () => api.createReservationSupplement(reservationId, {
      kind: 'NOTAM',
      status: 'DRAFT',
      title: 'Domestic NOTAM',
      text: 'NOTAM draft generated from reservation workspace',
      actor: 'planner'
    }),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['reservation-supplements', reservationId] })
  });
  const addApreq = useMutation({
    mutationFn: () => api.createReservationSupplement(reservationId, {
      kind: 'APREQ',
      status: 'DRAFT',
      title: 'APREQ',
      text: 'APREQ coordination request',
      actor: 'planner'
    }),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['reservation-supplements', reservationId] })
  });
  const addApproval = useMutation({
    mutationFn: () => api.createReservationSupplement(reservationId, {
      kind: 'APPROVAL',
      status: 'OPEN',
      title: 'Supervisor Approval',
      text: 'Pending supervisor review',
      actor: 'supervisor'
    }),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['reservation-supplements', reservationId] })
  });
  const transitionSupplement = useMutation({
    mutationFn: (value: { id: string; status: string }) =>
      api.transitionReservationSupplement(reservationId, value.id, {
        status: value.status,
        actor: value.status === 'APPROVED' ? 'supervisor' : 'planner',
        note: `${value.status} from reservation workspace`
      }),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['reservation-supplements', reservationId] })
  });
  return (
    <section className="workspace">
      <div className="toolbar">
        <h2>Reservation {reservationId}</h2>
        {reservation && <StatusBadge value={reservation.state} />}
        <button onClick={() => lock.mutate()}>Lock</button>
        <button className="secondary" onClick={() => unlock.mutate()}>Unlock</button>
        <button onClick={() => save.mutate()}>Save</button>
        <button onClick={() => parse.mutate()}>Parse</button>
        <button onClick={() => deconflict.mutate()}>Deconflict</button>
        <button className="secondary" onClick={() => forceParse.mutate()}>Force Parse</button>
        <button className="secondary" onClick={() => forceDeconflict.mutate()}>Force Deconflict</button>
        <button onClick={() => submit.mutate()}>Submit</button>
        <button onClick={() => approve.mutate()}>Approve</button>
        <button className="secondary" onClick={() => reject.mutate()}>Reject</button>
        <button className="secondary" onClick={() => cancel.mutate()}>Cancel</button>
        <button className="secondary" onClick={() => complete.mutate()}>Complete</button>
        <button className="secondary" onClick={() => navigate(`/deconfliction/${reservationId}`)}>Deconfliction</button>
      </div>
      <div className="grid two">
        <div className="panel">
          <h3>Sections A-G</h3>
          <textarea value={rawText || 'A.\nB.\nC.\nD.\nE.\nF.\nG.'} onChange={(event) => setRawText(event.target.value)} />
        </div>
        <div className="panel">
          <h3>Supplements</h3>
          <dl className="detail-list">
            <dt>State</dt><dd>{reservation?.state ?? 'Loading'}</dd>
            <dt>Locked By</dt><dd>{reservation?.lockedBy ?? 'Unlocked'}</dd>
            <dt>Conflicts</dt><dd>{reservation?.conflictCount ?? 0}</dd>
          </dl>
          <div className="toolbar compact">
            <button onClick={() => addNotam.mutate()}>Add NOTAM</button>
            <button onClick={() => addApreq.mutate()}>Add APREQ</button>
            <button onClick={() => addApproval.mutate()}>Add Approval</button>
          </div>
          {(supplements.data?.length ? supplements.data : []).map((item) => (
            <div className="event supplement" key={item.id}>
              <strong>{item.kind}</strong> <StatusBadge value={item.status} />
              <div>{item.title}</div>
              <p>{item.text}</p>
              <div className="toolbar compact">
                <button className="secondary" onClick={() => transitionSupplement.mutate({ id: item.id, status: 'SUBMITTED' })}>Submit</button>
                <button className="secondary" onClick={() => transitionSupplement.mutate({ id: item.id, status: 'APPROVED' })}>Approve</button>
                <button className="secondary" onClick={() => transitionSupplement.mutate({ id: item.id, status: 'REJECTED' })}>Reject</button>
              </div>
            </div>
          ))}
          {!supplements.data?.length && <p className="muted">No NOTAM, APREQ, approval, or coordination supplements yet.</p>}
          <h4>Diagnostics</h4>
          {(reservation?.diagnostics?.length ? reservation.diagnostics : ['No diagnostics yet']).map((item) => (
            <div className="event" key={item}>{item}</div>
          ))}
        </div>
      </div>
    </section>
  );
}
