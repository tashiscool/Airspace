import { useMutation, useQuery, useQueryClient } from '@tanstack/react-query';
import { createColumnHelper } from '@tanstack/react-table';
import { useNavigate, useParams } from 'react-router-dom';
import { useState } from 'react';
import { api } from '../api/client';
import { DataTable } from '../components/DataTable';
import { StatusBadge } from '../components/StatusBadge';
import type { MessageSummary } from '../types';

const column = createColumnHelper<MessageSummary>();
const columns = [
  column.accessor('family', { header: 'Family' }),
  column.accessor('direction', { header: 'Direction' }),
  column.accessor('status', { header: 'Status' }),
  column.accessor('subject', { header: 'Subject' })
];

export function MessagesPage() {
  const { messageId = 'inbox' } = useParams();
  const navigate = useNavigate();
  const queryClient = useQueryClient();
  const [draft, setDraft] = useState({
    family: 'USNS',
    direction: 'OUTBOUND',
    subject: 'Coordination',
    rawText: 'SVC RQ'
  });
  const messages = useQuery({ queryKey: ['messages'], queryFn: api.messages });
  const selected = useQuery({
    queryKey: ['message', messageId],
    queryFn: () => api.message(messageId),
    enabled: !!messageId && messageId !== 'inbox'
  });
  const send = useMutation({
    mutationFn: () => api.sendMessage({ ...draft, actor: 'planner' }),
    onSuccess: (message) => {
      queryClient.invalidateQueries({ queryKey: ['messages'] });
      navigate(`/messages/${message.id}`);
    }
  });
  const reply = useMutation({
    mutationFn: () => api.replyMessage(selectedMessage!.id, { rawText: draft.rawText, actor: 'planner' }),
    onSuccess: (message) => {
      queryClient.invalidateQueries({ queryKey: ['messages'] });
      navigate(`/messages/${message.id}`);
    }
  });
  const forward = useMutation({
    mutationFn: () => api.forwardMessage(selectedMessage!.id, { rawText: draft.rawText, actor: 'planner' }),
    onSuccess: (message) => {
      queryClient.invalidateQueries({ queryKey: ['messages'] });
      navigate(`/messages/${message.id}`);
    }
  });
  const selectedMessage = selected.data ?? messages.data?.find((message) => message.id === messageId);
  return (
    <section className="workspace">
      <div className="toolbar"><h2>Messaging</h2><button onClick={() => send.mutate()}>Send Message</button></div>
      <div className="grid two">
        <div className="panel">
          <h3>Message Queue</h3>
          <DataTable data={messages.data ?? []} columns={columns} onRowClick={(message) => navigate(`/messages/${message.id}`)} />
          <h3>Compose</h3>
          <div className="form-grid">
            <label>Family<input value={draft.family} onChange={(event) => setDraft({ ...draft, family: event.target.value })} /></label>
            <label>Direction<input value={draft.direction} onChange={(event) => setDraft({ ...draft, direction: event.target.value })} /></label>
            <label>Subject<input value={draft.subject} onChange={(event) => setDraft({ ...draft, subject: event.target.value })} /></label>
          </div>
          <textarea
            className="compact-textarea"
            value={draft.rawText}
            onChange={(event) => setDraft({ ...draft, rawText: event.target.value })}
          />
        </div>
        <div className="panel detail-panel">
          <h3>Message Detail</h3>
          {selectedMessage ? (
            <>
              <div className="toolbar compact">
                <button onClick={() => reply.mutate()}>Reply</button>
                <button className="secondary" onClick={() => forward.mutate()}>Forward</button>
              </div>
              <div className="detail-row"><span>Status</span><StatusBadge value={selectedMessage.status} /></div>
              <div className="detail-row"><span>Family</span><strong>{selectedMessage.family}</strong></div>
              <div className="detail-row"><span>Direction</span><strong>{selectedMessage.direction}</strong></div>
              <div className="detail-row"><span>Subject</span><strong>{selectedMessage.subject || 'No subject'}</strong></div>
              <div className="detail-row"><span>Mission</span><code>{selectedMessage.missionId || 'unlinked'}</code></div>
              <div className="detail-row"><span>Reservation</span><code>{selectedMessage.reservationId || 'unlinked'}</code></div>
              <pre className="raw-panel">{selectedMessage.rawText || 'No message body retained.'}</pre>
            </>
          ) : (
            <p className="muted">Open a message to inspect its retained source text and routing metadata.</p>
          )}
        </div>
      </div>
    </section>
  );
}
