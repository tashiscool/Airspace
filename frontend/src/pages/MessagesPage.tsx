import { useEffect, useMemo, useState } from 'react';
import type { ReactNode } from 'react';
import { useMutation, useQuery, useQueryClient } from '@tanstack/react-query';
import { createColumnHelper } from '@tanstack/react-table';
import { Link, useNavigate, useParams } from 'react-router-dom';
import { Archive, Forward, Mail, Plus, Reply, Search, Send, X } from 'lucide-react';
import { api } from '../api/client';
import { DataTable } from '../components/DataTable';
import { MutationNotice, QueryNotice } from '../components/Notices';
import { StatusBadge } from '../components/StatusBadge';
import { DEFAULT_RECIPIENT_PRESETS, messageBodyWithMetadata, recipientPresetSuggestions, relatedMessages } from '../lib/messagingView';
import { MESSAGE_FAMILIES, fmtZ, isNotamFamily } from '../lib/viewModels';
import { sourceFamilyForRow, writeWorkbenchJson, type WorkbenchSelection } from '../lib/workbenchState';
import type { MessageSummary } from '../types';

type MailboxMode = 'INBOX' | 'OUTBOX' | 'ARCHIVE' | 'ALL';

const column = createColumnHelper<MessageSummary>();
const columns = [
  column.accessor('family', { header: 'Family' }),
  column.accessor('direction', { header: 'Direction' }),
  column.accessor('status', { header: 'Status', cell: (info) => <StatusBadge value={info.getValue()} /> }),
  column.accessor('subject', { header: 'Subject' }),
  column.accessor('createdAt', { header: 'Time', cell: (info) => fmtZ(info.getValue()) })
];

export function MessagesPage() {
  const { messageId = '' } = useParams();
  const navigate = useNavigate();
  const queryClient = useQueryClient();
  const [mailbox, setMailbox] = useState<MailboxMode>('INBOX');
  const [familyChips, setFamilyChips] = useState<Set<string>>(new Set());
  const [archived, setArchived] = useState<Set<string>>(new Set());
  const [search, setSearch] = useState('');
  const [draft, setDraft] = useState({
    family: 'USNS',
    direction: 'OUTBOUND',
    subject: 'Coordination',
    rawText: 'SVC RQ'
  });
  const [recipients, setRecipients] = useState('CARF, USNOF');
  const [attachments, setAttachments] = useState('');
  const [recipientFilter, setRecipientFilter] = useState('');
  const messages = useQuery({ queryKey: ['messages'], queryFn: api.messages });
  const selectedMessage = messages.data?.find((message) => message.id === messageId) ?? messages.data?.[0];
  useEffect(() => {
    if (!selectedMessage) return;
    const selection: WorkbenchSelection = {
      missionId: selectedMessage.missionId,
      reservationId: selectedMessage.reservationId,
      messageId: selectedMessage.id,
      sourceFamily: sourceFamilyForRow({
        key: `message:${selectedMessage.id}`,
        family: isNotamFamily(selectedMessage.family) ? 'NOTAM' : 'MESSAGE',
        id: selectedMessage.id,
        title: selectedMessage.subject || selectedMessage.family,
        subtitle: `${selectedMessage.direction} · ${selectedMessage.family}`,
        status: selectedMessage.status,
        preview: selectedMessage.rawText ?? '',
        missionId: selectedMessage.missionId,
        reservationId: selectedMessage.reservationId
      }),
      label: selectedMessage.subject || `${selectedMessage.family} ${selectedMessage.id}`,
      lockState: selectedMessage.status
    };
    writeWorkbenchJson('airspace.workbench.selection', selection);
    window.dispatchEvent(new Event('airspace-workbench-selection'));
  }, [selectedMessage]);
  const filtered = useMemo(() => {
    const q = search.trim().toLowerCase();
    return (messages.data ?? []).filter((message) => {
      const inArchive = archived.has(message.id);
      if (mailbox === 'ARCHIVE' && !inArchive) return false;
      if (mailbox !== 'ARCHIVE' && inArchive) return false;
      if (mailbox === 'INBOX' && message.direction.toUpperCase() !== 'INBOUND') return false;
      if (mailbox === 'OUTBOX' && message.direction.toUpperCase() !== 'OUTBOUND') return false;
      if (familyChips.size && !familyChips.has(message.family)) return false;
      if (q && !`${message.family} ${message.direction} ${message.subject} ${message.rawText}`.toLowerCase().includes(q)) return false;
      return true;
    });
  }, [messages.data, archived, mailbox, familyChips, search]);
  const send = useMutation({
    mutationFn: () => api.sendMessage({ ...draft, rawText: messageBodyWithMetadata(draft.rawText, recipients, attachments), actor: 'planner' }),
    onSuccess: (message) => {
      queryClient.invalidateQueries({ queryKey: ['messages'] });
      navigate(`/messages/${message.id}`);
    }
  });
  const reply = useMutation({
    mutationFn: () => api.replyMessage(selectedMessage!.id, { ...draft, direction: 'OUTBOUND', rawText: messageBodyWithMetadata(draft.rawText, recipients, attachments), actor: 'planner' }),
    onSuccess: (message) => {
      queryClient.invalidateQueries({ queryKey: ['messages'] });
      navigate(`/messages/${message.id}`);
    }
  });
  const forward = useMutation({
    mutationFn: () => api.forwardMessage(selectedMessage!.id, { ...draft, direction: 'OUTBOUND', rawText: messageBodyWithMetadata(draft.rawText, recipients, attachments), actor: 'planner' }),
    onSuccess: (message) => {
      queryClient.invalidateQueries({ queryKey: ['messages'] });
      navigate(`/messages/${message.id}`);
    }
  });

  function toggleFamily(family: string) {
    setFamilyChips((current) => {
      const next = new Set(current);
      next.has(family) ? next.delete(family) : next.add(family);
      return next;
    });
  }

  return (
    <section className="message-workspace">
      <div className="board-toolbar">
        <span className="board-title"><Mail size={14} /> Messaging · USNS</span>
        <span className="board-count">{filtered.length}/{messages.data?.length ?? 0}</span>
        <div className="segmented">
          {(['INBOX', 'OUTBOX', 'ARCHIVE', 'ALL'] as MailboxMode[]).map((mode) => (
            <button key={mode} className={mailbox === mode ? 'active' : ''} onClick={() => setMailbox(mode)}>{mode}</button>
          ))}
        </div>
        <div className="search-box"><Search size={14} /><input value={search} onChange={(event) => setSearch(event.target.value)} placeholder="Search subject, body, family..." /></div>
        <button onClick={() => send.mutate()}><Send size={14} /> Send</button>
      </div>
      <div className="filter-strip">
        <span>Family</span>
        {MESSAGE_FAMILIES.map((family) => {
          const count = (messages.data ?? []).filter((message) => message.family === family).length;
          return (
            <button key={family} className={familyChips.has(family) ? 'chip active' : 'chip'} onClick={() => toggleFamily(family)}>
              {family}<small>{count}</small>
            </button>
          );
        })}
        {familyChips.size > 0 && <button className="chip clear" onClick={() => setFamilyChips(new Set())}><X size={12} /> Clear</button>}
      </div>
      <div className="notice-stack">
        <QueryNotice query={messages} label="Messages" />
        <MutationNotice mutation={send} label="Send message" />
        <MutationNotice mutation={reply} label="Reply" />
        <MutationNotice mutation={forward} label="Forward" />
      </div>
      <div className="message-grid">
        <section className="panel">
          <DataTable
            data={filtered}
            columns={columns}
            onRowClick={(message) => navigate(`/messages/${message.id}`)}
            onRowDoubleClick={(message) => navigate(`/messages/${message.id}`)}
            isRowSelected={(message) => message.id === selectedMessage?.id}
          />
          <div className="compose-panel">
            <h3><Plus size={14} /> Compose</h3>
            <div className="form-grid">
              <label>Family<input value={draft.family} onChange={(event) => setDraft({ ...draft, family: event.target.value })} /></label>
              <label>Direction<input value={draft.direction} onChange={(event) => setDraft({ ...draft, direction: event.target.value })} /></label>
              <label>Subject<input value={draft.subject} onChange={(event) => setDraft({ ...draft, subject: event.target.value })} /></label>
              <label>Recipients<input value={recipients} onChange={(event) => setRecipients(event.target.value)} placeholder="USNOF, ZNY, CARF..." /></label>
              <label>Attachments<input value={attachments} onChange={(event) => setAttachments(event.target.value)} placeholder="filename.pdf, image.png..." /></label>
            </div>
            <div className="recipient-presets">
              <span>Recipients</span>
              <input value={recipientFilter} onChange={(event) => setRecipientFilter(event.target.value)} placeholder="Filter directory..." />
              {recipientPresetSuggestions(recipientFilter, DEFAULT_RECIPIENT_PRESETS).map((preset) => (
                <button
                  key={preset}
                  className="chip"
                  onClick={() => setRecipients((current) => current ? `${current}, ${preset}` : preset)}
                  type="button"
                >
                  {preset}
                </button>
              ))}
            </div>
            <textarea className="compact-textarea" value={draft.rawText} onChange={(event) => setDraft({ ...draft, rawText: event.target.value })} />
          </div>
        </section>
        <aside className="panel detail-panel">
          <h3>Message Detail</h3>
          {selectedMessage ? (
            <>
              <div className="toolbar compact">
                <button disabled={!selectedMessage} onClick={() => reply.mutate()}><Reply size={14} /> Reply</button>
                <button className="secondary" disabled={!selectedMessage} onClick={() => forward.mutate()}><Forward size={14} /> Forward</button>
                <button
                  className="secondary"
                  onClick={() => setArchived((current) => {
                    const next = new Set(current);
                    next.has(selectedMessage.id) ? next.delete(selectedMessage.id) : next.add(selectedMessage.id);
                    return next;
                  })}
                >
                  <Archive size={14} /> {archived.has(selectedMessage.id) ? 'Unarchive' : 'Archive'}
                </button>
              </div>
              <Detail label="Status"><StatusBadge value={selectedMessage.status} /></Detail>
              <Detail label="Family"><strong>{selectedMessage.family}</strong>{isNotamFamily(selectedMessage.family) && <span className="constraint-note">NOTAM constraint source</span>}</Detail>
              <Detail label="Direction"><strong>{selectedMessage.direction}</strong></Detail>
              <Detail label="Subject"><strong>{selectedMessage.subject || 'No subject'}</strong></Detail>
              <Detail label="Mission">{selectedMessage.missionId ? <Link to={`/missions/${selectedMessage.missionId}`}>{selectedMessage.missionId}</Link> : <code>unlinked</code>}</Detail>
              <Detail label="Reservation">{selectedMessage.missionId && selectedMessage.reservationId ? <Link to={`/missions/${selectedMessage.missionId}/reservations/${selectedMessage.reservationId}`}>{selectedMessage.reservationId}</Link> : <code>unlinked</code>}</Detail>
              <Detail label="Created"><span>{fmtZ(selectedMessage.createdAt)}</span></Detail>
              <pre className="raw-panel">{selectedMessage.rawText || 'No message body retained.'}</pre>
              <h4>Thread / Related Traffic</h4>
              <div className="event-list">
                {relatedMessages(messages.data ?? [], selectedMessage).map((message) => (
                  <button key={message.id} className="queue-item" onClick={() => navigate(`/messages/${message.id}`)}>
                    <span>{message.subject || message.family}</span>
                    <StatusBadge value={message.status} />
                    <small>{message.direction} · {fmtZ(message.createdAt)}</small>
                  </button>
                ))}
                {!relatedMessages(messages.data ?? [], selectedMessage).length && <p className="muted">No related thread traffic found by mission, reservation, or family.</p>}
              </div>
            </>
          ) : (
            <p className="muted">Open a message to inspect retained source text, routing metadata, and related mission/reservation links.</p>
          )}
        </aside>
      </div>
    </section>
  );
}

function Detail({ label, children }: { label: string; children: ReactNode }) {
  return <div className="detail-row"><span>{label}</span><div>{children}</div></div>;
}
