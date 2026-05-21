import { useEffect, useState } from 'react';
import { useMutation, useQuery, useQueryClient } from '@tanstack/react-query';
import { createColumnHelper } from '@tanstack/react-table';
import { useNavigate, useParams } from 'react-router-dom';
import { Send } from 'lucide-react';
import { api } from '../api/client';
import { DataTable } from '../components/DataTable';
import { ErrorNotice, MutationNotice, QueryNotice } from '../components/Notices';
import { StatusBadge } from '../components/StatusBadge';
import { writeWorkbenchJson, type WorkbenchSelection } from '../lib/workbenchState';
import type { FeedArtifactSummary, FeedTransactionSummary } from '../types';

type FeedIngestResponse = {
  id?: string;
  results?: Array<{ envelope?: { id?: string } }>;
};

export function FeedPage() {
  const { artifactId = '' } = useParams();
  const navigate = useNavigate();
  const queryClient = useQueryClient();
  const [draft, setDraft] = useState({
    sourceId: 'operator',
    type: 'WEATHER',
    rawPayload: 'METAR KJFK 200000Z 18012KT 2SM RA BKN010'
  });
  const feed = useQuery({ queryKey: ['feed-artifacts'], queryFn: api.feedArtifacts });
  const selected = useQuery({ queryKey: ['feed-artifact', artifactId], queryFn: () => api.feedArtifact(artifactId), enabled: !!artifactId });
  const transactions = useQuery({ queryKey: ['feed-transactions', artifactId], queryFn: () => api.feedTransactions(artifactId), enabled: !!artifactId });
  useEffect(() => {
    if (!selected.data) return;
    const selection: WorkbenchSelection = {
      feedArtifactId: selected.data.id,
      sourceFamily: 'USNS',
      label: `${selected.data.type} feed ${selected.data.sourceId}`,
      lockState: selected.data.accepted ? 'Accepted' : 'Rejected'
    };
    writeWorkbenchJson('airspace.workbench.selection', selection);
    window.dispatchEvent(new Event('airspace-workbench-selection'));
  }, [selected.data]);
  const ingest = useMutation({
    mutationFn: async () => api.ingestFeed(draft) as Promise<FeedIngestResponse>,
    onSuccess: (batch: FeedIngestResponse) => {
      queryClient.invalidateQueries({ queryKey: ['feed-artifacts'] });
      const id = batch?.results?.[0]?.envelope?.id ?? batch?.id;
      if (id) navigate(`/feed/${id}`);
    }
  });
  const column = createColumnHelper<FeedArtifactSummary>();
  const txColumn = createColumnHelper<FeedTransactionSummary>();
  const columns = [
    column.accessor('sourceId', { header: 'Source' }),
    column.accessor('type', { header: 'Type' }),
    column.accessor('accepted', { header: 'Status', cell: (info) => <StatusBadge value={info.getValue() ? 'ACCEPTED' : 'REJECTED'} /> }),
    column.accessor('rawPayloadHash', { header: 'Payload Hash' }),
    column.accessor((row) => row.diagnostics.length, { id: 'diagnostics', header: 'Diagnostics' })
  ];
  const txColumns = [
    txColumn.accessor('type', { header: 'Type' }),
    txColumn.accessor('status', { header: 'Status', cell: (info) => <StatusBadge value={info.getValue()} /> }),
    txColumn.accessor('supported', { header: 'Supported', cell: (info) => String(info.getValue()) }),
    txColumn.accessor('normalizedText', { header: 'Normalized' }),
    txColumn.accessor((row) => row.errors.length + row.warnings.length, { id: 'diagnostics', header: 'Diagnostics' })
  ];
  return (
    <section className="feed-workspace">
      <aside className="feed-list">
        <header><Send size={14} /> USNS / Feed Artifacts <span>{feed.data?.length ?? 0}</span></header>
        <DataTable
          data={feed.data ?? []}
          columns={columns}
          onRowClick={(artifact) => navigate(`/feed/${artifact.id}`)}
          onRowDoubleClick={(artifact) => navigate(`/feed/${artifact.id}`)}
          isRowSelected={(artifact) => artifact.id === artifactId}
        />
      </aside>
      <main className="feed-detail">
        <section className="panel">
          <div className="panel-heading"><h3>Manual Feed Ingest</h3><button onClick={() => ingest.mutate()}>Ingest Payload</button></div>
          <MutationNotice mutation={ingest} label="Feed ingest" />
          <div className="form-grid">
            <label>Source<input value={draft.sourceId} onChange={(event) => setDraft({ ...draft, sourceId: event.target.value })} /></label>
            <label>Type<input value={draft.type} onChange={(event) => setDraft({ ...draft, type: event.target.value })} /></label>
          </div>
          <textarea className="compact-textarea" value={draft.rawPayload} onChange={(event) => setDraft({ ...draft, rawPayload: event.target.value })} />
        </section>
        <section className="panel">
          <h3>Artifact Detail</h3>
          <div className="notice-stack">
            <QueryNotice query={feed} label="Feed artifacts" />
            <ErrorNotice error={selected.error} title="Feed artifact unavailable" />
            <ErrorNotice error={transactions.error} title="Feed transactions unavailable" />
          </div>
          {selected.data ? (
            <>
              <div className="detail-grid">
                <Detail label="Status"><StatusBadge value={selected.data.accepted ? 'ACCEPTED' : 'REJECTED'} /></Detail>
                <Detail label="Source">{selected.data.sourceId}</Detail>
                <Detail label="Type">{selected.data.type}</Detail>
                <Detail label="Hash"><code>{selected.data.rawPayloadHash}</code></Detail>
              </div>
              <h4>Transactions</h4>
              <DataTable data={transactions.data ?? []} columns={txColumns} />
              <h4>Diagnostics</h4>
              {(selected.data.diagnostics.length ? selected.data.diagnostics : ['No diagnostics']).map((item) => <div className="event" key={item}>{item}</div>)}
              <pre className="raw-panel">{selected.data.rawPayload || 'No raw payload retained.'}</pre>
            </>
          ) : <p className="muted">Open a feed artifact to inspect retained raw traffic and parsed transactions.</p>}
        </section>
      </main>
    </section>
  );
}

function Detail({ label, children }: { label: string; children: React.ReactNode }) {
  return <div className="detail-row"><span>{label}</span><div>{children}</div></div>;
}
