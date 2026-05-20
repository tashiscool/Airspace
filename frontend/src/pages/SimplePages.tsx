import { useMutation, useQuery, useQueryClient } from '@tanstack/react-query';
import { createColumnHelper } from '@tanstack/react-table';
import { useNavigate, useParams } from 'react-router-dom';
import { useState } from 'react';
import { api } from '../api/client';
import { DataTable } from '../components/DataTable';
import { OperationsMap } from '../components/OperationsMap';
import { StatusBadge } from '../components/StatusBadge';
import type { AirspaceFeature, FeedArtifactSummary, FeedTransactionSummary, HistoryEventSummary, ReferencePointSummary, SearchResultSummary } from '../types';

export function DeconflictionPage() {
  const { reservationId = '' } = useParams();
  const queryClient = useQueryClient();
  const features = useQuery({
    queryKey: ['reservation-features', reservationId],
    queryFn: () => api.reservationFeatures(reservationId),
    enabled: !!reservationId
  });
  const forceDeconflict = useMutation({
    mutationFn: () => api.forceDeconflictReservation(reservationId, 'planner', 'Force deconflict from deconfliction workspace'),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['reservation-features', reservationId] })
  });
  const conflictColumn = createColumnHelper<AirspaceFeature>();
  const conflicts = (features.data?.features ?? []).filter((feature) => feature.properties?.featureKind === 'conflict');
  const columns = [
    conflictColumn.accessor((row) => String(row.properties?.firstReservationId ?? ''), { id: 'first', header: 'First' }),
    conflictColumn.accessor((row) => String(row.properties?.secondReservationId ?? ''), { id: 'second', header: 'Second' }),
    conflictColumn.accessor((row) => Number(row.properties?.minimumLateralDistanceNauticalMiles ?? 0).toFixed(2), { id: 'lateral', header: 'Lat NM' }),
    conflictColumn.accessor((row) => Number(row.properties?.verticalSeparationFeet ?? 0).toFixed(0), { id: 'vertical', header: 'Vert Ft' }),
    conflictColumn.accessor((row) => String(row.properties?.belowMinimumDuration ?? false), { id: 'short', header: 'Too Short' })
  ];
  return (
    <section className="workspace">
      <div className="toolbar">
        <h2>Deconfliction</h2>
        <StatusBadge value={conflicts.length ? 'CONFLICTS' : 'CLEAR'} />
        <button className="secondary" onClick={() => forceDeconflict.mutate()}>Force Deconflict</button>
      </div>
      <div className="grid two">
        <div className="panel">
          <h3>Conflict Review</h3>
          <DataTable data={conflicts} columns={columns} />
          <h3>Explanation</h3>
          {(conflicts.length ? conflicts : [{ id: 'clear', properties: { explanation: 'No conflict features returned for this reservation.' } }]).map((feature) => (
            <div className="event" key={feature.id || String(feature.properties?.id)}>{String(feature.properties?.explanation ?? 'No explanation')}</div>
          ))}
        </div>
        <OperationsMap features={features.data} />
      </div>
    </section>
  );
}

export function SearchPage() {
  const navigate = useNavigate();
  const [query, setQuery] = useState('');
  const search = useQuery({
    queryKey: ['search', query],
    queryFn: () => api.search(query),
    enabled: query.trim().length > 1
  });
  const column = createColumnHelper<SearchResultSummary>();
  const columns = [
    column.accessor('type', { header: 'Type' }),
    column.accessor('title', { header: 'Title' }),
    column.accessor('status', { header: 'Status', cell: (info) => info.getValue() ? <StatusBadge value={info.getValue() || ''} /> : null }),
    column.accessor('snippet', { header: 'Snippet' }),
    column.accessor('updatedAt', { header: 'Updated' })
  ];
  return (
    <section className="workspace">
      <div className="toolbar">
        <h2>Search</h2>
        <input
          className="search-input"
          value={query}
          onChange={(event) => setQuery(event.target.value)}
          placeholder="Mission, message, feed artifact, history"
        />
      </div>
      <DataTable
        data={search.data ?? []}
        columns={columns}
        onRowClick={(row) => row.route && navigate(row.route)}
      />
      {query.trim().length <= 1 && <p className="muted">Enter at least two characters to search operational records.</p>}
    </section>
  );
}

export function HistoryPage() {
  const navigate = useNavigate();
  const history = useQuery({ queryKey: ['history'], queryFn: api.history });
  const column = createColumnHelper<HistoryEventSummary>();
  const columns = [
    column.accessor('createdAt', { header: 'Time' }),
    column.accessor('aggregateType', { header: 'Aggregate' }),
    column.accessor('eventType', { header: 'Event' }),
    column.accessor('actor', { header: 'Actor' }),
    column.accessor('note', { header: 'Note' })
  ];
  return (
    <section className="workspace">
      <h2>History</h2>
      <DataTable
        data={history.data ?? []}
        columns={columns}
        onRowClick={(event) => {
          const route = routeForHistory(event);
          if (route) navigate(route);
        }}
      />
    </section>
  );
}

function routeForHistory(event: HistoryEventSummary): string | undefined {
  if (event.aggregateType === 'mission') return `/missions/${event.aggregateId}`;
  if (event.aggregateType === 'message') return `/messages/${event.aggregateId}`;
  if (event.aggregateType === 'feed') return `/feed/${event.aggregateId}`;
  if (event.aggregateType === 'decision') return `/decisions/${event.aggregateId}`;
  return undefined;
}

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
  const selected = useQuery({
    queryKey: ['feed-artifact', artifactId],
    queryFn: () => api.feedArtifact(artifactId),
    enabled: !!artifactId
  });
  const transactions = useQuery({
    queryKey: ['feed-transactions', artifactId],
    queryFn: () => api.feedTransactions(artifactId),
    enabled: !!artifactId
  });
  const column = createColumnHelper<FeedArtifactSummary>();
  const transactionColumn = createColumnHelper<FeedTransactionSummary>();
  const columns = [
    column.accessor('sourceId', { header: 'Source' }),
    column.accessor('type', { header: 'Type' }),
    column.accessor('accepted', { header: 'Status', cell: (info) => <StatusBadge value={info.getValue() ? 'ACCEPTED' : 'REJECTED'} /> }),
    column.accessor('rawPayloadHash', { header: 'Payload Hash' }),
    column.accessor((row) => row.diagnostics.length, { id: 'diagnostics', header: 'Diagnostics' })
  ];
  const ingest = useMutation({
    mutationFn: () => api.ingestFeed(draft),
    onSuccess: (batch: any) => {
      queryClient.invalidateQueries({ queryKey: ['feed-artifacts'] });
      const id = batch?.results?.[0]?.envelope?.id;
      if (id) navigate(`/feed/${id}`);
    }
  });
  const transactionColumns = [
    transactionColumn.accessor('type', { header: 'Type' }),
    transactionColumn.accessor('status', { header: 'Status', cell: (info) => <StatusBadge value={info.getValue()} /> }),
    transactionColumn.accessor('supported', { header: 'Supported', cell: (info) => String(info.getValue()) }),
    transactionColumn.accessor((row) => row.errors.length + row.warnings.length, { id: 'diagnostics', header: 'Diagnostics' })
  ];
  return (
    <section className="workspace">
      <div className="toolbar">
        <h2>Feed Artifacts</h2>
        <button onClick={() => ingest.mutate()}>Ingest Payload</button>
      </div>
      <div className="grid two">
        <div className="panel">
          <DataTable data={feed.data ?? []} columns={columns} onRowClick={(artifact) => navigate(`/feed/${artifact.id}`)} />
          <h3>Manual Feed Ingest</h3>
          <div className="form-grid">
            <label>Source<input value={draft.sourceId} onChange={(event) => setDraft({ ...draft, sourceId: event.target.value })} /></label>
            <label>Type<input value={draft.type} onChange={(event) => setDraft({ ...draft, type: event.target.value })} /></label>
          </div>
          <textarea
            className="compact-textarea"
            value={draft.rawPayload}
            onChange={(event) => setDraft({ ...draft, rawPayload: event.target.value })}
          />
        </div>
        <div className="panel detail-panel">
          <h3>Artifact Detail</h3>
          {selected.data ? (
            <>
              <div className="detail-row"><span>Status</span><StatusBadge value={selected.data.accepted ? 'ACCEPTED' : 'REJECTED'} /></div>
              <div className="detail-row"><span>Source</span><strong>{selected.data.sourceId}</strong></div>
              <div className="detail-row"><span>Type</span><strong>{selected.data.type}</strong></div>
              <div className="detail-row"><span>Received</span><strong>{selected.data.receivedAt || 'unknown'}</strong></div>
              <div className="detail-row"><span>Hash</span><code>{selected.data.rawPayloadHash}</code></div>
              <h4>Downstream Artifacts</h4>
              <ul>{selected.data.downstreamArtifactIds.map((id) => <li key={id}><code>{id}</code></li>)}</ul>
              <h4>Diagnostics</h4>
              <ul>{selected.data.diagnostics.map((item) => <li key={item}>{item}</li>)}</ul>
              <h4>Transactions</h4>
              <DataTable data={transactions.data ?? []} columns={transactionColumns} />
              <pre className="raw-panel">{selected.data.rawPayload || 'No raw payload retained.'}</pre>
            </>
          ) : (
            <p className="muted">Open a feed artifact to inspect retained raw traffic, diagnostics, and downstream IDs.</p>
          )}
        </div>
      </div>
    </section>
  );
}

export function ConfigPage() {
  const queryClient = useQueryClient();
  const config = useQuery({ queryKey: ['config'], queryFn: api.config });
  const points = useQuery({ queryKey: ['reference-points'], queryFn: () => api.referencePoints() });
  const addPoint = useMutation({
    mutationFn: () => api.createReferencePoint({
      identifier: `LOCAL${Math.floor(Math.random() * 1000)}`,
      pointType: 'FIX',
      latitude: 38.0,
      longitude: -77.0,
      altitudeFeet: 0,
      source: 'operator'
    }),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['reference-points'] })
  });
  const [referenceImport, setReferenceImport] = useState('type,identifier,latitude,longitude,altitudeFeet,source,version\nFIX,DEMOFIX,39.0,-76.0,0,local,v1');
  const previewImport = useMutation({ mutationFn: () => api.previewReferenceImport(referenceImport) });
  const applyImport = useMutation({
    mutationFn: () => api.applyReferenceImport(referenceImport),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['reference-points'] })
  });
  const column = createColumnHelper<ReferencePointSummary>();
  const columns = [
    column.accessor('identifier', { header: 'Identifier' }),
    column.accessor('pointType', { header: 'Type' }),
    column.accessor('latitude', { header: 'Lat' }),
    column.accessor('longitude', { header: 'Lon' }),
    column.accessor('altitudeFeet', { header: 'Alt Ft' }),
    column.accessor('source', { header: 'Source' })
  ];
  return (
    <section className="workspace">
      <div className="toolbar">
        <h2>Configuration</h2>
        <button onClick={() => addPoint.mutate()}>Add Local Fix</button>
      </div>
      <div className="grid two">
        <div className="panel">
          <h3>Runtime Settings</h3>
          <pre>{JSON.stringify(config.data ?? {}, null, 2)}</pre>
          <h3>Reference Import</h3>
          <textarea
            className="compact-textarea"
            value={referenceImport}
            onChange={(event) => setReferenceImport(event.target.value)}
          />
          <div className="toolbar compact">
            <button className="secondary" onClick={() => previewImport.mutate()}>Preview Import</button>
            <button onClick={() => applyImport.mutate()}>Apply Import</button>
          </div>
          {(previewImport.data || applyImport.data) && (
            <pre className="raw-panel">{JSON.stringify(previewImport.data ?? applyImport.data, null, 2)}</pre>
          )}
        </div>
        <div className="panel">
          <h3>Reference Points</h3>
          <DataTable data={points.data ?? []} columns={columns} />
        </div>
      </div>
    </section>
  );
}
