import { useEffect, useMemo, useState } from 'react';
import type { ReactNode } from 'react';
import { useMutation, useQueries, useQuery, useQueryClient } from '@tanstack/react-query';
import { createColumnHelper } from '@tanstack/react-table';
import { useNavigate } from 'react-router-dom';
import { ArrowDownUp, Filter, Mail, Plus, Radio, Search, Upload, X } from 'lucide-react';
import { api } from '../api/client';
import { DataTable } from '../components/DataTable';
import { ErrorNotice, MutationNotice, QueryNotice } from '../components/Notices';
import { StatusBadge } from '../components/StatusBadge';
import type { ExplorerRow } from '../lib/viewModels';
import { compactId, fmtZ, rowFromFeed, rowFromMessage, rowsFromMissionDetail } from '../lib/viewModels';
import {
  actionsForWorkbenchRow,
  DEFAULT_LAYOUT_PREFS,
  isAttentionRow,
  routeForWorkbenchAction,
  readWorkbenchJson,
  routeForWorkbenchRow,
  selectionFromRow,
  writeWorkbenchJson,
  type WorkbenchLayoutPrefs
} from '../lib/workbenchState';
import type { MessageSummary, MissionSummary } from '../types';

type SortMode = 'TIME' | 'CATEGORY';
type FamilyFilter = ExplorerRow['family'];
type StatusFilter = string;

const rowColumn = createColumnHelper<ExplorerRow>();
const rowColumns = [
  rowColumn.accessor('family', { header: 'Family' }),
  rowColumn.accessor('title', { header: 'Title' }),
  rowColumn.accessor('subtitle', { header: 'Context' }),
  rowColumn.accessor('status', { header: 'Status', cell: (info) => info.getValue() ? <StatusBadge value={info.getValue()} /> : null }),
  rowColumn.accessor('time', { header: 'Time', cell: (info) => fmtZ(info.getValue()) })
];

export function ExplorerPage() {
  const navigate = useNavigate();
  const queryClient = useQueryClient();
  const [filter, setFilter] = useState('');
  const [sortMode, setSortMode] = useState<SortMode>('TIME');
  const [storedPrefs, setStoredPrefs] = useState<WorkbenchLayoutPrefs>(() => readWorkbenchJson('airspace.workbench.layout', DEFAULT_LAYOUT_PREFS));
  const [familyFilter, setFamilyFilter] = useState<Set<FamilyFilter>>(() => new Set(storedPrefs.explorerFilters.families as FamilyFilter[]));
  const [statusFilter, setStatusFilter] = useState<Set<StatusFilter>>(() => new Set(storedPrefs.explorerFilters.statuses));
  const [attentionOnly, setAttentionOnly] = useState(storedPrefs.explorerFilters.attentionOnly);
  const [missionScope, setMissionScope] = useState<string | null>(null);
  const [messageScope, setMessageScope] = useState<string | null>(null);
  const [notamMode, setNotamMode] = useState(false);
  const [selectedKey, setSelectedKey] = useState<string | null>(null);

  const missions = useQuery({ queryKey: ['missions'], queryFn: api.missions });
  const messages = useQuery({ queryKey: ['messages'], queryFn: api.messages });
  const feed = useQuery({ queryKey: ['feed-artifacts'], queryFn: api.feedArtifacts });
  const metrics = useQuery({ queryKey: ['metrics'], queryFn: api.metrics });
  const missionDetails = useQueries({
    queries: (missions.data ?? []).slice(0, 12).map((mission) => ({
      queryKey: ['mission', mission.id, 'explorer'],
      queryFn: () => api.mission(mission.id),
      staleTime: 30_000
    }))
  });

  const create = useMutation({
    mutationFn: () => api.createMission({ title: 'Operational Mission', actor: 'planner' }),
    onSuccess: (mission) => {
      queryClient.invalidateQueries({ queryKey: ['missions'] });
      navigate(`/missions/${mission.id}`);
    }
  });
  const ingestWeatherObservation = useMutation({
    mutationFn: () => api.ingestFeed({
      sourceId: 'operator-console',
      type: 'WEATHER',
      rawPayload: 'METAR KJFK 200000Z 18012KT 2SM RA BKN010'
    }),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['feed-artifacts'] })
  });

  const rows = useMemo<ExplorerRow[]>(() => {
    const detailRows = missionDetails.flatMap((result) => rowsFromMissionDetail(result.data));
    const detailMissionIds = new Set(detailRows.filter((row) => row.family === 'MISSION').map((row) => row.id));
    const missionRows = (missions.data ?? [])
      .filter((mission) => !detailMissionIds.has(mission.id))
      .map((mission): ExplorerRow => ({
        key: `mission:${mission.id}`,
        family: 'MISSION',
        id: mission.id,
        title: mission.missionNumber,
        subtitle: mission.title ?? 'Mission workspace',
        status: mission.status,
        missionId: mission.id,
        route: `/missions/${mission.id}`,
        time: mission.updatedAt,
        preview: `${mission.reservationCount} reservation(s) · ${mission.lockedBy ? `locked by ${mission.lockedBy}` : 'unlocked'}`
      }));
    const allReservationIds = new Set(detailRows.filter((row) => row.family === 'RESERVATION').map((row) => row.id));
    const messageRows = (messages.data ?? []).map(rowFromMessage);
    const feedRows = (feed.data ?? []).map(rowFromFeed);
    return [...detailRows, ...missionRows, ...messageRows, ...feedRows]
      .filter((row) => row.family !== 'RESERVATION' || !allReservationIds.has(row.id) || row.key.startsWith('reservation:'));
  }, [missionDetails, missions.data, messages.data, feed.data]);

  const filtered = useMemo(() => {
    const q = filter.trim().toLowerCase();
    let output = rows;
    if (notamMode) output = output.filter((row) => row.family === 'NOTAM');
    if (attentionOnly) output = output.filter(isAttentionRow);
    if (familyFilter.size) output = output.filter((row) => familyFilter.has(row.family));
    if (statusFilter.size) output = output.filter((row) => row.status && statusFilter.has(row.status));
    if (missionScope) output = output.filter((row) => row.missionId === missionScope || row.id === missionScope);
    if (messageScope) output = output.filter((row) => row.id === messageScope);
    if (q) output = output.filter((row) => `${row.id} ${row.title} ${row.subtitle} ${row.preview}`.toLowerCase().includes(q));
    return [...output].sort((a, b) => {
      if (sortMode === 'CATEGORY') return a.family.localeCompare(b.family) || a.title.localeCompare(b.title);
      return String(b.time ?? '').localeCompare(String(a.time ?? '')) || a.family.localeCompare(b.family);
    });
  }, [rows, filter, familyFilter, statusFilter, attentionOnly, missionScope, messageScope, sortMode, notamMode]);

  const selected = filtered.find((row) => row.key === selectedKey) ?? filtered[0];
  const missionGroups = useMemo(() => groupMissions(missions.data ?? []), [missions.data]);
  const messageGroups = useMemo(() => groupMessages(messages.data ?? []), [messages.data]);
  const statuses = useMemo(() => {
    const values = new Set(rows.map((row) => row.status).filter(Boolean) as string[]);
    return [...values].sort();
  }, [rows]);

  useEffect(() => {
    const currentPrefs = readWorkbenchJson('airspace.workbench.layout', DEFAULT_LAYOUT_PREFS);
    const next = {
      ...currentPrefs,
      explorerFilters: {
        families: [...familyFilter],
        statuses: [...statusFilter],
        attentionOnly
      }
    };
    setStoredPrefs(next);
    writeWorkbenchJson('airspace.workbench.layout', next);
  }, [familyFilter, statusFilter, attentionOnly]);

  useEffect(() => {
    if (!selected) return;
    writeWorkbenchJson('airspace.workbench.selection', selectionFromRow(selected));
    window.dispatchEvent(new Event('airspace-workbench-selection'));
  }, [selected]);

  function openRow(row?: ExplorerRow) {
    const route = routeForWorkbenchRow(row);
    if (route) navigate(route);
  }

  function toggleFamily(family: FamilyFilter) {
    setFamilyFilter((current) => {
      const next = new Set(current);
      next.has(family) ? next.delete(family) : next.add(family);
      return next;
    });
  }

  function toggleStatus(status: StatusFilter) {
    setStatusFilter((current) => {
      const next = new Set(current);
      next.has(status) ? next.delete(status) : next.add(status);
      return next;
    });
  }

  return (
    <section
      className="explorer-workspace"
      tabIndex={0}
      onKeyDown={(event) => {
        if (!filtered.length) return;
        const currentIndex = Math.max(0, filtered.findIndex((row) => row.key === selected?.key));
        if (event.key === 'ArrowDown' || event.key === 'ArrowUp') {
          event.preventDefault();
          const nextIndex = event.key === 'ArrowDown'
            ? Math.min(filtered.length - 1, currentIndex + 1)
            : Math.max(0, currentIndex - 1);
          setSelectedKey(filtered[nextIndex].key);
        }
        if (event.key === 'Enter') openRow(selected);
        if (event.key === 'Escape') {
          setMissionScope(null);
          setMessageScope(null);
          setFamilyFilter(new Set());
          setStatusFilter(new Set());
          setAttentionOnly(false);
          setNotamMode(false);
        }
      }}
    >
      <aside className="split-tree">
        <TreePanel title="MISSIONS · BY STATUS" loading={missions.isLoading}>
          {missionGroups.map(([status, group]) => (
            <details key={status} open>
              <summary>{status}<span>{group.length}</span></summary>
              {group.map((mission) => (
                <button
                  key={mission.id}
                  className={missionScope === mission.id ? 'tree-row active' : 'tree-row'}
                  onClick={() => {
                    setMissionScope(mission.id);
                    setMessageScope(null);
                  }}
                  onDoubleClick={() => navigate(`/missions/${mission.id}`)}
                >
                  <span>{mission.missionNumber}</span>
                  <StatusBadge value={mission.status} />
                </button>
              ))}
            </details>
          ))}
        </TreePanel>
        <TreePanel title="MESSAGES · BY FAMILY" loading={messages.isLoading}>
          {messageGroups.map(([family, group]) => (
            <details key={family} open>
              <summary>{family}<span>{group.length}</span></summary>
              {group.slice(0, 20).map((message) => (
                <button
                  key={message.id}
                  className={messageScope === message.id ? 'tree-row active' : 'tree-row'}
                  onClick={() => {
                    setMessageScope(message.id);
                    setMissionScope(null);
                  }}
                  onDoubleClick={() => navigate(`/messages/${message.id}`)}
                >
                  <span>{message.subject || compactId(message.id)}</span>
                  <StatusBadge value={message.status} />
                </button>
              ))}
            </details>
          ))}
        </TreePanel>
      </aside>

      <main className="explorer-board">
        <div className="board-toolbar">
          <span className="board-title">Mission Explorer</span>
          <span className="board-count">{filtered.length}/{rows.length}</span>
          <div className="search-box"><Search size={14} /><input value={filter} onChange={(event) => setFilter(event.target.value)} placeholder="Filter mission, reservation, message, NOTAM..." /></div>
          <button className="secondary" onClick={() => setSortMode(sortMode === 'TIME' ? 'CATEGORY' : 'TIME')}><ArrowDownUp size={14} /> {sortMode}</button>
          <button className={notamMode ? 'active-filter' : 'secondary'} onClick={() => setNotamMode((value) => !value)}><Radio size={14} /> NOTAM View</button>
          <button className={attentionOnly ? 'active-filter' : 'secondary'} onClick={() => setAttentionOnly((value) => !value)}>Attention Only</button>
          <button onClick={() => create.mutate()}><Plus size={14} /> Mission</button>
          <button className="secondary" onClick={() => ingestWeatherObservation.mutate()}><Upload size={14} /> Ingest Weather</button>
        </div>
        <div className="filter-strip">
          <Filter size={13} />
          {(['MISSION', 'RESERVATION', 'MESSAGE', 'NOTAM', 'APREQ', 'APPROVAL', 'FEED', 'DECISION'] as FamilyFilter[]).map((family) => (
            <button key={family} className={familyFilter.has(family) ? 'chip active' : 'chip'} onClick={() => toggleFamily(family)}>{family}</button>
          ))}
          <span className="filter-divider" />
          {statuses.map((status) => (
            <button key={status} className={statusFilter.has(status) ? 'chip active' : 'chip'} onClick={() => toggleStatus(status)}>{status}</button>
          ))}
          {(missionScope || messageScope || familyFilter.size > 0 || statusFilter.size > 0 || notamMode) && (
            <button
              className="chip clear"
              onClick={() => {
                setMissionScope(null);
                setMessageScope(null);
                setFamilyFilter(new Set());
                setStatusFilter(new Set());
                setAttentionOnly(false);
                setNotamMode(false);
              }}
            >
              <X size={12} /> Clear
            </button>
          )}
        </div>
        <div className="notice-stack">
          <QueryNotice query={missions} label="Missions" />
          <QueryNotice query={messages} label="Messages" />
          <ErrorNotice error={feed.error} title="Feed artifacts unavailable" />
          <ErrorNotice error={metrics.error} title="Metrics unavailable" />
          <MutationNotice mutation={create} label="Create mission" />
          <MutationNotice mutation={ingestWeatherObservation} label="Weather feed ingest" />
        </div>
        <div className="explorer-table-zone">
          <DataTable
            data={filtered}
            columns={rowColumns}
            onRowClick={(row) => {
              setSelectedKey(row.key);
            }}
            onRowDoubleClick={openRow}
            isRowSelected={(row) => row.key === selected?.key}
          />
        </div>
        <section className="preview-panel">
          {selected ? (
            <>
              <div className="preview-primary">
                <strong>{selected.title}</strong>
                <p>{selected.subtitle}</p>
                <dl className="mini-kv">
                  <dt>Family</dt><dd>{selected.family}</dd>
                  <dt>ID</dt><dd>{selected.id}</dd>
                  <dt>Time</dt><dd>{fmtZ(selected.time)}</dd>
                </dl>
              </div>
              <StatusBadge value={selected.status ?? selected.family} />
              <pre className="preview-text">{selected.preview}</pre>
              <div className="toolbar compact">
                {actionsForWorkbenchRow(selected).map((action) => (
                  <button
                    key={action}
                    className={action === 'open' ? undefined : 'secondary'}
                    onClick={() => {
                      const route = routeForWorkbenchAction(action, selected);
                      if (route) navigate(route);
                    }}
                  >
                    {actionLabel(action)}
                  </button>
                ))}
                {selected.family === 'MESSAGE' || selected.family === 'NOTAM' ? <button className="secondary" onClick={() => navigate('/messages')}><Mail size={14} /> Queue</button> : null}
              </div>
            </>
          ) : (
            <p className="muted">No records match the current filters.</p>
          )}
        </section>
      </main>
      <aside className="metric-sidebar">
        {Object.entries(metrics.data ?? {})
          .filter(([name]) => name.startsWith('product.'))
          .slice(0, 10)
          .map(([name, value]) => (
            <div className="metric-tile" key={name}>
              <span>{name.replace('product.', '')}</span>
              <strong>{Math.round(value)}</strong>
            </div>
          ))}
      </aside>
    </section>
  );
}

function actionLabel(action: ReturnType<typeof actionsForWorkbenchRow>[number]) {
  return action.replace(/-/g, ' ').replace(/\b\w/g, (char) => char.toUpperCase());
}

function TreePanel({ title, loading, children }: { title: string; loading?: boolean; children: ReactNode }) {
  return (
    <section className="tree-panel">
      <header>{title}{loading && <small>LOADING...</small>}</header>
      <div>{children}</div>
    </section>
  );
}

function groupMissions(missions: MissionSummary[]) {
  const groups = new Map<string, MissionSummary[]>();
  for (const mission of missions) {
    const key = mission.status || 'UNKNOWN';
    groups.set(key, [...(groups.get(key) ?? []), mission]);
  }
  return [...groups.entries()].sort(([a], [b]) => a.localeCompare(b));
}

function groupMessages(messages: MessageSummary[]) {
  const groups = new Map<string, MessageSummary[]>();
  for (const message of messages) {
    const key = message.family || 'UNKNOWN';
    groups.set(key, [...(groups.get(key) ?? []), message]);
  }
  return [...groups.entries()].sort(([a], [b]) => a.localeCompare(b));
}
