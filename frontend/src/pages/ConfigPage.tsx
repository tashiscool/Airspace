import { useEffect, useState } from 'react';
import { useMutation, useQuery, useQueryClient } from '@tanstack/react-query';
import { createColumnHelper } from '@tanstack/react-table';
import { Database } from 'lucide-react';
import { api } from '../api/client';
import { DataTable } from '../components/DataTable';
import { ErrorNotice, MutationNotice, QueryNotice } from '../components/Notices';
import { writeWorkbenchJson, type WorkbenchSelection } from '../lib/workbenchState';
import type { ReferencePointSummary } from '../types';

type ConfigNode = 'navaids' | 'fixes' | 'airspace' | 'recipients' | 'users' | 'separation' | 'imports' | 'system';

const AIRSPACE_GROUPS = [
  { id: 'AG-PAC-W', name: 'PACIFIC WEST', owner: '613 AOC', classification: 'OPERATIONAL', members: 14 },
  { id: 'AG-ATL-E', name: 'ATLANTIC EAST', owner: '618 AOC', classification: 'OPERATIONAL', members: 9 },
  { id: 'AG-GOM', name: 'GULF OF MEXICO', owner: 'CENTCOM', classification: 'OPERATIONAL', members: 6 },
  { id: 'AG-ARC', name: 'ARCTIC', owner: 'NORAD', classification: 'RESTRICTED', members: 4 }
];

const RECIPIENTS = [
  { id: 'RC-001', handle: 'FAA-ATCSCC', channel: 'USNS', address: 'atcscc@usns.faa', priority: 'ROUTINE' },
  { id: 'RC-002', handle: 'NORAD-CMOC', channel: 'USNS', address: 'cmoc@norad.usns', priority: 'PRIORITY' },
  { id: 'RC-003', handle: 'ZJX-CTR', channel: 'USNS', address: 'zjx@usns.faa', priority: 'ROUTINE' },
  { id: 'RC-004', handle: 'FLEET-W', channel: 'INTERNAL', address: 'fleet-w@carf', priority: 'PRIORITY' },
  { id: 'RC-005', handle: '618-AOC-DUTY', channel: 'INTERNAL', address: 'duty@618aoc', priority: 'IMMEDIATE' }
];

const USERS = [
  { id: 'U-001', username: 'j.planner', role: 'PLANNER', status: 'ACTIVE' },
  { id: 'U-002', username: 'k.controller', role: 'CONTROLLER', status: 'ACTIVE' },
  { id: 'U-003', username: 'm.approver', role: 'APPROVER', status: 'ACTIVE' },
  { id: 'U-004', username: 's.admin', role: 'ADMIN', status: 'ACTIVE' },
  { id: 'U-005', username: 't.observer', role: 'READONLY', status: 'SUSPENDED' }
];

const PERMISSIONS = [
  'reservation.create',
  'reservation.submit',
  'reservation.approve',
  'reservation.reject',
  'deconflict.run',
  'deconflict.force',
  'message.transmit',
  'refdata.edit',
  'users.manage',
  'system.configure'
];

export function ConfigPage() {
  const queryClient = useQueryClient();
  const [selected, setSelected] = useState<ConfigNode>('navaids');
  const config = useQuery({ queryKey: ['config'], queryFn: api.config });
  const points = useQuery({ queryKey: ['reference-points'], queryFn: () => api.referencePoints() });
  const [referenceImport, setReferenceImport] = useState('type,identifier,latitude,longitude,altitudeFeet,source,version\nFIX,DEMOFIX,39.0,-76.0,0,local,v1');
  const previewImport = useMutation({ mutationFn: () => api.previewReferenceImport(referenceImport) });
  const applyImport = useMutation({
    mutationFn: () => api.applyReferenceImport(referenceImport),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['reference-points'] })
  });
  const addPoint = useMutation({
    mutationFn: () => api.createReferencePoint({
      identifier: `LOCAL${Math.floor(Math.random() * 1000)}`,
      pointType: 'FIX',
      latitude: 38,
      longitude: -77,
      altitudeFeet: 0,
      source: 'operator'
    }),
    onSuccess: () => queryClient.invalidateQueries({ queryKey: ['reference-points'] })
  });
  useEffect(() => {
    const selection: WorkbenchSelection = {
      sourceFamily: 'REFERENCE',
      label: `Config · ${selected}`,
      lockState: `${points.data?.length ?? 0} reference points`
    };
    writeWorkbenchJson('airspace.workbench.selection', selection);
    window.dispatchEvent(new Event('airspace-workbench-selection'));
  }, [selected, points.data?.length]);
  return (
    <section className="config-workspace">
      <aside className="config-tree">
        <header><Database size={14} /> Configuration</header>
        {([
          ['navaids', 'NAVAIDs'],
          ['fixes', 'Fixes / Reference Points'],
          ['airspace', 'Airspace Groups'],
          ['recipients', 'Recipients'],
          ['users', 'Users / RBAC'],
          ['separation', 'Flight Path Separation'],
          ['imports', 'Reference Imports'],
          ['system', 'System Configuration']
        ] as Array<[ConfigNode, string]>).map(([id, label]) => <button key={id} className={selected === id ? 'active' : ''} onClick={() => setSelected(id)}>{label}</button>)}
      </aside>
      <main className="panel">
        <div className="notice-stack">
          <ErrorNotice error={config.error} title="Config unavailable" />
          <QueryNotice query={points} label="Reference points" />
          <MutationNotice mutation={previewImport} label="Preview import" />
          <MutationNotice mutation={applyImport} label="Apply import" />
          <MutationNotice mutation={addPoint} label="Add reference point" />
        </div>
        {selected === 'navaids' && <ReferenceTable rows={(points.data ?? []).filter((point) => point.pointType === 'NAVAID')} onAdd={() => addPoint.mutate()} />}
        {selected === 'fixes' && <ReferenceTable rows={(points.data ?? []).filter((point) => point.pointType !== 'NAVAID')} onAdd={() => addPoint.mutate()} />}
        {selected === 'airspace' && <AirspaceGroupsPanel />}
        {selected === 'recipients' && <RecipientsPanel />}
        {selected === 'users' && <UsersPanel />}
        {selected === 'separation' && <SeparationPanel />}
        {selected === 'imports' && (
          <>
            <h3>Reference Import</h3>
            <textarea className="compact-textarea" value={referenceImport} onChange={(event) => setReferenceImport(event.target.value)} />
            <div className="toolbar compact">
              <button className="secondary" onClick={() => previewImport.mutate()}>Preview Import</button>
              <button onClick={() => applyImport.mutate()}>Apply Import</button>
            </div>
            {(previewImport.data || applyImport.data) && <pre className="raw-panel">{JSON.stringify(previewImport.data ?? applyImport.data, null, 2)}</pre>}
          </>
        )}
        {selected === 'system' && <pre className="raw-panel">{JSON.stringify(config.data ?? {}, null, 2)}</pre>}
      </main>
    </section>
  );
}

function AirspaceGroupsPanel() {
  return (
    <ConfigTable
      title="Airspace Groups"
      head={['Group', 'Name', 'Owner', 'Class', 'Members']}
      rows={AIRSPACE_GROUPS.map((group) => [group.id, group.name, group.owner, group.classification, String(group.members)])}
      note="Local operational grouping surface. Backend persistence can replace these reference rows when an airspace-group endpoint is added."
    />
  );
}

function RecipientsPanel() {
  return (
    <ConfigTable
      title="Recipients"
      head={['ID', 'Handle', 'Channel', 'Address', 'Priority']}
      rows={RECIPIENTS.map((recipient) => [recipient.id, recipient.handle, recipient.channel, recipient.address, recipient.priority])}
      note="Recipient directory mirrors the legacy CARF workflow shape; message send/reply/forward still uses the real Airspace message API."
    />
  );
}

function UsersPanel() {
  return (
    <section>
      <div className="panel-heading"><h3>Users / RBAC</h3><span>local role matrix</span></div>
      <div className="config-split">
        <ConfigTable title="Users" head={['ID', 'Username', 'Role', 'Status']} rows={USERS.map((user) => [user.id, user.username, user.role, user.status])} />
        <div>
          <h4>Permission Matrix</h4>
          <table className="data-table">
            <thead><tr><th>Permission</th><th>Planner</th><th>Controller</th><th>Approver</th><th>Admin</th></tr></thead>
            <tbody>
              {PERMISSIONS.map((permission) => (
                <tr key={permission}>
                  <td>{permission}</td>
                  <td>{permission.startsWith('reservation.') ? 'Y' : '-'}</td>
                  <td>{permission.startsWith('deconflict') || permission === 'message.transmit' ? 'Y' : '-'}</td>
                  <td>{permission.includes('approve') || permission.includes('reject') || permission === 'deconflict.force' ? 'Y' : '-'}</td>
                  <td>Y</td>
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      </div>
    </section>
  );
}

function SeparationPanel() {
  const [lateral, setLateral] = useState(120);
  const [vertical, setVertical] = useState(2000);
  const [time, setTime] = useState(15);
  return (
    <section>
      <div className="panel-heading"><h3>Flight Path Separation</h3><span>operator policy preview</span></div>
      <div className="policy-grid">
        <label>Lateral NM<input type="number" value={lateral} onChange={(event) => setLateral(Number(event.target.value))} /></label>
        <label>Vertical FT<input type="number" value={vertical} onChange={(event) => setVertical(Number(event.target.value))} /></label>
        <label>Minimum Duration MIN<input type="number" value={time} onChange={(event) => setTime(Number(event.target.value))} /></label>
      </div>
      <p className="muted">These controls mirror the legacy CARF configuration workflow. The authoritative deconfliction thresholds still come from the backend engine/config APIs.</p>
    </section>
  );
}

function ConfigTable({ title, head, rows, note }: { title: string; head: string[]; rows: string[][]; note?: string }) {
  return (
    <section>
      <div className="panel-heading"><h3>{title}</h3><span>{rows.length}</span></div>
      {note && <p className="muted">{note}</p>}
      <table className="data-table">
        <thead><tr>{head.map((item) => <th key={item}>{item}</th>)}</tr></thead>
        <tbody>
          {rows.map((row) => (
            <tr key={row.join(':')}>{row.map((cell, index) => <td key={`${cell}:${index}`}>{cell}</td>)}</tr>
          ))}
        </tbody>
      </table>
    </section>
  );
}

function ReferenceTable({ rows, onAdd }: { rows: ReferencePointSummary[]; onAdd: () => void }) {
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
    <>
      <div className="panel-heading"><h3>Reference Points</h3><button onClick={onAdd}>Add Local Fix</button></div>
      <DataTable data={rows} columns={columns} />
    </>
  );
}
