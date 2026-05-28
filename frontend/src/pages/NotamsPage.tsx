import { useEffect, useState } from 'react';
import { useQuery } from '@tanstack/react-query';
import { Link, useNavigate } from 'react-router-dom';
import { Radio, ShieldAlert } from 'lucide-react';
import { api } from '../api/client';
import { ErrorNotice, QueryNotice } from '../components/Notices';
import { StatusBadge } from '../components/StatusBadge';
import { notamRowsFromSources } from '../lib/viewModels';
import { writeWorkbenchJson, type WorkbenchSelection } from '../lib/workbenchState';

export function NotamsPage() {
  const navigate = useNavigate();
  const messages = useQuery({ queryKey: ['messages'], queryFn: api.messages });
  const search = useQuery({ queryKey: ['search', 'NOTAM'], queryFn: () => api.search('NOTAM') });
  const rows = notamRowsFromSources(messages.data ?? [], [], search.data ?? []);
  const [selectedKey, setSelectedKey] = useState('');
  const selected = rows.find((row) => `${row.source}:${row.id}` === selectedKey) ?? rows[0];
  useEffect(() => {
    const selection: WorkbenchSelection = {
      messageId: selected?.source === 'MESSAGE' ? selected.id : undefined,
      sourceFamily: 'NOTAM',
      label: selected?.title ?? 'NOTAM constraints',
      lockState: selected?.status ?? `${rows.length} retained`
    };
    writeWorkbenchJson('airspace.workbench.selection', selection);
    window.dispatchEvent(new Event('airspace-workbench-selection'));
  }, [selected, rows.length]);
  return (
    <section className="workspace operational-page">
      <div className="page-header">
        <div>
          <h2><Radio size={18} /> NOTAM Constraints</h2>
          <p>Domestic, FDC, ICAO, SNOWTAM, BIRDTAM, ASHTAM, GENOT, and service NOTAM traffic. These are constraints, not CARF/ALTRV reservations.</p>
        </div>
        <StatusBadge value={`${rows.length} NOTAM-LIKE`} />
      </div>
      <div className="notice-strip"><ShieldAlert size={15} /> A NOTAM may affect deconfliction and decisions, but only CARF/ALTRV source families become reservations.</div>
      <div className="notice-strip"><ShieldAlert size={15} /> RVR, SMGCS, LVO, and low-visibility procedure traffic is advisory procedural context: Airspace helps operators reconcile local FAA terms with ICAO/operator language, but it does not declare airport procedure state.</div>
      <div className="notice-stack">
        <QueryNotice query={messages} label="Messages" />
        <ErrorNotice error={search.error} title="NOTAM search unavailable" />
      </div>
      <div className="panel">
        <table className="data-table">
          <thead><tr><th>ID</th><th>Source</th><th>Family</th><th>Status</th><th>Title</th><th>Text</th></tr></thead>
          <tbody>
            {rows.map((row) => (
              <tr
                key={`${row.source}:${row.id}`}
                className={`${row.source}:${row.id}` === `${selected?.source}:${selected?.id}` ? 'selected-row' : undefined}
                onClick={() => setSelectedKey(`${row.source}:${row.id}`)}
                onDoubleClick={() => row.route && navigate(row.route)}
              >
                <td><Link to={row.route ?? '#'}>{row.id}</Link></td>
                <td>{row.source}</td>
                <td>{row.family}</td>
                <td><StatusBadge value={row.status ?? 'RETAINED'} /></td>
                <td>{row.title}</td>
                <td>{row.text}</td>
              </tr>
            ))}
            {!rows.length && <tr><td colSpan={6} className="empty-cell">No NOTAM-like records found from messages, supplements, or search.</td></tr>}
          </tbody>
        </table>
      </div>
    </section>
  );
}
