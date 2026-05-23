import { useState } from 'react';
import { useQuery } from '@tanstack/react-query';
import { createColumnHelper } from '@tanstack/react-table';
import { useNavigate } from 'react-router-dom';
import { Search } from 'lucide-react';
import { api } from '../api/client';
import { DataTable } from '../components/DataTable';
import { ErrorNotice } from '../components/Notices';
import { StatusBadge } from '../components/StatusBadge';
import type { SearchResultSummary } from '../types';
import { fmtZ } from '../lib/viewModels';
import { searchTypeLabel, selectionForSearch } from '../lib/searchView';
import { writeWorkbenchJson } from '../lib/workbenchState';

export function SearchPage() {
  const navigate = useNavigate();
  const [query, setQuery] = useState('');
  const search = useQuery({ queryKey: ['search', query], queryFn: () => api.search(query), enabled: query.trim().length > 1 });
  const column = createColumnHelper<SearchResultSummary>();
  const columns = [
    column.accessor((row) => searchTypeLabel(row.type), { id: 'type', header: 'Type' }),
    column.accessor('title', { header: 'Title' }),
    column.accessor('status', { header: 'Status', cell: (info) => info.getValue() ? <StatusBadge value={info.getValue()} /> : null }),
    column.accessor('snippet', { header: 'Snippet' }),
    column.accessor('updatedAt', { header: 'Updated', cell: (info) => fmtZ(info.getValue()) })
  ];
  return (
    <section className="workspace operational-page">
      <div className="page-header"><div><h2><Search size={18} /> Search</h2><p>Find missions, messages, feed artifacts, decisions, NOTAM constraints, and history.</p></div></div>
      <div className="search-box large"><Search size={15} /><input value={query} onChange={(event) => setQuery(event.target.value)} placeholder="Mission, message, feed artifact, history..." /></div>
      <ErrorNotice error={search.error} title="Search unavailable" />
      <DataTable
        data={search.data ?? []}
        columns={columns}
        onRowClick={(row) => {
          writeWorkbenchJson('airspace.workbench.selection', selectionForSearch(row));
          window.dispatchEvent(new Event('airspace-workbench-selection'));
        }}
        onRowDoubleClick={(row) => row.route && navigate(row.route)}
      />
    </section>
  );
}
