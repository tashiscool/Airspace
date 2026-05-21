import {
  ColumnDef,
  flexRender,
  getCoreRowModel,
  useReactTable
} from '@tanstack/react-table';

type Props<T> = {
  data: T[];
  columns: ColumnDef<T, any>[];
  onRowClick?: (row: T) => void;
  onRowDoubleClick?: (row: T) => void;
  isRowSelected?: (row: T) => boolean;
};

export function DataTable<T>({ data, columns, onRowClick, onRowDoubleClick, isRowSelected }: Props<T>) {
  const table = useReactTable({ data, columns, getCoreRowModel: getCoreRowModel() });
  return (
    <div className="data-table-wrap">
      <table className="data-table">
        <thead>
          {table.getHeaderGroups().map((group) => (
            <tr key={group.id}>
              {group.headers.map((header) => (
                <th key={header.id}>{flexRender(header.column.columnDef.header, header.getContext())}</th>
              ))}
            </tr>
          ))}
        </thead>
        <tbody>
          {table.getRowModel().rows.map((row) => (
            <tr
              key={row.id}
              className={isRowSelected?.(row.original) ? 'selected-row' : undefined}
              aria-selected={isRowSelected?.(row.original)}
              tabIndex={onRowClick ? 0 : undefined}
              onClick={() => onRowClick?.(row.original)}
              onDoubleClick={() => onRowDoubleClick?.(row.original)}
              onKeyDown={(event) => {
                if (event.key === 'Enter') (onRowDoubleClick ?? onRowClick)?.(row.original);
                if (event.key === ' ') {
                  event.preventDefault();
                  onRowClick?.(row.original);
                }
                if (event.key === 'o' || event.key === 'O') onRowDoubleClick?.(row.original);
              }}
            >
              {row.getVisibleCells().map((cell) => (
                <td key={cell.id}>{flexRender(cell.column.columnDef.cell, cell.getContext())}</td>
              ))}
            </tr>
          ))}
          {!table.getRowModel().rows.length && (
            <tr>
              <td colSpan={columns.length} className="empty-cell">No records.</td>
            </tr>
          )}
        </tbody>
      </table>
    </div>
  );
}
