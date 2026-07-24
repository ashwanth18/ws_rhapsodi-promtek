import { ReactNode } from 'react'
import { cn } from '../../lib/cn'

export type Column<T> = {
  key: string
  header: string
  render: (row: T) => ReactNode
  className?: string
  sortable?: boolean
}

type Props<T> = {
  columns: Column<T>[]
  rows: T[]
  rowKey: (row: T) => string | number
  emptyMessage?: string
  onRowClick?: (row: T) => void
  rowClassName?: (row: T) => string
  sortKey?: string
  sortDir?: 'asc' | 'desc'
  onSort?: (key: string) => void
}

export default function DataTable<T>({
  columns,
  rows,
  rowKey,
  emptyMessage = 'No data available',
  onRowClick,
  rowClassName,
  sortKey,
  sortDir,
  onSort,
}: Props<T>) {
  if (rows.length === 0) {
    return (
      <div className="rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--card-surface)] px-6 py-12 text-center text-sm text-[var(--text-muted)]">
        {emptyMessage}
      </div>
    )
  }

  return (
    <div className="overflow-hidden rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--card-surface)] shadow-card">
      <div className="overflow-x-auto">
        <table className="w-full min-w-[640px] border-collapse text-sm">
          <thead>
            <tr className="border-b border-[var(--border)] bg-[var(--surface-2)]">
              {columns.map((col) => (
                <th
                  key={col.key}
                  className={cn(
                    'px-4 py-3 text-left text-xs font-semibold uppercase tracking-wider text-[var(--text-faint)]',
                    col.sortable && onSort && 'cursor-pointer select-none hover:text-[var(--text-secondary)]',
                    col.className
                  )}
                  onClick={
                    col.sortable && onSort
                      ? () => onSort(col.key)
                      : undefined
                  }
                >
                  <span className="inline-flex items-center gap-1">
                    {col.header}
                    {sortKey === col.key && (
                      <span className="text-[var(--accent)]">
                        {sortDir === 'asc' ? '↑' : '↓'}
                      </span>
                    )}
                  </span>
                </th>
              ))}
            </tr>
          </thead>
          <tbody>
            {rows.map((row, idx) => (
              <tr
                key={rowKey(row)}
                className={cn(
                  'border-b border-[var(--border)] transition-colors last:border-b-0',
                  idx % 2 === 1 && 'bg-[var(--table-row-alt)]',
                  onRowClick && 'cursor-pointer hover:bg-[var(--table-row-hover)]',
                  rowClassName?.(row)
                )}
                onClick={onRowClick ? () => onRowClick(row) : undefined}
              >
                {columns.map((col) => (
                  <td
                    key={col.key}
                    className={cn('px-4 py-3 text-[var(--text-secondary)]', col.className)}
                  >
                    {col.render(row)}
                  </td>
                ))}
              </tr>
            ))}
          </tbody>
        </table>
      </div>
    </div>
  )
}
