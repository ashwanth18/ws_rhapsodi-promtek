import { type ReactNode, useEffect, useState } from 'react'
import { Link } from 'react-router-dom'
import DateTimeRangeField from '../components/DateTimeRangeField'
import DateTimeText from '../components/DateTimeText'
import GlassCard from '../components/GlassCard'
import Button from '../components/ui/button'
import DataTable, { Column } from '../components/ui/DataTable'
import Select from '../components/ui/select'
import StatusBadge from '../components/ui/StatusBadge'
import { SectionHeader } from '../components/ui/SectionHeader'
import { useRuntimeConfig } from '../config/RuntimeConfig'
import SidebarLayout from './SidebarLayout'

type Row = {
  event_id: string
  sent_utc: string | null
  batch_id: string | null
  completed: boolean
  weightment_count: number
}

function FilterField({
  label,
  children,
  className = '',
}: {
  label: string
  children: ReactNode
  className?: string
}) {
  return (
    <label
      className={`flex shrink-0 flex-col gap-1 rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2 ${className}`}
    >
      <span className="text-[10px] font-semibold uppercase tracking-[0.14em] text-[var(--text-faint)]">
        {label}
      </span>
      {children}
    </label>
  )
}

function toFilterIsoStart(dateValue: string): string | undefined {
  return dateValue ? `${dateValue}T00:00:00Z` : undefined
}

function toFilterIsoEnd(dateValue: string): string | undefined {
  return dateValue ? `${dateValue}T23:59:59.999Z` : undefined
}

export default function BatchListPage() {
  const { apiBase } = useRuntimeConfig()
  const [rows, setRows] = useState<Row[]>([])
  const [loading, setLoading] = useState(false)
  const [total, setTotal] = useState(0)
  const [statusFilter, setStatusFilter] = useState<'not_completed' | 'completed' | 'all'>(
    'not_completed'
  )
  const [batchQuery, setBatchQuery] = useState('')
  const [timeFrom, setTimeFrom] = useState('')
  const [timeTo, setTimeTo] = useState('')
  const [timeSort, setTimeSort] = useState<'asc' | 'desc'>('desc')
  const [page, setPage] = useState(1)
  const pageSize = 8

  const loadRows = async () => {
    try {
      setLoading(true)
      const search = new URLSearchParams()
      search.set('limit', String(pageSize))
      search.set('offset', String((page - 1) * pageSize))
      search.set('status', statusFilter)
      search.set('time_sort', timeSort)
      if (batchQuery.trim()) search.set('batch_query', batchQuery.trim())
      const timeFromIso = toFilterIsoStart(timeFrom)
      const timeToIso = toFilterIsoEnd(timeTo)
      if (timeFromIso) search.set('time_from', timeFromIso)
      if (timeToIso) search.set('time_to', timeToIso)
      const res = await fetch(`${apiBase}/webhook_weightments/summary?${search.toString()}`)
      const json = await res.json()
      setRows(json.rows || [])
      setTotal(json.total ?? 0)
    } finally {
      setLoading(false)
    }
  }

  useEffect(() => {
    void loadRows()
  }, [apiBase, page, statusFilter, batchQuery, timeFrom, timeTo, timeSort])

  useEffect(() => {
    setPage(1)
  }, [statusFilter, batchQuery, timeFrom, timeTo, timeSort])

  const pageCount = Math.max(1, Math.ceil(total / pageSize))
  const safePage = Math.min(page, pageCount)

  const columns: Column<Row>[] = [
    {
      key: 'batch_id',
      header: 'Batch ID',
      render: (row) => <span className="font-medium text-[var(--text-primary)]">{row.batch_id ?? '—'}</span>,
    },
    {
      key: 'sent_utc',
      header: 'Released',
      sortable: true,
      render: (row) => <DateTimeText value={row.sent_utc} />,
    },
    {
      key: 'status',
      header: 'Status',
      render: (row) => (
        <StatusBadge
          label={row.completed ? 'Complete' : 'In progress'}
          tone={row.completed ? 'good' : 'warn'}
        />
      ),
    },
    {
      key: 'weightment_count',
      header: 'Weightments',
      className: 'text-right',
      render: (row) => row.weightment_count,
    },
    {
      key: 'actions',
      header: '',
      className: 'text-right',
      render: (row) => (
        <Link to={`/batches/${encodeURIComponent(row.event_id)}`}>
          <Button size="sm">Open</Button>
        </Link>
      ),
    },
  ]

  return (
    <SidebarLayout>
      <div className="px-5 py-5 lg:px-6">
        <SectionHeader
          title="Batches"
          description="Released batches grouped from MES webhook weightments."
          action={
            <Button onClick={() => loadRows()} disabled={loading}>
              {loading ? 'Refreshing…' : 'Refresh'}
            </Button>
          }
        />

        <GlassCard className="mb-4">
          <div className="flex flex-wrap items-end gap-3">
            <FilterField label="Status" className="min-w-[140px]">
              <Select
                value={statusFilter}
                onChange={(e) =>
                  setStatusFilter(e.target.value as 'not_completed' | 'completed' | 'all')
                }
              >
                <option value="not_completed">In progress</option>
                <option value="completed">Complete</option>
                <option value="all">All</option>
              </Select>
            </FilterField>
            <FilterField label="Batch search" className="min-w-[180px]">
              <input
                className="w-full rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-1)] px-2 py-1.5 text-sm"
                placeholder="Batch ID"
                value={batchQuery}
                onChange={(e) => setBatchQuery(e.target.value)}
              />
            </FilterField>
            <DateTimeRangeField
              from={timeFrom}
              to={timeTo}
              onFromChange={setTimeFrom}
              onToChange={setTimeTo}
              onClear={() => {
                setTimeFrom('')
                setTimeTo('')
              }}
            />
          </div>
        </GlassCard>

        {loading ? (
          <div className="py-12 text-center text-sm text-[var(--text-muted)]">Loading…</div>
        ) : (
          <DataTable
            columns={columns}
            rows={rows}
            rowKey={(row) => row.event_id}
            emptyMessage="No batches matched the current filters."
            sortKey="sent_utc"
            sortDir={timeSort}
            onSort={() => setTimeSort((c) => (c === 'desc' ? 'asc' : 'desc'))}
          />
        )}

        <div className="mt-4 flex items-center justify-between text-sm">
          <span className="text-[var(--text-muted)]">
            Page {safePage} of {pageCount} · {total} batches
          </span>
          <div className="flex gap-2">
            <Button variant="outline" onClick={() => setPage((p) => Math.max(1, p - 1))} disabled={safePage <= 1}>
              Previous
            </Button>
            <Button variant="outline" onClick={() => setPage((p) => Math.min(pageCount, p + 1))} disabled={safePage >= pageCount}>
              Next
            </Button>
          </div>
        </div>
      </div>
    </SidebarLayout>
  )
}
