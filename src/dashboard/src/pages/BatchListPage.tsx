import { type ReactNode, useEffect, useState } from 'react'
import { Link } from 'react-router-dom'

import DateTimeRangeField from '../components/DateTimeRangeField'
import DateTimeText from '../components/DateTimeText'
import GlassCard from '../components/GlassCard'
import Button from '../components/ui/button'
import Select from '../components/ui/select'
import { useRuntimeConfig } from '../config/RuntimeConfig'
import SidebarLayout from './SidebarLayout'

type Row = {
  event_id: string
  sent_utc: string | null
  batch_id: string | null
  completed: boolean
  weightment_count: number
}

type BatchSummaryResponse = {
  rows?: Row[]
  total?: number
}

function TimeSortIcon({ order }: { order: 'asc' | 'desc' }) {
  return (
    <svg
      className="h-4 w-4"
      viewBox="0 0 24 24"
      fill="none"
      xmlns="http://www.w3.org/2000/svg"
      aria-hidden="true"
    >
      <path
        d="M8 7L12 3L16 7"
        stroke="currentColor"
        strokeWidth="2"
        strokeLinecap="round"
        strokeLinejoin="round"
        className={order === 'asc' ? 'opacity-100' : 'opacity-35'}
      />
      <path
        d="M16 17L12 21L8 17"
        stroke="currentColor"
        strokeWidth="2"
        strokeLinecap="round"
        strokeLinejoin="round"
        className={order === 'desc' ? 'opacity-100' : 'opacity-35'}
      />
    </svg>
  )
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
      className={`flex shrink-0 flex-col gap-1 rounded-xl border border-[var(--border)] bg-[var(--surface)]/70 px-3 py-2 ${className}`}
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
  const [manualRefreshing, setManualRefreshing] = useState(false)
  const [total, setTotal] = useState(0)
  const [statusFilter, setStatusFilter] = useState<'not_completed' | 'completed' | 'all'>(
    'not_completed'
  )
  const [batchQuery, setBatchQuery] = useState('')
  const [timeFrom, setTimeFrom] = useState('')
  const [timeTo, setTimeTo] = useState('')
  const [timeSort, setTimeSort] = useState<'asc' | 'desc'>('desc')
  const [page, setPage] = useState(1)
  const pageSize = 5

  const loadRows = async ({ manual = false }: { manual?: boolean } = {}) => {
    try {
      setLoading(true)
      if (manual) {
        setManualRefreshing(true)
      }
      const search = new URLSearchParams()
      search.set('limit', String(pageSize))
      search.set('offset', String((page - 1) * pageSize))
      search.set('status', statusFilter)
      search.set('time_sort', timeSort)
      if (batchQuery.trim()) {
        search.set('batch_query', batchQuery.trim())
      }
      const timeFromIso = toFilterIsoStart(timeFrom)
      const timeToIso = toFilterIsoEnd(timeTo)
      if (timeFromIso) {
        search.set('time_from', timeFromIso)
      }
      if (timeToIso) {
        search.set('time_to', timeToIso)
      }
      const res = await fetch(`${apiBase}/webhook_weightments/summary?${search.toString()}`)
      const json = (await res.json()) as BatchSummaryResponse
      setRows(json.rows || [])
      setTotal(json.total ?? 0)
    } finally {
      setLoading(false)
      if (manual) {
        setManualRefreshing(false)
      }
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
  const visibleStart = total === 0 ? 0 : (safePage - 1) * pageSize + 1
  const visibleEnd = total === 0 ? 0 : Math.min(visibleStart + rows.length - 1, total)

  return (
    <SidebarLayout>
      <div className="px-6 py-6">
        <div className="mb-4 flex items-end justify-between">
          <div>
            <h1 className="text-2xl font-bold" style={{ fontFamily: 'Space Grotesk' }}>
              Batches
            </h1>
            <p className="text-[var(--text-secondary)]">
              One row per released batch, grouped from the stored weightments
            </p>
          </div>
          <Button onClick={() => loadRows({ manual: true })} disabled={loading}>
            {manualRefreshing ? 'Refreshing…' : 'Refresh'}
          </Button>
        </div>

        <GlassCard>
          <div className="mb-4 flex items-end justify-between gap-3">
            <div className="flex flex-1 items-end gap-3 overflow-x-auto pb-1">
              <FilterField label="Status" className="min-w-[140px]">
                <Select
                  value={statusFilter}
                  onChange={(event) =>
                    setStatusFilter(event.target.value as 'not_completed' | 'completed' | 'all')
                  }
                >
                  <option value="not_completed">Not Complete</option>
                  <option value="completed">Complete</option>
                  <option value="all">All</option>
                </Select>
              </FilterField>
              <FilterField label="Batch" className="min-w-[180px]">
                <input
                  className="w-56 rounded-md border border-[var(--border)] bg-[var(--surface)] px-2 py-1 text-sm text-[var(--text-primary)]"
                  placeholder="Filter by batch id"
                  value={batchQuery}
                  onChange={(event) => setBatchQuery(event.target.value)}
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
            <div className="shrink-0 rounded-xl border border-[var(--border)] bg-[var(--surface)]/70 px-3 py-2 text-xs text-[var(--text-muted)]">
              {total === 0
                ? 'Showing 0 of 0 batches'
                : `Showing ${visibleStart}-${visibleEnd} of ${total} batches`}
            </div>
          </div>
          <div className="overflow-auto">
            <table className="w-full text-sm">
              <thead className="text-left text-[var(--text-secondary)]">
                <tr>
                  <th>Batch ID</th>
                  <th>
                    <button
                      type="button"
                      className="inline-flex items-center gap-1 font-medium hover:text-[var(--text-primary)]"
                      onClick={() => setTimeSort((current) => (current === 'desc' ? 'asc' : 'desc'))}
                      aria-label={`Sort released time ${timeSort === 'desc' ? 'ascending' : 'descending'}`}
                    >
                      <span>Released Time</span>
                      <TimeSortIcon order={timeSort} />
                    </button>
                  </th>
                  <th>Status</th>
                  <th className="text-right">Total Weightments</th>
                  <th />
                </tr>
              </thead>
              <tbody>
                {loading ? (
                  <tr>
                    <td className="py-6 text-center text-[var(--text-secondary)]" colSpan={5}>
                      Loading…
                    </td>
                  </tr>
                ) : rows.length === 0 ? (
                  <tr>
                    <td className="py-6 text-center text-[var(--text-secondary)]" colSpan={5}>
                      No batches matched the current filters.
                    </td>
                  </tr>
                ) : (
                  rows.map((row) => (
                    <tr key={row.event_id} className="border-t border-[var(--border)]">
                      <td className="py-3 pr-4">{row.batch_id ?? '—'}</td>
                      <td className="py-3 pr-4">
                        <DateTimeText value={row.sent_utc} />
                      </td>
                      <td className="py-3 pr-4">
                        <span
                          className={`inline-flex rounded-full px-2 py-1 text-xs font-semibold ${
                            row.completed
                              ? 'bg-[var(--status-good-bg)] text-[var(--status-good-fg)]'
                              : 'bg-[var(--status-bad-bg)] text-[var(--status-bad-fg)]'
                          }`}
                        >
                          {row.completed ? 'Complete' : 'Not Complete'}
                        </span>
                      </td>
                      <td className="py-3 pr-4 text-right">{row.weightment_count}</td>
                      <td className="py-3 text-right">
                        <Link
                          to={`/batches/${encodeURIComponent(row.event_id)}`}
                          className="inline-flex items-center justify-center rounded-md bg-[var(--button-primary-bg)] px-3 py-1.5 text-sm font-semibold text-[var(--button-primary-text)] transition-colors hover:bg-[var(--button-primary-hover)]"
                        >
                          Open
                        </Link>
                      </td>
                    </tr>
                  ))
                )}
              </tbody>
            </table>
          </div>
          <div className="mt-3 flex items-center justify-between text-sm">
            <div className="text-[var(--text-muted)]">
              Page {safePage} of {pageCount}
            </div>
            <div className="flex items-center gap-2">
              <Button
                variant="ghost"
                onClick={() => setPage((current) => Math.max(1, current - 1))}
                disabled={safePage <= 1}
              >
                Previous
              </Button>
              <Button
                variant="ghost"
                onClick={() => setPage((current) => Math.min(pageCount, current + 1))}
                disabled={safePage >= pageCount}
              >
                Next
              </Button>
            </div>
          </div>
        </GlassCard>
      </div>
    </SidebarLayout>
  )
}
