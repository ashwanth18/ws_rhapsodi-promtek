import { useEffect, useMemo, useState } from 'react'
import { Link } from 'react-router-dom'

import DateTimeText from '../components/DateTimeText'
import GlassCard from '../components/GlassCard'
import Button from '../components/ui/button'
import { useRuntimeConfig } from '../config/RuntimeConfig'
import SidebarLayout from './SidebarLayout'

type Row = {
  event_id: string
  sent_utc: string | null
  batch_id: string | null
  completed: boolean
  weightment_count: number
}

export default function WebhookWeightmentsPage() {
  const { apiBase } = useRuntimeConfig()
  const [rows, setRows] = useState<Row[]>([])
  const [loading, setLoading] = useState(false)
  const [statusFilter, setStatusFilter] = useState<'not_completed' | 'completed' | 'all'>(
    'not_completed'
  )
  const [eventFilter, setEventFilter] = useState('')
  const [batchFilter, setBatchFilter] = useState('')

  const loadRows = async () => {
    try {
      setLoading(true)
      const res = await fetch(`${apiBase}/webhook_weightments/summary?limit=200`)
      const json = await res.json()
      setRows(json.rows || [])
    } finally {
      setLoading(false)
    }
  }

  useEffect(() => {
    loadRows()
  }, [apiBase])

  const filteredRows = useMemo(() => {
    const eventQuery = eventFilter.trim().toLowerCase()
    const batchQuery = batchFilter.trim().toLowerCase()
    return rows.filter((row) => {
      if (statusFilter === 'not_completed' && row.completed) {
        return false
      }
      if (statusFilter === 'completed' && !row.completed) {
        return false
      }
      if (eventQuery && !row.event_id.toLowerCase().includes(eventQuery)) {
        return false
      }
      const batchId = (row.batch_id || '').toLowerCase()
      if (batchQuery && !batchId.includes(batchQuery)) {
        return false
      }
      return true
    })
  }, [rows, statusFilter, eventFilter, batchFilter])

  return (
    <SidebarLayout>
      <div className="px-6 py-6">
        <div className="mb-4 flex items-end justify-between">
          <div>
            <h1 className="text-2xl font-bold" style={{ fontFamily: 'Space Grotesk' }}>
              Webhook Batches
            </h1>
            <p className="text-white/70">
              One row per released batch, grouped from the stored weightments
            </p>
          </div>
          <Button onClick={() => loadRows()} disabled={loading}>
            {loading ? 'Refreshing…' : 'Refresh'}
          </Button>
        </div>

        <GlassCard>
          <div className="mb-3 flex flex-wrap items-end justify-between gap-3">
            <div className="flex flex-wrap items-center gap-2 text-sm">
              <span className="text-white/60">Status</span>
              <select
                className="rounded-md border border-slate-700 bg-slate-900/60 px-2 py-1 text-sm"
                value={statusFilter}
                onChange={(event) =>
                  setStatusFilter(event.target.value as 'not_completed' | 'completed' | 'all')
                }
              >
                <option value="not_completed">Not Complete</option>
                <option value="completed">Complete</option>
                <option value="all">All</option>
              </select>
              <span className="text-white/60">Batch</span>
              <input
                className="w-56 rounded-md border border-slate-700 bg-slate-900/60 px-2 py-1 text-sm"
                placeholder="Filter by batch id"
                value={batchFilter || eventFilter}
                onChange={(event) => {
                  setBatchFilter(event.target.value)
                  setEventFilter(event.target.value)
                }}
              />
            </div>
            <div className="text-xs text-white/60">
              Showing {filteredRows.length} of {rows.length} batches
            </div>
          </div>
          <div className="overflow-auto">
            <table className="w-full text-sm">
              <thead className="text-left text-white/70">
                <tr>
                  <th>Batch ID</th>
                  <th>Released Time</th>
                  <th>Completed</th>
                  <th className="text-right">Weightments</th>
                  <th />
                </tr>
              </thead>
              <tbody>
                {loading ? (
                  <tr>
                    <td className="py-6 text-center text-white/70" colSpan={5}>
                      Loading…
                    </td>
                  </tr>
                ) : filteredRows.length === 0 ? (
                  <tr>
                    <td className="py-6 text-center text-white/70" colSpan={5}>
                      No webhook batches matched the current filters.
                    </td>
                  </tr>
                ) : (
                  filteredRows.map((row) => (
                    <tr key={row.event_id} className="border-t border-slate-800">
                      <td className="py-3 pr-4">{row.batch_id ?? '—'}</td>
                      <td className="py-3 pr-4">
                        <DateTimeText value={row.sent_utc} />
                      </td>
                      <td className="py-3 pr-4">
                        <span
                          className={`inline-flex rounded-full px-2 py-1 text-xs font-semibold ${
                            row.completed
                              ? 'bg-emerald-500/20 text-emerald-300'
                              : 'bg-rose-500/20 text-rose-300'
                          }`}
                        >
                          {row.completed ? 'Complete' : 'Not Complete'}
                        </span>
                      </td>
                      <td className="py-3 pr-4 text-right">{row.weightment_count}</td>
                      <td className="py-3 text-right">
                        <Link
                          to={`/webhook-weightments/${encodeURIComponent(row.event_id)}`}
                          className="inline-flex items-center justify-center rounded-md bg-[#38bdf8] px-3 py-1.5 text-sm font-semibold text-black transition-colors hover:bg-[#60a5fa]"
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
        </GlassCard>
      </div>
    </SidebarLayout>
  )
}
