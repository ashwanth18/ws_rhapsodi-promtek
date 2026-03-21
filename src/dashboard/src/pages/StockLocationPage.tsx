import { useEffect, useState } from 'react'

import DateTimeText from '../components/DateTimeText'
import GlassCard from '../components/GlassCard'
import Button from '../components/ui/button'
import { useRuntimeConfig } from '../config/RuntimeConfig'
import SidebarLayout from './SidebarLayout'

type Row = {
  id: number
  event_id: string
  context_id: string | null
  site_id: string | null
  created_utc: string | null
  stock_item_location_id: number | null
  stock_item_location_code: string | null
  stock_item_id: string | null
  stock_item_code: string | null
  stock_item_name: string | null
  created_at: string | null
}

export default function StockLocationPage() {
  const { apiBase } = useRuntimeConfig()
  const [rows, setRows] = useState<Row[]>([])
  const [loading, setLoading] = useState(false)
  const [manualRefreshing, setManualRefreshing] = useState(false)

  const loadRows = async ({ manual = false }: { manual?: boolean } = {}) => {
    try {
      setLoading(true)
      if (manual) {
        setManualRefreshing(true)
      }
      const res = await fetch(`${apiBase}/stock_location_allocations?limit=200`)
      const json = await res.json()
      setRows(json.rows || [])
    } finally {
      setLoading(false)
      if (manual) {
        setManualRefreshing(false)
      }
    }
  }

  useEffect(() => {
    loadRows()
  }, [apiBase])

  return (
    <SidebarLayout>
      <div className="px-6 py-6">
        <div className="mb-4 flex items-end justify-between">
          <div>
            <h1 className="text-2xl font-bold" style={{ fontFamily: 'Space Grotesk' }}>
              Location Allocations
            </h1>
            <p className="text-[var(--text-secondary)]">
              Stored location allocation events from the MES integration
            </p>
          </div>
          <Button onClick={() => loadRows({ manual: true })} disabled={loading}>
            {manualRefreshing ? 'Refreshing…' : 'Refresh'}
          </Button>
        </div>

        <GlassCard>
          <div className="mb-4 flex items-end justify-between gap-3">
            <div className="text-sm text-[var(--text-secondary)]">
              Most recent stored location allocation events
            </div>
            <div className="shrink-0 rounded-xl border border-[var(--border)] bg-[var(--surface)]/70 px-3 py-2 text-xs text-[var(--text-muted)]">
              {rows.length === 0
                ? 'Showing 0 of 0 rows'
                : `Showing ${rows.length} most recent rows`}
            </div>
          </div>
          <div className="overflow-auto">
            <table className="w-full text-sm">
              <thead className="text-left text-[var(--text-secondary)]">
                <tr>
                  <th>ID</th>
                  <th>Event ID</th>
                  <th>Created UTC</th>
                  <th>Site ID</th>
                  <th>Location ID</th>
                  <th>Location Code</th>
                  <th>Stock Item ID</th>
                  <th>Stock Item Code</th>
                  <th>Ingredient Name</th>
                  <th>Stored At</th>
                </tr>
              </thead>
              <tbody>
                {loading ? (
                  <tr>
                    <td className="py-6 text-center text-[var(--text-secondary)]" colSpan={10}>
                      Loading…
                    </td>
                  </tr>
                ) : rows.length === 0 ? (
                  <tr>
                    <td className="py-6 text-center text-[var(--text-secondary)]" colSpan={10}>
                      No stock location events found.
                    </td>
                  </tr>
                ) : (
                  rows.map((row) => (
                    <tr key={row.id} className="border-t border-[var(--border)]">
                      <td className="py-3 pr-4">{row.id}</td>
                      <td className="py-3 pr-4">{row.event_id}</td>
                      <td className="py-3 pr-4">
                        <DateTimeText value={row.created_utc} />
                      </td>
                      <td className="py-3 pr-4">{row.site_id ?? '—'}</td>
                      <td className="py-3 pr-4">{row.stock_item_location_id ?? '—'}</td>
                      <td className="py-3 pr-4">{row.stock_item_location_code ?? '—'}</td>
                      <td className="py-3 pr-4">{row.stock_item_id ?? '—'}</td>
                      <td className="py-3 pr-4">{row.stock_item_code ?? '—'}</td>
                      <td className="py-3 pr-4">{row.stock_item_name ?? '—'}</td>
                      <td className="py-3">
                        <DateTimeText value={row.created_at} />
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
