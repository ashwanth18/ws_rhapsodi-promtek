import { useEffect, useState } from 'react'
import DateTimeText from '../components/DateTimeText'
import Button from '../components/ui/button'
import DataTable, { Column } from '../components/ui/DataTable'
import { SectionHeader } from '../components/ui/SectionHeader'
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

  const loadRows = async () => {
    try {
      setLoading(true)
      const res = await fetch(`${apiBase}/stock_location_allocations?limit=200`)
      const json = await res.json()
      setRows(json.rows || [])
    } finally {
      setLoading(false)
    }
  }

  useEffect(() => {
    void loadRows()
  }, [apiBase])

  const columns: Column<Row>[] = [
    { key: 'id', header: 'ID', render: (row) => row.id },
    { key: 'event_id', header: 'Event', render: (row) => row.event_id },
    { key: 'created_utc', header: 'Created', render: (row) => <DateTimeText value={row.created_utc} /> },
    { key: 'location', header: 'Location', render: (row) => row.stock_item_location_code ?? row.stock_item_location_id ?? '—' },
    { key: 'stock_item_id', header: 'Ingredient', render: (row) => row.stock_item_name ?? row.stock_item_id ?? '—' },
    { key: 'site_id', header: 'Site', render: (row) => row.site_id ?? '—' },
  ]

  return (
    <SidebarLayout>
      <div className="px-5 py-5 lg:px-6">
        <SectionHeader
          title="Location Allocations"
          description="MES stock location allocation events used for robot target mapping."
          action={
            <Button onClick={() => loadRows()} disabled={loading}>
              {loading ? 'Refreshing…' : 'Refresh'}
            </Button>
          }
        />
        {loading ? (
          <div className="py-12 text-center text-sm text-[var(--text-muted)]">Loading…</div>
        ) : (
          <DataTable
            columns={columns}
            rows={rows}
            rowKey={(row) => row.id}
            emptyMessage="No stock location events found."
          />
        )}
      </div>
    </SidebarLayout>
  )
}
