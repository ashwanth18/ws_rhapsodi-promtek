import { useEffect, useState } from 'react'
import SidebarLayout from './SidebarLayout'
import GlassCard from '../components/GlassCard'
import Button from '../components/ui/button'

type Row = {
  id: number
  batch_id: string
  start_time: string
  end_time: string | null
  target_weight: number
  actual_weight: number | null
  success: number
  cycle_time: number | null
  error_message: string | null
}

const API_BASE = (import.meta as any).env.VITE_API_BASE || 'http://localhost:8000'

export default function LogsPage() {
  const [rows, setRows] = useState<Row[]>([])
  const [loading, setLoading] = useState(false)
  const [demo, setDemo] = useState<boolean>(false)

  // helpers
  const buildDemoRows = (): Row[] => {
    const now = Date.now()
    return Array.from({ length: 25 }).map((_, i) => {
      const start = new Date(now - (i + 1) * 60_000)
      const target = 1 + Math.random() * 0.5
      const actual = target + (Math.random() - 0.5) * 0.08
      const success = Math.abs(actual - target) / target < 0.05 ? 1 : 0
      return {
        id: i + 1,
        batch_id: `demo-${(i + 1).toString().padStart(3, '0')}`,
        start_time: start.toISOString(),
        end_time: new Date(start.getTime() + 5000 + Math.random() * 5000).toISOString(),
        target_weight: +target.toFixed(3),
        actual_weight: +actual.toFixed(3),
        success,
        cycle_time: +(5 + Math.random() * 5).toFixed(2),
        error_message: null,
      }
    })
  }

  const loadRows = async (useDemo: boolean) => {
    try {
      setLoading(true)
      if (useDemo) {
        setRows(buildDemoRows())
      } else {
        const res = await fetch(`${API_BASE}/historical_data?days=30`)
        const json = await res.json()
        setRows(json.raw_data || [])
      }
    } finally {
      setLoading(false)
    }
  }

  // init from env/localStorage
  useEffect(() => {
    const envDemo = (import.meta as any).env.VITE_DEMO_MODE === '1'
    const stored = window.localStorage.getItem('demoMode') === '1'
    const initial = envDemo || stored
    setDemo(initial)
    loadRows(initial)
  }, [])

  // on demo toggle
  useEffect(() => {
    window.localStorage.setItem('demoMode', demo ? '1' : '0')
    // If user toggled from the page, reload data accordingly
    loadRows(demo)
  }, [demo])

  return (
    <SidebarLayout>
      <div className="px-6 py-6">
        <div className="flex items-end justify-between mb-4">
          <div>
            <h1 className="text-2xl font-bold" style={{ fontFamily: 'Space Grotesk' }}>Historical Logs</h1>
            <p className="text-white/70">Batch results with target/actual weight, success, and cycle time</p>
          </div>
          <div className="flex items-center gap-2">
            <span className={`text-xs ${demo ? 'text-emerald-400' : 'text-white/60'}`}>{demo ? 'Demo Data' : 'Live Data'}</span>
            <Button onClick={() => setDemo(true)} className="text-black">Populate Demo</Button>
            <Button variant="ghost" onClick={() => setDemo(false)}>Use Live</Button>
          </div>
        </div>
        <GlassCard>
          <div className="overflow-auto">
            <table className="w-full text-sm">
              <thead className="text-left text-white/70">
                <tr>
                  <th>Batch</th>
                  <th>Start</th>
                  <th>End</th>
                  <th className="text-right">Target (kg)</th>
                  <th className="text-right">Actual (kg)</th>
                  <th className="text-right">Deviation (kg)</th>
                  <th>Status</th>
                  <th className="text-right">Cycle (s)</th>
                </tr>
              </thead>
              <tbody>
                {loading ? (
                  <tr><td className="py-4" colSpan={8}>Loading…</td></tr>
                ) : rows.length === 0 ? (
                  <tr><td className="py-4" colSpan={8}>No data</td></tr>
                ) : (
                  rows.map((r) => {
                    const dev = r.actual_weight != null ? Math.abs(r.actual_weight - r.target_weight) : null
                    const ok = r.success === 1
                    return (
                      <tr key={r.id} className="border-t border-slate-800">
                        <td>{r.batch_id}</td>
                        <td>{r.start_time}</td>
                        <td>{r.end_time ?? '—'}</td>
                        <td className="text-right">{r.target_weight.toFixed(3)}</td>
                        <td className="text-right">{r.actual_weight != null ? r.actual_weight.toFixed(3) : '—'}</td>
                        <td className="text-right">{dev != null ? dev.toFixed(3) : '—'}</td>
                        <td>
                          <span className={`text-xs px-2 py-0.5 rounded-md ${ok ? 'bg-emerald-400/15 text-emerald-400' : 'bg-rose-400/15 text-rose-400'}`}>
                            {ok ? 'Success' : 'Failed'}
                          </span>
                        </td>
                        <td className="text-right">{r.cycle_time != null ? r.cycle_time.toFixed(2) : '—'}</td>
                      </tr>
                    )
                  })
                )}
              </tbody>
            </table>
          </div>
        </GlassCard>
      </div>
    </SidebarLayout>
  )
}


