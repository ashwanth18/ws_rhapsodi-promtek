import { useEffect, useMemo, useState } from 'react'
import SidebarLayout from './SidebarLayout'
import GlassCard from '../components/GlassCard'
import Button from '../components/ui/button'
import {
  Bar,
  BarChart,
  CartesianGrid,
  Legend,
  ReferenceLine,
  ResponsiveContainer,
  Scatter,
  ScatterChart,
  Tooltip,
  XAxis,
  YAxis,
} from 'recharts'
import { Card, CardContent, CardHeader, CardTitle } from '../components/ui/card'

type Row = {
  id: number
  batch_id: string | null
  episode_index: number | null
  start_time_ns: number | null
  end_time_ns: number | null
  target_weight_g: number | null
  final_weight_g: number | null
  net_weight_g: number | null
  avg_flow_rate_g_s: number | null
  total_episode_time_s: number | null
  overshoot_g: number | null
}

const API_BASE = (import.meta as any).env.VITE_API_BASE || 'http://localhost:8000'

export default function LogsPage() {
  const [rows, setRows] = useState<Row[]>([])
  const [loading, setLoading] = useState(false)
  const [plotBatchFilter, setPlotBatchFilter] = useState<string>('all')
  const [tableBatchFilter, setTableBatchFilter] = useState<string>('all')
  const [tableEpisodeFilter, setTableEpisodeFilter] = useState<string>('')
  const [tablePage, setTablePage] = useState<number>(1)

  // helpers
  const formatTimestamp = (ms: number | null) => {
    if (!ms || !Number.isFinite(ms)) return '—'
    const dt = new Date(ms)
    return dt.toLocaleString(undefined, {
      year: 'numeric',
      month: '2-digit',
      day: '2-digit',
      hour: '2-digit',
      minute: '2-digit',
      second: '2-digit',
    })
  }
  const loadRows = async () => {
    try {
      setLoading(true)
      const res = await fetch(`${API_BASE}/lightsout_processed?limit=100`)
      const json = await res.json()
      setRows(json.rows || [])
    } finally {
      setLoading(false)
    }
  }

  // init
  useEffect(() => {
    loadRows()
  }, [])

  const distinctBatches = Array.from(
    new Set(rows.map((row) => row.batch_id).filter((value): value is string => Boolean(value)))
  ).sort()

  const tableFilteredRows = useMemo(() => {
    const episode = tableEpisodeFilter.trim()
    return rows.filter((row) => {
      if (tableBatchFilter !== 'all' && row.batch_id !== tableBatchFilter) {
        return false
      }
      if (episode) {
        const episodeNumber = Number.parseInt(episode, 10)
        if (!Number.isFinite(episodeNumber)) {
          return false
        }
        if (row.episode_index !== episodeNumber) {
          return false
        }
      }
      return true
    })
  }, [rows, tableBatchFilter, tableEpisodeFilter])

  const tablePageSize = 20
  const tablePageCount = Math.max(1, Math.ceil(tableFilteredRows.length / tablePageSize))
  const tablePageSafe = Math.min(tablePage, tablePageCount)
  const tablePageRows = tableFilteredRows.slice(
    (tablePageSafe - 1) * tablePageSize,
    tablePageSafe * tablePageSize
  )

  const plotRows = useMemo(() => {
    if (plotBatchFilter === 'all') {
      return rows
    }
    return rows.filter((row) => row.batch_id === plotBatchFilter)
  }, [rows, plotBatchFilter])

  useEffect(() => {
    setTablePage(1)
  }, [tableBatchFilter, tableEpisodeFilter])

  const plotPointsFinalVsTarget = plotRows
    .filter((row) => row.target_weight_g != null && row.final_weight_g != null)
    .map((row) => ({
      x: row.target_weight_g as number,
      y: row.final_weight_g as number,
      label: `Episode ${row.episode_index ?? '—'}`,
    }))

  const plotPointsDurationVsFinal = plotRows
    .filter((row) => row.total_episode_time_s != null && row.final_weight_g != null)
    .map((row) => ({
      x: row.total_episode_time_s as number,
      y: row.final_weight_g as number,
      label: `Episode ${row.episode_index ?? '—'}`,
    }))

  const overshootValues = plotRows
    .map((row) => row.overshoot_g)
    .filter((value): value is number => value != null && Number.isFinite(value))
    .sort((a, b) => a - b)

  const overshootMean = overshootValues.length
    ? overshootValues.reduce((sum, value) => sum + value, 0) / overshootValues.length
    : null

  const overshootMedian = overshootValues.length
    ? (() => {
        const mid = Math.floor(overshootValues.length / 2)
        return overshootValues.length % 2 === 0
          ? (overshootValues[mid - 1] + overshootValues[mid]) / 2
          : overshootValues[mid]
      })()
    : null

  const histogramBins = 10
  const overshootHistogram = (() => {
    if (overshootValues.length === 0) {
      return { centers: [], percentages: [], min: 0, max: 0, step: 0 }
    }
    const min = Math.min(...overshootValues)
    const max = Math.max(...overshootValues)
    const range = max - min || 1
    const step = range / histogramBins
    const counts = Array.from({ length: histogramBins }, () => 0)
    overshootValues.forEach((value) => {
      const idx = Math.min(histogramBins - 1, Math.floor((value - min) / step))
      counts[idx] += 1
    })
    const percentages = counts.map((count) => (count / overshootValues.length) * 100)
    const centers = counts.map((_, idx) => min + step * (idx + 0.5))
    return { centers, percentages, min, max, step }
  })()
  const overshootMinX = overshootValues.length
    ? overshootHistogram.min - overshootHistogram.step * 0.5
    : 0
  const overshootMaxX = overshootValues.length
    ? overshootHistogram.max + overshootHistogram.step * 0.5
    : 1

  const averageOf = (items: Row[], pick: (row: Row) => number | null): number | null => {
    const values = items.map(pick).filter((value): value is number => value != null && Number.isFinite(value))
    if (values.length === 0) {
      return null
    }
    return values.reduce((sum, value) => sum + value, 0) / values.length
  }

  const totalEpisodes = rows.length
  const avgFinal = averageOf(rows, (row) => row.final_weight_g)
  const avgNet = averageOf(rows, (row) => row.net_weight_g)
  const avgFlow = averageOf(rows, (row) => row.avg_flow_rate_g_s)
  const avgDuration = averageOf(rows, (row) => row.total_episode_time_s)
  const avgOvershoot = averageOf(rows, (row) => row.overshoot_g)

  const overshootByBatch = distinctBatches.map((batch) => {
    const batchRows = tableFilteredRows.filter((row) => row.batch_id === batch)
    return {
      batch,
      avgOvershoot: averageOf(batchRows, (row) => row.overshoot_g) ?? 0,
      count: batchRows.length,
    }
  })

  const rechartsFinalVsTarget = plotPointsFinalVsTarget.map((point) => ({
    target: point.x,
    final: point.y,
    label: point.label,
  }))
  const rechartsDurationVsFinal = plotPointsDurationVsFinal.map((point) => ({
    duration: point.x,
    final: point.y,
    label: point.label,
  }))
  const rechartsHistogram = overshootHistogram.centers.map((center, idx) => ({
    overshoot: center,
    percent: +overshootHistogram.percentages[idx].toFixed(2),
  }))

  return (
    <SidebarLayout>
      <div className="px-6 py-6">
        <div className="flex items-end justify-between mb-4">
          <div>
            <h1 className="text-2xl font-bold" style={{ fontFamily: 'Space Grotesk' }}>Historical Logs</h1>
            <p className="text-white/70">Processed lights-out episodes with weights and flow metrics</p>
          </div>
          <div className="flex items-center gap-2">
            <Button onClick={() => loadRows()} disabled={loading}>
              {loading ? (
                <span className="inline-flex items-center gap-2">
                  <svg
                    className="h-4 w-4 animate-spin"
                    viewBox="0 0 24 24"
                    fill="none"
                    xmlns="http://www.w3.org/2000/svg"
                    aria-hidden="true"
                  >
                    <circle
                      className="opacity-25"
                      cx="12"
                      cy="12"
                      r="10"
                      stroke="currentColor"
                      strokeWidth="4"
                    />
                    <path
                      className="opacity-75"
                      fill="currentColor"
                      d="M4 12a8 8 0 018-8v4a4 4 0 00-4 4H4z"
                    />
                  </svg>
                  Refreshing…
                </span>
              ) : (
                'Refresh'
              )}
            </Button>
          </div>
        </div>
        <GlassCard>
          <div className="flex flex-wrap items-end justify-between gap-3 mb-3">
            <div className="flex items-center gap-2 text-sm">
              <span className="text-white/60">Batch</span>
              <select
                className="rounded-md bg-slate-900/60 border border-slate-700 px-2 py-1 text-sm"
                value={tableBatchFilter}
                onChange={(event) => setTableBatchFilter(event.target.value)}
              >
                <option value="all">All</option>
                {distinctBatches.map((batch) => (
                  <option key={batch} value={batch}>{batch}</option>
                ))}
              </select>
              <span className="text-white/60">Episode</span>
              <input
                className="w-24 rounded-md bg-slate-900/60 border border-slate-700 px-2 py-1 text-sm"
                placeholder="e.g. 3"
                value={tableEpisodeFilter}
                onChange={(event) => setTableEpisodeFilter(event.target.value)}
              />
            </div>
            <div className="text-xs text-white/60">
              Showing {tableFilteredRows.length} entries
            </div>
          </div>
          <div className="overflow-auto">
            <table className="w-full text-sm">
              <thead className="text-left text-white/70">
                <tr>
                  <th>Batch</th>
                  <th>Episode</th>
                  <th>Start</th>
                  <th>End</th>
                  <th className="text-right">Target (g)</th>
                  <th className="text-right">Final (g)</th>
                  <th className="text-right">Net (g)</th>
                  <th className="text-right">Avg Flow (g/s)</th>
                  <th className="text-right">Duration (s)</th>
                  <th className="text-right">Overshoot (g)</th>
                </tr>
              </thead>
              <tbody>
                {loading ? (
                  <tr>
                    <td className="py-6 text-center" colSpan={10}>
                      <span className="inline-flex items-center gap-2 text-white/70">
                        <svg
                          className="h-4 w-4 animate-spin"
                          viewBox="0 0 24 24"
                          fill="none"
                          xmlns="http://www.w3.org/2000/svg"
                          aria-hidden="true"
                        >
                          <circle
                            className="opacity-25"
                            cx="12"
                            cy="12"
                            r="10"
                            stroke="currentColor"
                            strokeWidth="4"
                          />
                          <path
                            className="opacity-75"
                            fill="currentColor"
                            d="M4 12a8 8 0 018-8v4a4 4 0 00-4 4H4z"
                          />
                        </svg>
                        Loading…
                      </span>
                    </td>
                  </tr>
                ) : tableFilteredRows.length === 0 ? (
                  <tr><td className="py-4" colSpan={10}>No data</td></tr>
                ) : (
                  tablePageRows.map((r) => {
                    const startMs = r.start_time_ns ? r.start_time_ns / 1_000_000 : null
                    const endMs = r.end_time_ns ? r.end_time_ns / 1_000_000 : null
                    const start = formatTimestamp(startMs)
                    const end = formatTimestamp(endMs)
                    return (
                      <tr key={r.id} className="border-t border-slate-800">
                        <td>{r.batch_id ?? '—'}</td>
                        <td>{r.episode_index ?? '—'}</td>
                        <td>{start}</td>
                        <td>{end}</td>
                        <td className="text-right">{r.target_weight_g != null ? r.target_weight_g.toFixed(2) : '—'}</td>
                        <td className="text-right">{r.final_weight_g != null ? r.final_weight_g.toFixed(2) : '—'}</td>
                        <td className="text-right">{r.net_weight_g != null ? r.net_weight_g.toFixed(2) : '—'}</td>
                        <td className="text-right">{r.avg_flow_rate_g_s != null ? r.avg_flow_rate_g_s.toFixed(2) : '—'}</td>
                        <td className="text-right">{r.total_episode_time_s != null ? r.total_episode_time_s.toFixed(2) : '—'}</td>
                        <td className="text-right">{r.overshoot_g != null ? r.overshoot_g.toFixed(2) : '—'}</td>
                      </tr>
                    )
                  })
                )}
              </tbody>
            </table>
          </div>
          <div className="mt-3 flex items-center justify-between text-sm">
            <div className="text-white/60">
              Page {tablePageSafe} of {tablePageCount}
            </div>
            <div className="flex items-center gap-2">
              <Button
                variant="ghost"
                onClick={() => setTablePage((p) => Math.max(1, p - 1))}
                disabled={tablePageSafe <= 1}
              >
                Previous
              </Button>
              <Button
                variant="ghost"
                onClick={() => setTablePage((p) => Math.min(tablePageCount, p + 1))}
                disabled={tablePageSafe >= tablePageCount}
              >
                Next
              </Button>
            </div>
          </div>
        </GlassCard>

        <div className="mt-6 grid grid-cols-1 lg:grid-cols-3 gap-4">
          <GlassCard>
            <div className="text-xs text-white/60 mb-2">Episode Summary</div>
            <div className="grid grid-cols-2 gap-3 text-sm">
              <div>Total Episodes</div>
              <div className="text-right">{totalEpisodes}</div>
              <div>Avg Final (g)</div>
              <div className="text-right">{avgFinal != null ? avgFinal.toFixed(2) : '—'}</div>
              <div>Avg Net (g)</div>
              <div className="text-right">{avgNet != null ? avgNet.toFixed(2) : '—'}</div>
              <div>Avg Flow (g/s)</div>
              <div className="text-right">{avgFlow != null ? avgFlow.toFixed(2) : '—'}</div>
              <div>Avg Duration (s)</div>
              <div className="text-right">{avgDuration != null ? avgDuration.toFixed(2) : '—'}</div>
              <div>Avg Overshoot (g)</div>
              <div className="text-right">{avgOvershoot != null ? avgOvershoot.toFixed(2) : '—'}</div>
            </div>
          </GlassCard>
          <GlassCard>
            <div className="text-xs text-white/60 mb-2">Avg Overshoot by Batch</div>
            {overshootByBatch.length === 0 ? (
              <div className="text-sm text-white/60">No batch data.</div>
            ) : (
              <div className="space-y-2">
                {overshootByBatch.map((item) => {
                  const maxValue = Math.max(1, ...overshootByBatch.map((x) => Math.abs(x.avgOvershoot)))
                  const width = Math.min(100, Math.abs(item.avgOvershoot) / maxValue * 100)
                  return (
                    <div key={item.batch} className="text-xs">
                      <div className="flex justify-between mb-1">
                        <span className="text-white/80">{item.batch} ({item.count})</span>
                        <span className="text-white/60">{item.avgOvershoot.toFixed(2)} g</span>
                      </div>
                      <div className="h-2 rounded-full bg-slate-800">
                        <div
                          className="h-2 rounded-full bg-emerald-400"
                          style={{ width: `${width}%` }}
                        />
                      </div>
                    </div>
                  )
                })}
              </div>
            )}
          </GlassCard>
        </div>

        <div className="mt-8">
          <div className="flex items-center justify-between mb-3">
            <h2 className="text-lg font-semibold">Plots (shadcn/ui)</h2>
            <div className="flex items-center gap-2 text-sm">
              <span className="text-white/60">Batch</span>
              <select
                className="rounded-md bg-slate-900/60 border border-slate-700 px-2 py-1 text-sm"
                value={plotBatchFilter}
                onChange={(event) => setPlotBatchFilter(event.target.value)}
              >
                <option value="all">All</option>
                {distinctBatches.map((batch) => (
                  <option key={batch} value={batch}>{batch}</option>
                ))}
              </select>
            </div>
          </div>
          <div className="grid grid-cols-1 lg:grid-cols-2 gap-4">
            <Card>
              <CardHeader>
                <CardTitle>Final Weight vs Target</CardTitle>
              </CardHeader>
              <CardContent>
                <div style={{ width: '100%', height: 240 }}>
                  <ResponsiveContainer>
                    <ScatterChart>
                      <CartesianGrid strokeDasharray="3 3" stroke="#1f2937" />
                      <XAxis
                        dataKey="target"
                        type="number"
                        name="Target"
                        unit="g"
                        tick={{ fill: '#cbd5f5', fontSize: 11 }}
                      />
                      <YAxis
                        dataKey="final"
                        type="number"
                        name="Final"
                        unit="g"
                        tick={{ fill: '#cbd5f5', fontSize: 11 }}
                      />
                      <Tooltip cursor={{ strokeDasharray: '3 3' }} />
                      <Legend />
                      <Scatter
                        name={`Episodes (${rechartsFinalVsTarget.length})`}
                        data={rechartsFinalVsTarget}
                        fill="#38bdf8"
                      />
                    </ScatterChart>
                  </ResponsiveContainer>
                </div>
              </CardContent>
            </Card>
            <Card>
              <CardHeader>
                <CardTitle>Duration vs Final Weight</CardTitle>
              </CardHeader>
              <CardContent>
                <div style={{ width: '100%', height: 240 }}>
                  <ResponsiveContainer>
                    <ScatterChart>
                      <CartesianGrid strokeDasharray="3 3" stroke="#1f2937" />
                      <XAxis
                        dataKey="duration"
                        type="number"
                        name="Duration"
                        unit="s"
                        tick={{ fill: '#cbd5f5', fontSize: 11 }}
                      />
                      <YAxis
                        dataKey="final"
                        type="number"
                        name="Final"
                        unit="g"
                        tick={{ fill: '#cbd5f5', fontSize: 11 }}
                      />
                      <Tooltip cursor={{ strokeDasharray: '3 3' }} />
                      <Legend />
                      <Scatter
                        name={`Episodes (${rechartsDurationVsFinal.length})`}
                        data={rechartsDurationVsFinal}
                        fill="#fbbf24"
                      />
                    </ScatterChart>
                  </ResponsiveContainer>
                </div>
              </CardContent>
            </Card>
          </div>
          <div className="mt-4">
            <Card>
              <CardHeader>
                <CardTitle>Overshoot Distribution (% of Episodes)</CardTitle>
              </CardHeader>
              <CardContent>
                <div style={{ width: '100%', height: 240 }}>
                  <ResponsiveContainer>
                    <BarChart data={rechartsHistogram}>
                      <CartesianGrid strokeDasharray="3 3" stroke="#1f2937" />
                      <XAxis
                        dataKey="overshoot"
                        type="number"
                        domain={[overshootMinX, overshootMaxX]}
                        tick={{ fill: '#cbd5f5', fontSize: 11 }}
                      />
                      <YAxis tick={{ fill: '#cbd5f5', fontSize: 11 }} />
                      <Tooltip />
                      <Legend />
                      <Bar
                        dataKey="percent"
                        name={`Episodes (${overshootValues.length})`}
                        fill="#34d399"
                      />
                      {overshootMean != null && (
                        <ReferenceLine
                          x={overshootMean}
                          stroke="#60a5fa"
                          strokeDasharray="4 3"
                          label={`Mean (${overshootMean.toFixed(2)}g)`}
                        />
                      )}
                      {overshootMedian != null && (
                        <ReferenceLine
                          x={overshootMedian}
                          stroke="#fbbf24"
                          strokeDasharray="4 3"
                          label={`Median (${overshootMedian.toFixed(2)}g)`}
                        />
                      )}
                    </BarChart>
                  </ResponsiveContainer>
                </div>
              </CardContent>
            </Card>
          </div>
        </div>
      </div>
    </SidebarLayout>
  )
}


