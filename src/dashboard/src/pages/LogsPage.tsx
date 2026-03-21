import { type ReactNode, useEffect, useState } from 'react'
import { useNavigate } from 'react-router-dom'
import SidebarLayout from './SidebarLayout'
import DateTimeText from '../components/DateTimeText'
import DateTimeRangeField from '../components/DateTimeRangeField'
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
import Select from '../components/ui/select'
import { useRuntimeConfig } from '../config/RuntimeConfig'

type Row = {
  id: number
  event_id: string | null
  weightment_id: number | null
  mode: string | null
  run_id: string | null
  batch_id: string | null
  ingredient_id: string | null
  episode_index: number | null
  start_time_ns: number | null
  end_time_ns: number | null
  target_weight_g: number | null
  final_weight_g: number | null
  net_weight_g: number | null
  avg_flow_rate_g_s: number | null
  total_episode_time_s: number | null
  overshoot_g: number | null
  scoop_duration_s: number | null
  pour_duration_s: number | null
}

type LogsResponse = {
  rows?: Row[]
  total?: number
  available_batches?: string[]
  available_modes?: string[]
}

function TimeSortIcon({
  order,
  active = true,
}: {
  order: 'asc' | 'desc'
  active?: boolean
}) {
  return (
    <svg
      className={`h-4 w-4 ${active ? 'text-[var(--text-primary)]' : 'text-[var(--text-faint)]'}`}
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
        className={order === 'asc' ? 'opacity-100' : 'opacity-25'}
      />
      <path
        d="M16 17L12 21L8 17"
        stroke="currentColor"
        strokeWidth="2"
        strokeLinecap="round"
        strokeLinejoin="round"
        className={order === 'desc' ? 'opacity-100' : 'opacity-25'}
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

export default function LogsPage() {
  const { apiBase } = useRuntimeConfig()
  const navigate = useNavigate()
  const [tableRows, setTableRows] = useState<Row[]>([])
  const [plotRows, setPlotRows] = useState<Row[]>([])
  const [loading, setLoading] = useState(false)
  const [manualRefreshing, setManualRefreshing] = useState(false)
  const [tableTotal, setTableTotal] = useState<number>(0)
  const [availableBatches, setAvailableBatches] = useState<string[]>([])
  const [availableModes, setAvailableModes] = useState<string[]>([])
  const [plotBatchFilter, setPlotBatchFilter] = useState<string>('all')
  const [plotModeFilter, setPlotModeFilter] = useState<string>('all')
  const [tableBatchFilter, setTableBatchFilter] = useState<string>('all')
  const [tableModeFilter, setTableModeFilter] = useState<string>('all')
  const [tableEpisodeFilter, setTableEpisodeFilter] = useState<string>('')
  const [tableTimeFrom, setTableTimeFrom] = useState<string>('')
  const [tableTimeTo, setTableTimeTo] = useState<string>('')
  const [tableTimeSort, setTableTimeSort] = useState<'asc' | 'desc'>('desc')
  const [tableTimeSortField, setTableTimeSortField] = useState<'start' | 'end'>('start')
  const [tablePage, setTablePage] = useState<number>(1)
  const tablePageSize = 10

  // helpers
  const buildLogsUrl = (params: {
    limit: number
    offset?: number
    batchFilter?: string
    modeFilter?: string
    episodeIndex?: number | null
    timeFrom?: string
    timeTo?: string
    timeSort?: 'asc' | 'desc'
    timeSortField?: 'start' | 'end'
  }) => {
    const search = new URLSearchParams()
    search.set('limit', String(params.limit))
    if ((params.offset ?? 0) > 0) {
      search.set('offset', String(params.offset))
    }
    if (params.batchFilter && params.batchFilter !== 'all') {
      search.set('batch_id', params.batchFilter)
    }
    if (params.modeFilter && params.modeFilter !== 'all') {
      search.set('mode', params.modeFilter)
    }
    if (params.episodeIndex != null) {
      search.set('episode_index', String(params.episodeIndex))
    }
    if (params.timeFrom) {
      search.set('time_from', params.timeFrom)
    }
    if (params.timeTo) {
      search.set('time_to', params.timeTo)
    }
    if (params.timeSort) {
      search.set('time_sort', params.timeSort)
    }
    if (params.timeSortField) {
      search.set('time_sort_field', params.timeSortField)
    }
    return `${apiBase}/lightsout_processed?${search.toString()}`
  }

  const loadRows = async ({ manual = false }: { manual?: boolean } = {}) => {
    try {
      setLoading(true)
      if (manual) {
        setManualRefreshing(true)
      }
      const rawEpisode = tableEpisodeFilter.trim()
      const parsedEpisode = rawEpisode ? Number.parseInt(rawEpisode, 10) : null
      const hasValidEpisode = parsedEpisode != null && Number.isFinite(parsedEpisode)
      const hasInvalidEpisode = rawEpisode.length > 0 && !hasValidEpisode

      const tableUrl = hasInvalidEpisode
        ? null
        : buildLogsUrl({
            limit: tablePageSize,
            offset: (tablePage - 1) * tablePageSize,
            batchFilter: tableBatchFilter,
            modeFilter: tableModeFilter,
            episodeIndex: hasValidEpisode ? parsedEpisode : null,
            timeFrom: toFilterIsoStart(tableTimeFrom),
            timeTo: toFilterIsoEnd(tableTimeTo),
            timeSort: tableTimeSort,
            timeSortField: tableTimeSortField,
          })
      const plotUrl = buildLogsUrl({
        limit: 0,
        batchFilter: plotBatchFilter,
        modeFilter: plotModeFilter,
      })

      const [tableJson, plotJson] = await Promise.all([
        tableUrl
          ? fetch(tableUrl).then((res) => res.json() as Promise<LogsResponse>)
          : Promise.resolve<LogsResponse>({ rows: [], total: 0 }),
        fetch(plotUrl).then((res) => res.json() as Promise<LogsResponse>),
      ])

      setTableRows(tableJson.rows || [])
      setTableTotal(tableJson.total ?? 0)
      setPlotRows(plotJson.rows || [])
      setAvailableBatches(tableJson.available_batches || plotJson.available_batches || [])
      setAvailableModes(tableJson.available_modes || plotJson.available_modes || [])
    } finally {
      setLoading(false)
      if (manual) {
        setManualRefreshing(false)
      }
    }
  }

  // init
  useEffect(() => {
    loadRows()
  }, [
    apiBase,
    plotBatchFilter,
    plotModeFilter,
    tableBatchFilter,
    tableModeFilter,
    tableEpisodeFilter,
    tableTimeFrom,
    tableTimeTo,
    tableTimeSort,
    tableTimeSortField,
    tablePage,
  ])

  const distinctBatches = availableBatches
  const distinctModes = availableModes

  const tablePageCount = Math.max(1, Math.ceil(tableTotal / tablePageSize))
  const tablePageSafe = Math.min(tablePage, tablePageCount)
  const visibleTableStart = tableTotal === 0 ? 0 : (tablePageSafe - 1) * tablePageSize + 1
  const visibleTableEnd =
    tableTotal === 0 ? 0 : Math.min(visibleTableStart + tableRows.length - 1, tableTotal)

  const plotDistinctBatches = Array.from(
    new Set(plotRows.map((row) => row.batch_id).filter((value): value is string => Boolean(value)))
  ).sort()

  useEffect(() => {
    setTablePage(1)
  }, [
    tableBatchFilter,
    tableModeFilter,
    tableEpisodeFilter,
    tableTimeFrom,
    tableTimeTo,
    tableTimeSort,
    tableTimeSortField,
  ])

  const toggleTableTimeSort = (field: 'start' | 'end') => {
    if (tableTimeSortField === field) {
      setTableTimeSort((current) => (current === 'desc' ? 'asc' : 'desc'))
      return
    }
    setTableTimeSortField(field)
    setTableTimeSort('desc')
  }

  const plotPointsFinalVsTarget = plotRows
    .filter((row) => row.target_weight_g != null && row.final_weight_g != null)
    .map((row) => ({
      x: row.target_weight_g as number,
      y: row.final_weight_g as number,
      label: `Run ${row.episode_index ?? '—'}`,
    }))

  const plotPointsDurationVsFinal = plotRows
    .filter((row) => row.total_episode_time_s != null && row.final_weight_g != null)
    .map((row) => ({
      x: row.total_episode_time_s as number,
      y: row.final_weight_g as number,
      label: `Run ${row.episode_index ?? '—'}`,
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

  const openBatchDetails = (row: Row) => {
    if (!row.event_id || row.weightment_id == null) {
      return
    }
    navigate(`/batches/${encodeURIComponent(row.event_id)}?weightmentId=${row.weightment_id}`)
  }

  const totalEpisodes = plotRows.length
  const avgFinal = averageOf(plotRows, (row) => row.final_weight_g)
  const avgNet = averageOf(plotRows, (row) => row.net_weight_g)
  const avgFlow = averageOf(plotRows, (row) => row.avg_flow_rate_g_s)
  const avgDuration = averageOf(plotRows, (row) => row.total_episode_time_s)
  const avgOvershoot = averageOf(plotRows, (row) => row.overshoot_g)

  const overshootByBatch = plotDistinctBatches.map((batch) => {
    const batchRows = plotRows.filter((row) => row.batch_id === batch)
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
            <h1 className="text-2xl font-bold" style={{ fontFamily: 'Space Grotesk' }}>Run History</h1>
            <p className="text-[var(--text-secondary)]">Processed robot traces and derived metrics</p>
          </div>
          <div className="flex items-center gap-2">
            <Button onClick={() => loadRows({ manual: true })} disabled={loading}>
              {manualRefreshing ? (
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
          <div className="mb-4 flex items-end justify-between gap-3">
            <div className="flex flex-1 items-end gap-3 overflow-x-auto pb-1">
              <FilterField label="Batch" className="min-w-[140px]">
                <Select
                  value={tableBatchFilter}
                  onChange={(event) => setTableBatchFilter(event.target.value)}
                >
                  <option value="all">All</option>
                  {distinctBatches.map((batch) => (
                    <option key={batch} value={batch}>{batch}</option>
                  ))}
                </Select>
              </FilterField>
              <FilterField label="Mode" className="min-w-[140px]">
                <Select
                  value={tableModeFilter}
                  onChange={(event) => setTableModeFilter(event.target.value)}
                >
                  <option value="all">All</option>
                  {distinctModes.map((mode) => (
                    <option key={mode} value={mode}>{mode}</option>
                  ))}
                </Select>
              </FilterField>
              <FilterField label="Run" className="min-w-[110px]">
                <input
                  className="w-24 rounded-md border border-[var(--border)] bg-[var(--surface)] px-2 py-1 text-sm text-[var(--text-primary)]"
                  placeholder="e.g. 3"
                  value={tableEpisodeFilter}
                  onChange={(event) => setTableEpisodeFilter(event.target.value)}
                />
              </FilterField>
              <DateTimeRangeField
                from={tableTimeFrom}
                to={tableTimeTo}
                onFromChange={setTableTimeFrom}
                onToChange={setTableTimeTo}
                onClear={() => {
                  setTableTimeFrom('')
                  setTableTimeTo('')
                }}
              />
            </div>
            <div className="shrink-0 rounded-xl border border-[var(--border)] bg-[var(--surface)]/70 px-3 py-2 text-xs text-[var(--text-muted)]">
              {tableTotal === 0
                ? 'Showing 0 of 0 entries'
                : `Showing ${visibleTableStart}-${visibleTableEnd} of ${tableTotal} entries`}
            </div>
          </div>
          <div className="overflow-auto">
            <table className="w-full text-sm">
              <thead className="text-left text-[var(--text-secondary)]">
                <tr>
                  <th>Batch</th>
                  <th>Run ID</th>
                  <th>Ingredient</th>
                  <th>
                    <button
                      type="button"
                      className="inline-flex items-center gap-1 font-medium hover:text-[var(--text-primary)]"
                      onClick={() => toggleTableTimeSort('start')}
                      aria-label={`Sort start time ${tableTimeSort === 'desc' ? 'ascending' : 'descending'}`}
                    >
                      <span>Start</span>
                      <TimeSortIcon order={tableTimeSort} active={tableTimeSortField === 'start'} />
                    </button>
                  </th>
                  <th>
                    <button
                      type="button"
                      className="inline-flex items-center gap-1 font-medium hover:text-[var(--text-primary)]"
                      onClick={() => toggleTableTimeSort('end')}
                      aria-label={`Sort end time ${tableTimeSort === 'desc' ? 'ascending' : 'descending'}`}
                    >
                      <span>End</span>
                      <TimeSortIcon order={tableTimeSort} active={tableTimeSortField === 'end'} />
                    </button>
                  </th>
                  <th className="text-right">Target (g)</th>
                  <th className="text-right">Final (g)</th>
                  <th className="text-right">Net (g)</th>
                  <th className="text-right">Duration (s)</th>
                  <th className="text-right">Scoop Duration (s)</th>
                  <th className="text-right">Pour Duration (s)</th>
                  <th className="text-right">Final Error (g)</th>
                </tr>
              </thead>
              <tbody>
                {loading ? (
                  <tr>
                    <td className="py-6 text-center" colSpan={12}>
                      <span className="inline-flex items-center gap-2 text-[var(--text-secondary)]">
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
                ) : tableRows.length === 0 ? (
                  <tr><td className="py-4" colSpan={12}>No data</td></tr>
                ) : (
                  tableRows.map((r) => {
                    const startMs = r.start_time_ns ? r.start_time_ns / 1_000_000 : null
                    const endMs = r.end_time_ns ? r.end_time_ns / 1_000_000 : null
                    const isBatchRow = Boolean(r.event_id && r.weightment_id != null)
                    return (
                      <tr
                        key={r.id}
                        className={`border-t border-[var(--border)] ${
                          isBatchRow ? 'cursor-pointer hover:bg-[var(--table-row-hover)]' : ''
                        }`}
                        onClick={isBatchRow ? () => openBatchDetails(r) : undefined}
                      >
                        <td>{r.batch_id ?? '—'}</td>
                        <td>{r.run_id ?? '—'}</td>
                        <td>{r.ingredient_id ?? '—'}</td>
                        <td><DateTimeText value={startMs} /></td>
                        <td><DateTimeText value={endMs} /></td>
                        <td className="text-right">{r.target_weight_g != null ? r.target_weight_g.toFixed(2) : '—'}</td>
                        <td className="text-right">{r.final_weight_g != null ? r.final_weight_g.toFixed(2) : '—'}</td>
                        <td className="text-right">{r.net_weight_g != null ? r.net_weight_g.toFixed(2) : '—'}</td>
                        <td className="text-right">{r.total_episode_time_s != null ? r.total_episode_time_s.toFixed(2) : '—'}</td>
                        <td className="text-right">{r.scoop_duration_s != null ? r.scoop_duration_s.toFixed(2) : '—'}</td>
                        <td className="text-right">{r.pour_duration_s != null ? r.pour_duration_s.toFixed(2) : '—'}</td>
                        <td className="text-right">{r.overshoot_g != null ? r.overshoot_g.toFixed(2) : '—'}</td>
                      </tr>
                    )
                  })
                )}
              </tbody>
            </table>
          </div>
          <div className="mt-3 flex items-center justify-between text-sm">
            <div className="text-[var(--text-muted)]">
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
            <div className="mb-2 text-xs text-[var(--text-muted)]">Run Summary</div>
            <div className="grid grid-cols-2 gap-3 text-sm">
              <div>Total Runs</div>
              <div className="text-right">{totalEpisodes}</div>
              <div>Avg Final (g)</div>
              <div className="text-right">{avgFinal != null ? avgFinal.toFixed(2) : '—'}</div>
              <div>Avg Net (g)</div>
              <div className="text-right">{avgNet != null ? avgNet.toFixed(2) : '—'}</div>
              <div>Avg Flow (g/s)</div>
              <div className="text-right">{avgFlow != null ? avgFlow.toFixed(2) : '—'}</div>
              <div>Avg Duration (s)</div>
              <div className="text-right">{avgDuration != null ? avgDuration.toFixed(2) : '—'}</div>
              <div>Avg Final Error (g)</div>
              <div className="text-right">{avgOvershoot != null ? avgOvershoot.toFixed(2) : '—'}</div>
            </div>
          </GlassCard>
          {/* <GlassCard>
            <div className="mb-2 text-xs text-[var(--text-muted)]">Avg Final Error by Batch</div>
            {overshootByBatch.length === 0 ? (
              <div className="text-sm text-[var(--text-muted)]">No batch data.</div>
            ) : (
              <div className="space-y-2">
                {overshootByBatch.map((item) => {
                  const maxValue = Math.max(1, ...overshootByBatch.map((x) => Math.abs(x.avgOvershoot)))
                  const width = Math.min(100, Math.abs(item.avgOvershoot) / maxValue * 100)
                  return (
                    <div key={item.batch} className="text-xs">
                      <div className="flex justify-between mb-1">
                        <span className="text-[var(--text-secondary)]">{item.batch} ({item.count})</span>
                        <span className="text-[var(--text-muted)]">{item.avgOvershoot.toFixed(2)} g</span>
                      </div>
                      <div className="h-2 rounded-full bg-[var(--surface-strong)]">
                        <div
                          className="h-2 rounded-full bg-[var(--status-good-fg)]"
                          style={{ width: `${width}%` }}
                        />
                      </div>
                    </div>
                  )
                })}
              </div>
            )}
          </GlassCard> */}
        </div>

        <div className="mt-8">
          <div className="flex items-center justify-between mb-3">
            <h2 className="text-lg font-semibold">Plots (shadcn/ui)</h2>
            <div className="flex items-center gap-2 text-sm">
              <span className="text-[var(--text-muted)]">Batch</span>
              <Select
                value={plotBatchFilter}
                onChange={(event) => setPlotBatchFilter(event.target.value)}
              >
                <option value="all">All</option>
                {distinctBatches.map((batch) => (
                  <option key={batch} value={batch}>{batch}</option>
                ))}
              </Select>
              <span className="text-[var(--text-muted)]">Mode</span>
              <Select
                value={plotModeFilter}
                onChange={(event) => setPlotModeFilter(event.target.value)}
              >
                <option value="all">All</option>
                {distinctModes.map((mode) => (
                  <option key={mode} value={mode}>{mode}</option>
                ))}
              </Select>
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
                      <CartesianGrid strokeDasharray="3 3" stroke="var(--chart-grid)" />
                      <XAxis
                        dataKey="target"
                        type="number"
                        name="Target"
                        unit="g"
                        tick={{ fill: 'var(--chart-axis)', fontSize: 11 }}
                      />
                      <YAxis
                        dataKey="final"
                        type="number"
                        name="Final"
                        unit="g"
                        tick={{ fill: 'var(--chart-axis)', fontSize: 11 }}
                      />
                      <Tooltip cursor={{ strokeDasharray: '3 3' }} />
                      <Legend />
                      <Scatter
                        name={`Episodes (${rechartsFinalVsTarget.length})`}
                        data={rechartsFinalVsTarget}
                        fill="var(--accent)"
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
                      <CartesianGrid strokeDasharray="3 3" stroke="var(--chart-grid)" />
                      <XAxis
                        dataKey="duration"
                        type="number"
                        name="Duration"
                        unit="s"
                        tick={{ fill: 'var(--chart-axis)', fontSize: 11 }}
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
                <CardTitle>Final Error Distribution (% of Runs)</CardTitle>
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


