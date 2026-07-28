import { useEffect, useMemo, useState } from 'react'
import ReactECharts from 'echarts-for-react'
import {
  api,
  type DashboardPanel,
  type LogLine,
  type QueryResult,
} from '../../lib/api'
import { cn } from '../../lib/cn'
import {
  buildCartesianOption,
  buildGaugeOption,
  buildPieOption,
  buildStatSparklineOption,
  colorForValue,
  DEFAULT_THRESHOLDS,
  type SeriesBundle,
} from '../../lib/echartsTheme'
import { formatValue } from '../../lib/formatUnits'

function formatTick(ts: number): string {
  try {
    return new Date(ts * 1000).toLocaleTimeString([], {
      hour: '2-digit',
      minute: '2-digit',
    })
  } catch {
    return ''
  }
}

function matrixToBundle(result: unknown[]): SeriesBundle {
  const series = (result || []) as Array<{
    metric?: Record<string, string>
    values?: Array<[number | string, string]>
    value?: [number | string, string]
  }>
  const byTime = new Map<number, Record<string, number>>()
  const keys: string[] = []

  series.forEach((s, idx) => {
    const metric = s.metric || {}
    const name =
      metric.container ||
      metric.name ||
      metric.instance ||
      metric.__name__ ||
      `series_${idx}`
    if (!keys.includes(name)) keys.push(name)
    const values = s.values || (s.value ? [s.value] : [])
    for (const pair of values) {
      if (!pair || pair.length < 2) continue
      const t = Number(pair[0])
      const v = Number(pair[1])
      if (Number.isNaN(t) || Number.isNaN(v)) continue
      let row = byTime.get(t)
      if (!row) {
        row = {}
        byTime.set(t, row)
      }
      row[name] = v
    }
  })

  const timestamps = Array.from(byTime.keys()).sort((a, b) => a - b)
  const categories = timestamps.map(formatTick)
  return {
    categories,
    timestamps,
    series: keys.map((name) => ({
      name,
      data: timestamps.map((t) => {
        const v = byTime.get(t)?.[name]
        return v == null ? null : v
      }),
    })),
  }
}

function vectorToRows(result: unknown[]): Array<Record<string, string>> {
  return ((result || []) as Array<{
    metric?: Record<string, string>
    value?: [number | string, string]
  }>).map((s, i) => {
    const metric = s.metric || {}
    const value = s.value?.[1] ?? ''
    return {
      n: String(i + 1),
      ...metric,
      value: String(value),
    }
  })
}

function latestVectorSlices(
  result: unknown[],
): Array<{ name: string; value: number }> {
  const series = (result || []) as Array<{
    metric?: Record<string, string>
    values?: Array<[number | string, string]>
    value?: [number | string, string]
  }>
  return series
    .map((s, idx) => {
      const metric = s.metric || {}
      const name =
        metric.container ||
        metric.mode ||
        metric.name ||
        metric.instance ||
        metric.__name__ ||
        `series_${idx}`
      let raw: string | number | undefined
      if (s.value) raw = s.value[1]
      else if (s.values?.length) raw = s.values[s.values.length - 1][1]
      const value = Number(raw)
      return { name, value }
    })
    .filter((s) => !Number.isNaN(s.value))
}

function ChartBox({
  option,
  empty,
}: {
  option: object | null
  empty?: string
}) {
  if (!option) {
    return (
      <div className="px-2 py-6 text-center text-xs text-[var(--text-muted)]">
        {empty || 'No data'}
      </div>
    )
  }
  return (
    <ReactECharts
      option={option}
      style={{ height: '100%', width: '100%' }}
      opts={{ renderer: 'canvas' }}
      notMerge
      lazyUpdate
    />
  )
}

export default function Panel({
  panel,
  deviceId,
  sinceSeconds = 3600,
  refreshKey = 0,
  className,
}: {
  panel: DashboardPanel
  deviceId?: string
  sinceSeconds?: number
  refreshKey?: number
  className?: string
}) {
  const [data, setData] = useState<QueryResult | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [loading, setLoading] = useState(false)

  const isInstant =
    panel.type === 'stat' ||
    panel.type === 'gauge' ||
    panel.type === 'pie' ||
    panel.type === 'table'

  useEffect(() => {
    let cancelled = false
    const end = Date.now() / 1000
    const start = end - sinceSeconds
    setLoading(true)
    setError(null)
    api
      .query({
        ds: panel.datasource,
        expr: panel.query,
        start:
          isInstant && panel.datasource === 'prometheus' ? undefined : start,
        end: isInstant && panel.datasource === 'prometheus' ? undefined : end,
        step:
          isInstant && panel.datasource === 'prometheus' ? 'instant' : '30s',
        limit: panel.type === 'logs' ? 200 : 500,
        device_id: deviceId,
        direction: 'forward',
      })
      .then((payload) => {
        if (!cancelled) setData(payload)
      })
      .catch((err) => {
        if (!cancelled)
          setError(err instanceof Error ? err.message : String(err))
      })
      .finally(() => {
        if (!cancelled) setLoading(false)
      })
    return () => {
      cancelled = true
    }
  }, [
    panel.datasource,
    panel.query,
    panel.type,
    deviceId,
    sinceSeconds,
    refreshKey,
    isInstant,
  ])

  const bundle = useMemo(() => {
    if (!data || data.ds === 'loki') return null
    return matrixToBundle(data.result || [])
  }, [data])

  const statValue = useMemo(() => {
    if (bundle && bundle.series.length && bundle.timestamps.length) {
      const s = bundle.series[0]
      for (let i = s.data.length - 1; i >= 0; i--) {
        const v = s.data[i]
        if (typeof v === 'number') return v
      }
    }
    const slices = latestVectorSlices(data?.result || [])
    if (slices.length) return slices[0].value
    return null
  }, [bundle, data])

  const pieData = useMemo(
    () => latestVectorSlices(data?.result || []),
    [data],
  )

  const logLines: LogLine[] = data?.lines || []
  const unit = panel.unit
  const opts = panel.options

  const cartesianOption = useMemo(() => {
    if (
      !bundle ||
      !bundle.series.length ||
      (panel.type !== 'timeseries' &&
        panel.type !== 'area' &&
        panel.type !== 'bar')
    ) {
      return null
    }
    return buildCartesianOption({
      type: panel.type,
      bundle,
      unit,
      options: opts,
    })
  }, [bundle, panel.type, unit, opts])

  const gaugeOption = useMemo(() => {
    if (panel.type !== 'gauge') return null
    return buildGaugeOption({
      value: statValue,
      unit,
      min: panel.min,
      max: panel.max,
      options: opts,
    })
  }, [panel.type, panel.min, panel.max, statValue, unit, opts])

  const pieOption = useMemo(() => {
    if (panel.type !== 'pie' || !pieData.length) return null
    return buildPieOption({ slices: pieData, unit, options: opts })
  }, [panel.type, pieData, unit, opts])

  const sparkOption = useMemo(() => {
    if (panel.type !== 'stat' || !bundle?.series[0]) return null
    const values = bundle.series[0].data.filter(
      (v): v is number => typeof v === 'number',
    )
    if (values.length < 2) return null
    return buildStatSparklineOption({
      values,
      color: colorForValue(statValue, opts?.thresholds),
    })
  }, [panel.type, bundle, statValue, opts])

  const rows = useMemo(
    () => (panel.type === 'table' ? vectorToRows(data?.result || []) : []),
    [panel.type, data],
  )

  return (
    <div
      className={cn(
        'flex h-full flex-col overflow-hidden rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--surface-1)]',
        className,
      )}
    >
      <div className="flex items-center justify-between gap-2 border-b border-[var(--border)] px-3 py-2">
        <div className="min-w-0">
          <div className="truncate text-sm font-medium text-[var(--text-primary)]">
            {panel.title || 'Untitled'}
          </div>
          <div className="truncate font-mono text-[10px] text-[var(--text-muted)]">
            {panel.datasource} · {panel.type}
          </div>
        </div>
        {loading ? (
          <span className="text-[10px] text-[var(--text-muted)]">…</span>
        ) : null}
      </div>
      <div className="min-h-0 flex-1 p-2">
        {error ? (
          <div className="px-2 py-4 text-xs text-rose-300">{error}</div>
        ) : null}

        {!error && panel.type === 'stat' ? (
          <div className="flex h-full flex-col items-center justify-center">
            <div
              className="font-display text-3xl font-semibold"
              style={{
                color: colorForValue(statValue, opts?.thresholds),
              }}
            >
              {statValue == null
                ? '—'
                : opts?.decimals != null
                  ? `${statValue.toFixed(opts.decimals)}${unit === 'percent' || unit === '%' ? '%' : ''}`
                  : formatValue(statValue, unit)}
            </div>
            {sparkOption ? (
              <div className="mt-2 h-12 w-full">
                <ChartBox option={sparkOption} />
              </div>
            ) : null}
          </div>
        ) : null}

        {!error && panel.type === 'gauge' ? (
          <div className="flex h-full flex-col">
            <div className="min-h-0 flex-1">
              <ChartBox option={gaugeOption} empty="No value" />
            </div>
            <div className="flex flex-wrap items-center justify-center gap-2 px-1 pb-1 pt-0.5">
              {(opts?.thresholds?.length
                ? opts.thresholds
                : DEFAULT_THRESHOLDS
              )
                .slice()
                .sort((a, b) => a.upTo - b.upTo)
                .map((s, i) => (
                  <span
                    key={`${s.upTo}-${i}`}
                    className="inline-flex items-center gap-1.5 rounded-full border border-[var(--border)] bg-[var(--surface-2)] px-2 py-0.5 text-[10px] text-[var(--text-secondary)]"
                  >
                    <span
                      className="h-2 w-2 rounded-full"
                      style={{ background: s.color }}
                    />
                    <span className="font-medium" style={{ color: s.color }}>
                      {s.label || 'Stage'}
                    </span>
                    <span className="font-mono text-[var(--text-muted)]">
                      ≤ {formatValue(s.upTo, unit)}
                    </span>
                  </span>
                ))}
            </div>
          </div>
        ) : null}

        {!error && panel.type === 'pie' ? (
          <ChartBox option={pieOption} empty="No series" />
        ) : null}

        {!error &&
        (panel.type === 'timeseries' ||
          panel.type === 'area' ||
          panel.type === 'bar') ? (
          <ChartBox option={cartesianOption} empty="No series" />
        ) : null}

        {!error && panel.type === 'table' ? (
          <div className="h-full overflow-auto">
            <table className="w-full text-left text-[11px]">
              <thead className="sticky top-0 bg-[var(--surface-1)] text-[var(--text-muted)]">
                <tr>
                  {Object.keys(rows[0] || { value: '' }).map((h) => (
                    <th key={h} className="px-2 py-1 font-medium">
                      {h}
                    </th>
                  ))}
                </tr>
              </thead>
              <tbody>
                {rows.map((row, i) => (
                  <tr key={i} className="border-t border-[var(--border)]">
                    {Object.entries(row).map(([k, v], j) => (
                      <td
                        key={j}
                        className="max-w-[12rem] truncate px-2 py-1 font-mono text-[var(--text-secondary)]"
                      >
                        {k === 'value' && !Number.isNaN(Number(v))
                          ? formatValue(Number(v), unit)
                          : v}
                      </td>
                    ))}
                  </tr>
                ))}
              </tbody>
            </table>
            {!rows.length ? (
              <div className="px-2 py-6 text-center text-xs text-[var(--text-muted)]">
                No rows
              </div>
            ) : null}
          </div>
        ) : null}

        {!error && panel.type === 'logs' ? (
          <div className="h-full overflow-auto font-mono text-[11px] leading-[1.5]">
            {logLines.length === 0 ? (
              <div className="px-2 py-6 text-center text-xs text-[var(--text-muted)]">
                No log lines
              </div>
            ) : (
              logLines.map((line, i) => (
                <div key={`${line.ts}-${i}`} className="flex gap-2 px-1 py-0.5">
                  <span className="shrink-0 text-slate-600">
                    {formatTick(line.ts)}
                  </span>
                  <span className="shrink-0 text-sky-500/80">
                    {line.container || '—'}
                  </span>
                  <span className="whitespace-pre-wrap break-all text-slate-400">
                    {line.text}
                  </span>
                </div>
              ))
            )}
          </div>
        ) : null}
      </div>
    </div>
  )
}
