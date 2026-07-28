import type { PanelBuilderState, PromBuilderFn } from './api'

export const PROM_FNS: { value: PromBuilderFn; label: string }[] = [
  { value: 'raw', label: 'Raw (no wrapper)' },
  { value: 'rate', label: 'rate()' },
  { value: 'irate', label: 'irate()' },
  { value: 'increase', label: 'increase()' },
  { value: 'avg', label: 'avg()' },
  { value: 'sum', label: 'sum()' },
  { value: 'max', label: 'max()' },
  { value: 'min', label: 'min()' },
  { value: 'cpu_pct_from_idle', label: 'CPU % from idle' },
  { value: 'mem_pct_used', label: 'Memory % used' },
  { value: 'disk_root_pct', label: 'Disk % used (/)' },
  { value: 'temp_max', label: 'Temperature max °C' },
]

export const RANGE_OPTIONS = ['1m', '2m', '5m', '15m', '1h'] as const

function escapeLabelValue(v: string): string {
  return v.replace(/\\/g, '\\\\').replace(/"/g, '\\"')
}

function selector(
  metric: string,
  matchers: Array<{ label: string; value: string }>,
): string {
  const parts = matchers
    .filter((m) => m.label.trim() && m.value.trim())
    .map((m) => `${m.label.trim()}="${escapeLabelValue(m.value.trim())}"`)
  if (!parts.length) return metric
  return `${metric}{${parts.join(',')}}`
}

function byClause(groupBy: string[] | undefined): string {
  const labels = (groupBy || []).map((g) => g.trim()).filter(Boolean)
  if (!labels.length) return ''
  return ` by (${labels.join(', ')})`
}

function instanceMatchers(
  matchers: Array<{ label: string; value: string }>,
): Array<{ label: string; value: string }> {
  const hasInstance = matchers.some((m) => m.label.trim() === 'instance')
  if (hasInstance) return matchers.filter((m) => m.label.trim() && m.value.trim())
  return [{ label: 'instance', value: '$device_id' }, ...matchers]
}

/** Compile guided Prometheus fields into PromQL. */
export function buildPromQL(state: PanelBuilderState): string {
  const metric = (state.metric || '').trim()
  const matchers = state.matchers || []
  const fn = state.fn || 'raw'
  const range = state.range || '5m'
  const by = byClause(state.groupBy)

  if (fn === 'cpu_pct_from_idle') {
    const m = metric || 'node_cpu_seconds_total'
    const idleMatchers = [
      ...matchers.filter((x) => x.label.trim() !== 'mode'),
      { label: 'mode', value: 'idle' },
    ]
    const idleSel = selector(m, idleMatchers)
    return `100 - (avg${by} (rate(${idleSel}[${range}])) * 100)`
  }

  if (fn === 'mem_pct_used') {
    const mm = instanceMatchers(matchers)
    const avail = selector('node_memory_MemAvailable_bytes', mm)
    const total = selector('node_memory_MemTotal_bytes', mm)
    return `(1 - (${avail} / ${total})) * 100`
  }

  if (fn === 'disk_root_pct') {
    const base = instanceMatchers(matchers).filter((m) => m.label !== 'fstype')
    const withMount = base.some((m) => m.label === 'mountpoint')
      ? base
      : [...base, { label: 'mountpoint', value: '/' }]
    const parts = withMount
      .filter((m) => m.label.trim() && m.value.trim())
      .map((m) => `${m.label.trim()}="${escapeLabelValue(m.value.trim())}"`)
    parts.push('fstype!="rootfs"')
    const labels = parts.join(',')
    return (
      `(1 - (node_filesystem_avail_bytes{${labels}} ` +
      `/ node_filesystem_size_bytes{${labels}})) * 100`
    )
  }

  if (fn === 'temp_max') {
    const m = metric || 'node_hwmon_temp_celsius'
    const sel = selector(m, instanceMatchers(matchers))
    return `max${by || ' by (instance)'} (${sel})`
  }

  if (!metric) return ''
  const sel = selector(metric, matchers)

  if (fn === 'rate' || fn === 'irate' || fn === 'increase') {
    const inner = `${fn}(${sel}[${range}])`
    if (by) return `sum${by} (${inner})`
    return inner
  }
  if (fn === 'avg' || fn === 'sum' || fn === 'max' || fn === 'min') {
    return `${fn}${by} (${sel})`
  }
  return sel
}

/** Compile guided Loki fields into LogQL. */
export function buildLogQL(state: PanelBuilderState): string {
  const parts: string[] = []
  const host = (state.host || '').trim() || '$device_id'
  parts.push(`host="${escapeLabelValue(host)}"`)
  const container = (state.container || '').trim()
  if (container) {
    parts.push(`compose_service="${escapeLabelValue(container)}"`)
  }
  const stream = (state.stream || '').trim()
  if (stream === 'stdout' || stream === 'stderr') {
    parts.push(`stream="${stream}"`)
  }
  let q = `{${parts.join(',')}}`
  const line = (state.lineFilter || '').trim()
  if (line) {
    q += ` |= "${escapeLabelValue(line)}"`
  }
  return q
}

export function defaultBuilder(
  datasource: 'prometheus' | 'loki',
): PanelBuilderState {
  if (datasource === 'loki') {
    return {
      mode: 'guided',
      host: '$device_id',
      container: '',
      lineFilter: '',
      stream: '',
    }
  }
  return {
    mode: 'guided',
    metric: 'node_cpu_seconds_total',
    matchers: [{ label: 'instance', value: '$device_id' }],
    fn: 'cpu_pct_from_idle',
    groupBy: ['instance'],
    range: '2m',
  }
}

export function compileQuery(
  datasource: 'prometheus' | 'loki',
  state: PanelBuilderState,
): string {
  return datasource === 'loki' ? buildLogQL(state) : buildPromQL(state)
}
