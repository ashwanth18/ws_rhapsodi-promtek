export type UnitCode =
  | 'none'
  | 'short'
  | 'percent'
  | 'percentunit'
  | 'bytes'
  | 'bytesRate'
  | 'seconds'
  | 'milliseconds'
  | 'celsius'
  | 'ops'
  | 'bool'

export const UNIT_OPTIONS: { value: UnitCode; label: string }[] = [
  { value: 'none', label: 'None' },
  { value: 'short', label: 'Short (K/M/B)' },
  { value: 'percent', label: 'Percent (0–100)' },
  { value: 'percentunit', label: 'Percent unit (0–1)' },
  { value: 'bytes', label: 'Bytes' },
  { value: 'bytesRate', label: 'Bytes/s' },
  { value: 'seconds', label: 'Seconds' },
  { value: 'milliseconds', label: 'Milliseconds' },
  { value: 'celsius', label: 'Celsius' },
  { value: 'ops', label: 'ops/s' },
  { value: 'bool', label: 'Boolean' },
]

function si(value: number, suffix = ''): string {
  const abs = Math.abs(value)
  const units = [
    { v: 1e12, s: 'T' },
    { v: 1e9, s: 'G' },
    { v: 1e6, s: 'M' },
    { v: 1e3, s: 'K' },
  ]
  for (const u of units) {
    if (abs >= u.v) return `${(value / u.v).toFixed(2)}${u.s}${suffix}`
  }
  if (abs >= 100 || Number.isInteger(value)) return `${value.toFixed(0)}${suffix}`
  if (abs >= 10) return `${value.toFixed(1)}${suffix}`
  return `${value.toFixed(2)}${suffix}`
}

function bytes(value: number, rate = false): string {
  const abs = Math.abs(value)
  const units = ['B', 'KiB', 'MiB', 'GiB', 'TiB']
  let v = abs
  let i = 0
  while (v >= 1024 && i < units.length - 1) {
    v /= 1024
    i += 1
  }
  const sign = value < 0 ? '-' : ''
  const s = v >= 100 ? v.toFixed(0) : v >= 10 ? v.toFixed(1) : v.toFixed(2)
  return `${sign}${s} ${units[i]}${rate ? '/s' : ''}`
}

function duration(seconds: number): string {
  const abs = Math.abs(seconds)
  if (abs < 0.001) return `${(seconds * 1e6).toFixed(0)} µs`
  if (abs < 1) return `${(seconds * 1000).toFixed(1)} ms`
  if (abs < 60) return `${seconds.toFixed(2)} s`
  if (abs < 3600) return `${(seconds / 60).toFixed(1)} m`
  if (abs < 86400) return `${(seconds / 3600).toFixed(1)} h`
  return `${(seconds / 86400).toFixed(1)} d`
}

/** Format a numeric value using Grafana-style short unit codes. */
export function formatValue(
  value: number | null | undefined,
  unit?: string | null,
): string {
  if (value == null || Number.isNaN(value)) return '—'
  const u = (unit || 'none') as UnitCode | string
  switch (u) {
    case 'none':
    case '':
      return Number.isInteger(value) ? String(value) : value.toFixed(2)
    case 'short':
      return si(value)
    case 'percent':
      return `${value.toFixed(1)}%`
    case 'percentunit':
      return `${(value * 100).toFixed(1)}%`
    case 'bytes':
      return bytes(value, false)
    case 'bytesRate':
      return bytes(value, true)
    case 'seconds':
      return duration(value)
    case 'milliseconds':
      return duration(value / 1000)
    case 'celsius':
      return `${value.toFixed(1)} °C`
    case 'ops':
      return `${si(value)}/s`
    case 'bool':
      return value ? 'true' : 'false'
    default:
      // Legacy free-text suffix (e.g. "%")
      return `${Number.isInteger(value) ? value : value.toFixed(1)}${u}`
  }
}

/** Compact tick formatter for chart axes. */
export function formatAxisTick(value: number, unit?: string | null): string {
  if (value == null || Number.isNaN(value)) return ''
  const u = unit || 'none'
  if (u === 'percent' || u === '%') return `${Math.round(value)}`
  if (u === 'percentunit') return `${Math.round(value * 100)}`
  if (u === 'bytes' || u === 'bytesRate') {
    const abs = Math.abs(value)
    if (abs >= 1e9) return `${(value / 1e9).toFixed(0)}G`
    if (abs >= 1e6) return `${(value / 1e6).toFixed(0)}M`
    if (abs >= 1e3) return `${(value / 1e3).toFixed(0)}K`
    return String(Math.round(value))
  }
  if (u === 'short' || u === 'ops') return si(value).replace(/\.00$/, '')
  return Math.abs(value) >= 100 ? String(Math.round(value)) : value.toFixed(1)
}

export function gaugeMax(unit?: string | null): number {
  if (unit === 'percent') return 100
  if (unit === 'percentunit') return 1
  return 100
}
