import type { EChartsOption } from 'echarts'
import type { PanelOptions, PanelType, ThresholdStep } from './api'
import { formatAxisTick, formatValue, gaugeMax } from './formatUnits'

export const SERIES_COLORS = [
  '#38bdf8',
  '#a78bfa',
  '#34d399',
  '#fbbf24',
  '#f472b6',
  '#fb7185',
]

export const DEFAULT_THRESHOLDS: ThresholdStep[] = [
  { upTo: 70, color: '#34d399', label: 'OK' },
  { upTo: 90, color: '#fbbf24', label: 'Warn' },
  { upTo: 100, color: '#fb7185', label: 'Critical' },
]

export function defaultOptionsForType(type: PanelType): PanelOptions {
  if (type === 'gauge') {
    return {
      showPointer: true,
      thresholds: DEFAULT_THRESHOLDS.map((t) => ({ ...t })),
    }
  }
  if (type === 'area') {
    return { showLegend: true, smooth: true, fillOpacity: 0.25 }
  }
  if (type === 'timeseries' || type === 'bar') {
    return { showLegend: true, smooth: true, stack: false }
  }
  if (type === 'pie') {
    return { showLegend: true, donut: true }
  }
  if (type === 'stat') {
    return { decimals: 1, thresholds: DEFAULT_THRESHOLDS.map((t) => ({ ...t })) }
  }
  return {}
}

/** Convert absolute threshold steps into ECharts [fraction, color] pairs. */
export function thresholdsToAxisColors(
  thresholds: ThresholdStep[] | undefined,
  min: number,
  max: number,
): [number, string][] {
  const span = max - min
  if (!(span > 0)) return [[1, '#38bdf8']]
  const steps = [...(thresholds?.length ? thresholds : DEFAULT_THRESHOLDS)].sort(
    (a, b) => a.upTo - b.upTo,
  )
  const out: [number, string][] = []
  for (const s of steps) {
    const frac = Math.max(0, Math.min(1, (s.upTo - min) / span))
    out.push([frac, s.color])
  }
  if (!out.length) return [[1, '#38bdf8']]
  // Ensure last segment reaches 1
  out[out.length - 1][0] = 1
  return out
}

export function colorForValue(
  value: number | null,
  thresholds: ThresholdStep[] | undefined,
  fallback = '#38bdf8',
): string {
  if (value == null || !thresholds?.length) return fallback
  const sorted = [...thresholds].sort((a, b) => a.upTo - b.upTo)
  for (const s of sorted) {
    if (value <= s.upTo) return s.color
  }
  return sorted[sorted.length - 1]?.color || fallback
}

const baseText = '#94a3b8'
const gridLine = 'rgba(255,255,255,0.06)'

export type SeriesBundle = {
  categories: string[]
  timestamps: number[]
  series: Array<{ name: string; data: (number | null)[] }>
}

export function buildCartesianOption(args: {
  type: 'timeseries' | 'area' | 'bar'
  bundle: SeriesBundle
  unit?: string
  options?: PanelOptions
}): EChartsOption {
  const { type, bundle, unit, options } = args
  const smooth = options?.smooth !== false
  const stack = options?.stack ? 'total' : undefined
  const showLegend = options?.showLegend !== false
  const fillOpacity = options?.fillOpacity ?? 0.25
  const chartType = type === 'bar' ? 'bar' : 'line'
  const area = type === 'area'

  return {
    backgroundColor: 'transparent',
    animation: false,
    color: SERIES_COLORS,
    grid: {
      left: 48,
      right: 12,
      top: showLegend ? 28 : 12,
      bottom: 28,
    },
    legend: showLegend
      ? {
          type: 'scroll',
          top: 0,
          textStyle: { color: baseText, fontSize: 10 },
          pageTextStyle: { color: baseText },
        }
      : undefined,
    tooltip: {
      trigger: 'axis',
      backgroundColor: '#0d121a',
      borderColor: '#1e293b',
      textStyle: { color: '#e2e8f0', fontSize: 11 },
      valueFormatter: (v) =>
        formatValue(typeof v === 'number' ? v : Number(v), unit),
    },
    xAxis: {
      type: 'category',
      data: bundle.categories,
      boundaryGap: type === 'bar',
      axisLabel: { color: baseText, fontSize: 10 },
      axisLine: { lineStyle: { color: '#1e293b' } },
      axisTick: { show: false },
    },
    yAxis: {
      type: 'value',
      splitLine: { lineStyle: { color: gridLine } },
      axisLabel: {
        color: baseText,
        fontSize: 10,
        formatter: (v: number) => formatAxisTick(v, unit),
      },
    },
    series: bundle.series.map((s, i) => ({
      name: s.name,
      type: chartType,
      data: s.data,
      smooth: chartType === 'line' ? smooth : undefined,
      showSymbol: false,
      stack,
      areaStyle: area
        ? {
            opacity: fillOpacity,
            color: SERIES_COLORS[i % SERIES_COLORS.length],
          }
        : undefined,
      lineStyle: { width: 1.5 },
      itemStyle:
        chartType === 'bar'
          ? { color: SERIES_COLORS[i % SERIES_COLORS.length] }
          : undefined,
    })),
  }
}

function stageForValue(
  value: number | null,
  thresholds: ThresholdStep[],
): ThresholdStep | null {
  if (value == null || !thresholds.length) return null
  const sorted = [...thresholds].sort((a, b) => a.upTo - b.upTo)
  for (const s of sorted) {
    if (value <= s.upTo) return s
  }
  return sorted[sorted.length - 1] || null
}

/** Place stage boundary labels around the gauge arc (ECharts graphic). */
function stageLabelGraphics(
  thresholds: ThresholdStep[],
  min: number,
  max: number,
): EChartsOption['graphic'] {
  const span = max - min
  if (!(span > 0) || !thresholds.length) return []
  const startAngle = 210
  const endAngle = -30
  const sweep = startAngle - endAngle // 240
  const cx = 50 // %
  const cy = 56 // % — matches series center
  const r = 38 // % of container — outside the arc a bit

  return [...thresholds]
    .sort((a, b) => a.upTo - b.upTo)
    .map((s) => {
      const f = Math.max(0, Math.min(1, (s.upTo - min) / span))
      const deg = startAngle - f * sweep
      const rad = (deg * Math.PI) / 180
      // Screen y grows downward; ECharts angles are math-style from +x CCW,
      // but gauge uses clock-like mapping — match ECharts gauge convention:
      const x = cx + r * Math.cos(rad)
      const y = cy - r * Math.sin(rad)
      const label = s.label?.trim() || formatAxisTick(s.upTo, null)
      return {
        type: 'group' as const,
        left: `${x}%`,
        top: `${y}%`,
        bounding: 'raw' as const,
        children: [
          {
            type: 'circle' as const,
            shape: { cx: 0, cy: 0, r: 3.5 },
            style: { fill: s.color, stroke: '#0d121a', lineWidth: 1 },
            z: 10,
          },
          {
            type: 'text' as const,
            style: {
              text: `${label}\n${formatAxisTick(s.upTo, null)}`,
              fill: s.color,
              fontSize: 10,
              fontWeight: 600,
              align: 'center',
              verticalAlign: 'middle',
              textAlign: 'center',
            },
            // Offset label slightly outward from the dot
            x: 0,
            y: deg > 90 || deg < -90 ? 14 : -14,
            z: 10,
          },
        ],
      }
    })
}

export function buildGaugeOption(args: {
  value: number | null
  unit?: string
  min?: number
  max?: number
  options?: PanelOptions
}): EChartsOption {
  const min = args.min ?? 0
  const max = args.max ?? gaugeMax(args.unit)
  const steps = [
    ...(args.options?.thresholds?.length
      ? args.options.thresholds
      : DEFAULT_THRESHOLDS),
  ].sort((a, b) => a.upTo - b.upTo)
  const colors = thresholdsToAxisColors(steps, min, max)
  const showPointer = args.options?.showPointer !== false
  const decimals = args.options?.decimals
  const display =
    args.value == null
      ? '—'
      : decimals != null
        ? args.value.toFixed(decimals) +
          (args.unit === 'percent' || args.unit === '%'
            ? '%'
            : args.unit && args.unit !== 'none' && args.unit !== 'short'
              ? ''
              : '')
        : formatValue(args.value, args.unit)

  const active = stageForValue(args.value, steps)
  const span = max - min
  // Dense enough splits so axisLabel can highlight stage boundaries
  const splitNumber = Math.min(20, Math.max(steps.length, Math.round(span) || 10))

  return {
    backgroundColor: 'transparent',
    animation: false,
    graphic: stageLabelGraphics(steps, min, max),
    series: [
      {
        type: 'gauge',
        min,
        max,
        startAngle: 210,
        endAngle: -30,
        radius: '78%',
        center: ['50%', '56%'],
        splitNumber,
        axisLine: {
          lineStyle: {
            width: 16,
            color: colors,
          },
        },
        pointer: showPointer
          ? {
              length: '52%',
              width: 5,
              itemStyle: { color: 'auto' },
            }
          : { show: false },
        axisTick: {
          show: true,
          distance: -16,
          length: 5,
          splitNumber: 2,
          lineStyle: { color: '#475569', width: 1 },
        },
        splitLine: {
          show: true,
          distance: -16,
          length: 12,
          lineStyle: { color: '#94a3b8', width: 1.5 },
        },
        axisLabel: {
          distance: 20,
          color: baseText,
          fontSize: 9,
          formatter: (v: number) => {
            const hit = steps.find(
              (s) => Math.abs(s.upTo - v) <= Math.max(span * 0.02, 0.51),
            )
            if (hit) return '' // stage labels drawn via graphic
            if (Math.abs(v - min) <= Math.max(span * 0.02, 0.51)) {
              return formatAxisTick(min, args.unit)
            }
            return ''
          },
        },
        detail: {
          valueAnimation: false,
          formatter: () => display,
          color: active?.color || 'auto',
          fontSize: 22,
          fontWeight: 600,
          offsetCenter: [0, '68%'],
        },
        title: {
          show: Boolean(active?.label),
          offsetCenter: [0, '88%'],
          color: active?.color || baseText,
          fontSize: 12,
          fontWeight: 600,
        },
        data: [
          {
            value: args.value ?? min,
            name: active?.label || '',
          },
        ],
      },
    ],
  }
}

export function buildPieOption(args: {
  slices: Array<{ name: string; value: number }>
  unit?: string
  options?: PanelOptions
}): EChartsOption {
  const donut = args.options?.donut !== false
  const showLegend = args.options?.showLegend !== false
  return {
    backgroundColor: 'transparent',
    animation: false,
    color: SERIES_COLORS,
    tooltip: {
      trigger: 'item',
      backgroundColor: '#0d121a',
      borderColor: '#1e293b',
      textStyle: { color: '#e2e8f0', fontSize: 11 },
      valueFormatter: (v) =>
        formatValue(typeof v === 'number' ? v : Number(v), args.unit),
    },
    legend: showLegend
      ? {
          type: 'scroll',
          bottom: 0,
          textStyle: { color: baseText, fontSize: 10 },
        }
      : undefined,
    series: [
      {
        type: 'pie',
        radius: donut ? ['45%', '72%'] : ['0%', '72%'],
        center: ['50%', showLegend ? '46%' : '50%'],
        data: args.slices,
        label: { color: baseText, fontSize: 10 },
        itemStyle: { borderColor: '#0d121a', borderWidth: 2 },
      },
    ],
  }
}

export function buildStatSparklineOption(args: {
  values: number[]
  color: string
}): EChartsOption {
  return {
    backgroundColor: 'transparent',
    animation: false,
    grid: { left: 0, right: 0, top: 4, bottom: 0 },
    xAxis: { type: 'category', show: false, data: args.values.map((_, i) => i) },
    yAxis: { type: 'value', show: false, scale: true },
    series: [
      {
        type: 'line',
        data: args.values,
        showSymbol: false,
        smooth: true,
        lineStyle: { color: args.color, width: 1.5 },
        areaStyle: { color: args.color, opacity: 0.2 },
      },
    ],
  }
}
