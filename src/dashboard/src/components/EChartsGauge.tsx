import ReactECharts from 'echarts-for-react'

type Props = {
  value: number | null
  target: number
  tolerance: number // fraction
  max: number
}

export default function EChartsGauge({ value, target, tolerance, max }: Props) {
  const v = typeof value === 'number' ? value : 0
  const lo = Math.max(0, target * (1 - tolerance))
  const hi = Math.min(max, target * (1 + tolerance))
  const option = {
    backgroundColor: 'transparent',
    series: [
      {
        type: 'gauge',
        startAngle: 220,
        endAngle: -40,
        min: 0,
        max,
        splitNumber: 10,
        progress: { show: true, width: 10, itemStyle: { color: '#38bdf8' } },
        axisLine: { lineStyle: { width: 10, color: [[lo / max, '#fb7185'], [hi / max, '#34d399'], [1, '#fb7185']] } },
        axisTick: { show: false },
        splitLine: { show: false },
        axisLabel: { show: false },
        pointer: { show: true, length: '70%', width: 3 },
        itemStyle: { color: '#e6eefc' },
        title: { show: false },
        detail: { valueAnimation: true, formatter: (x: number) => `${x.toFixed(3)} kg`, color: '#e6eefc', fontSize: 18 },
        data: [{ value: v }],
      },
    ],
  }
  return <ReactECharts option={option} style={{ width: '100%', height: 200 }} notMerge lazyUpdate opts={{ renderer: 'svg' }} />
}













