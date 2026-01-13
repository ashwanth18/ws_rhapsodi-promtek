import ReactECharts from 'echarts-for-react'

type Point = { x: number; y: number }
type Props = { data: Point[]; height?: number; xLabel?: string; yLabel?: string }

export default function EChartsScatter({ data, height = 220, xLabel = 'Batch', yLabel = 'Overshoot %' }: Props) {
  const option = {
    backgroundColor: 'transparent',
    grid: { left: 48, right: 16, top: 20, bottom: 36 },
    xAxis: {
      type: 'value',
      name: xLabel,
      nameTextStyle: { color: '#b8c2d9', padding: [0, 0, 0, 6] },
      axisLine: { lineStyle: { color: 'rgba(255,255,255,0.12)' } },
      axisLabel: { color: '#b8c2d9' },
      splitLine: { lineStyle: { color: 'rgba(255,255,255,0.12)' } },
    },
    yAxis: {
      type: 'value',
      name: yLabel,
      nameTextStyle: { color: '#b8c2d9', padding: [0, 0, 6, 0] },
      axisLine: { show: false },
      axisLabel: { color: '#b8c2d9' },
      splitLine: { lineStyle: { color: 'rgba(255,255,255,0.12)' } },
    },
    tooltip: {
      trigger: 'item',
      backgroundColor: 'rgba(10,11,13,0.95)',
      borderColor: 'rgba(144,202,249,0.35)',
      textStyle: { color: '#e6eefc' },
      formatter: (p: any) => `${xLabel}: ${p.data[0]}<br/><b>${yLabel}: ${p.data[1].toFixed(2)}%</b>`,
    },
    series: [
      {
        type: 'scatter',
        data: data.map((d) => [d.x, d.y]),
        symbolSize: 6,
        itemStyle: { color: '#38bdf8' },
        emphasis: { itemStyle: { color: '#60a5fa' } },
      },
    ],
  }
  return <ReactECharts option={option} style={{ width: '100%', height }} notMerge lazyUpdate opts={{ renderer: 'svg' }} />
}













