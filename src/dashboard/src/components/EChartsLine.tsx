import ReactECharts from 'echarts-for-react'

type Point = { x: number; y: number }
type Props = { data: Point[]; height?: number; xMax?: number; xMin?: number }

export default function EChartsLine({ data, height = 260, xMax, xMin = 0 }: Props) {
  const option = {
    backgroundColor: 'transparent',
    grid: { left: 40, right: 16, top: 20, bottom: 28 },
    xAxis: {
      type: 'value',
      name: 's',
      nameTextStyle: { color: '#b8c2d9', padding: [0, 0, 0, 6] },
      min: xMin,
      max: xMax,
      axisLine: { lineStyle: { color: 'rgba(255,255,255,0.12)' } },
      axisLabel: { color: '#b8c2d9' },
      axisTick: { show: false },
    },
    yAxis: {
      type: 'value',
      axisLine: { show: false },
      splitLine: { lineStyle: { color: 'rgba(255,255,255,0.12)' } },
      axisLabel: { color: '#b8c2d9' },
    },
    tooltip: {
      trigger: 'axis',
      backgroundColor: 'rgba(15,22,36,0.95)',
      borderColor: 'rgba(144,202,249,0.35)',
      textStyle: { color: '#e6eefc' },
      formatter: (params: any) => {
        const p = params[0]
        return `${p.axisValueLabel}<br/><b>${Number(p.data).toFixed(3)} kg</b>`
      },
    },
    series: [
      {
        type: 'line',
        data: data.map((d) => [d.x, d.y]),
        smooth: true,
        showSymbol: false,
        lineStyle: { color: '#90caf9', width: 2 },
        areaStyle: {
          color: {
            type: 'linear', x: 0, y: 0, x2: 0, y2: 1,
            colorStops: [
              { offset: 0, color: 'rgba(66,165,245,0.6)' },
              { offset: 1, color: 'rgba(66,165,245,0.05)' },
            ],
          },
        },
      },
    ],
  }

  // Important: set width to 100% and prefer SVG renderer for reliability across environments
  return (
    <ReactECharts
      option={option}
      style={{ width: '100%', height }}
      notMerge
      lazyUpdate
      opts={{ renderer: 'svg' }}
    />
  )
}


