import Plot from 'react-plotly.js'

type Point = { x: number; y: number }
type Props = { data: Point[]; height?: number; xMax?: number; xMin?: number }

export default function PlotlyLine({ data, height = 260, xMax, xMin = 0 }: Props) {
  return (
    <Plot
      data={[
        {
          x: data.map((d) => d.x),
          y: data.map((d) => d.y),
          type: 'scatter',
          mode: 'lines',
          line: { color: '#90caf9', width: 2 },
          fill: 'tozeroy',
          fillcolor: 'rgba(66,165,245,0.2)',
        },
      ]}
      layout={{
        paper_bgcolor: 'rgba(0,0,0,0)',
        plot_bgcolor: 'rgba(0,0,0,0)',
        height,
        margin: { l: 48, r: 16, t: 20, b: 28 },
        font: { color: '#b8c2d9' },
        xaxis: { gridcolor: 'rgba(255,255,255,0.12)', title: 's', range: xMax != null ? [xMin, xMax] : undefined },
        yaxis: { gridcolor: 'rgba(255,255,255,0.12)' },
        showlegend: false,
      }}
      config={{ displayModeBar: false, responsive: true }}
      useResizeHandler
      style={{ width: '100%' }}
    />
  )
}


