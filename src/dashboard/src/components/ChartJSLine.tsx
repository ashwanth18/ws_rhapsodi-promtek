import { Line } from 'react-chartjs-2'
import {
  Chart as ChartJS,
  LineElement,
  PointElement,
  LinearScale,
  CategoryScale,
  Filler,
  Tooltip,
} from 'chart.js'

ChartJS.register(LineElement, PointElement, LinearScale, CategoryScale, Filler, Tooltip)

type Point = { x: string | number; y: number }
type Props = { data: Point[]; height?: number }

export default function ChartJSLine({ data, height = 220 }: Props) {
  const labels = data.map((d) => d.x)
  const values = data.map((d) => d.y)
  return (
    <Line
      height={height}
      options={{
        responsive: true,
        maintainAspectRatio: false,
        plugins: { legend: { display: false }, tooltip: { enabled: true } },
        scales: {
          x: {
            ticks: { color: '#b8c2d9', maxRotation: 0 },
            grid: { color: 'rgba(255,255,255,0.12)' },
          },
          y: {
            ticks: { color: '#b8c2d9' },
            grid: { color: 'rgba(255,255,255,0.12)' },
          },
        },
      }}
      data={{
        labels,
        datasets: [
          {
            data: values,
            borderColor: '#90caf9',
            backgroundColor: 'rgba(66,165,245,0.2)',
            fill: true,
            tension: 0.35,
            pointRadius: 0,
          },
        ],
      }}
    />
  )
}













