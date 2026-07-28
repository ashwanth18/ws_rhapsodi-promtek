import { useEffect, useState } from 'react'
import {
  CartesianGrid,
  Line,
  LineChart,
  ResponsiveContainer,
  Tooltip,
  XAxis,
  YAxis,
} from 'recharts'
import { api, type DeviceSeries, type FleetAlert } from '../lib/api'
import { StatusBadge } from './ui'

function toChart(points: Array<{ t: number; v: number }>) {
  return points.map((p) => ({
    t: p.t,
    label: new Date(p.t * 1000).toLocaleTimeString([], {
      hour: '2-digit',
      minute: '2-digit',
    }),
    v: p.v,
  }))
}

function MiniChart({
  title,
  data,
  color,
}: {
  title: string
  data: Array<{ label: string; v: number }>
  color: string
}) {
  return (
    <div className="rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--surface-1)] p-3">
      <div className="mb-2 text-xs font-medium text-[var(--text-secondary)]">
        {title}
      </div>
      <div className="h-36">
        {data.length === 0 ? (
          <div className="flex h-full items-center justify-center text-xs text-[var(--text-muted)]">
            No series
          </div>
        ) : (
          <ResponsiveContainer width="100%" height="100%">
            <LineChart data={data}>
              <CartesianGrid strokeDasharray="3 3" stroke="#ffffff10" />
              <XAxis dataKey="label" tick={{ fill: '#94a3b8', fontSize: 10 }} />
              <YAxis
                domain={[0, 100]}
                tick={{ fill: '#94a3b8', fontSize: 10 }}
                unit="%"
                width={36}
              />
              <Tooltip
                contentStyle={{
                  background: '#0d121a',
                  border: '1px solid #1e293b',
                  fontSize: 11,
                }}
              />
              <Line
                type="monotone"
                dataKey="v"
                stroke={color}
                dot={false}
                strokeWidth={1.5}
                isAnimationActive={false}
              />
            </LineChart>
          </ResponsiveContainer>
        )}
      </div>
    </div>
  )
}

export default function DeviceMetricsCharts({
  deviceId,
}: {
  deviceId: string
}) {
  const [series, setSeries] = useState<DeviceSeries | null>(null)
  const [alerts, setAlerts] = useState<FleetAlert[]>([])
  const [since, setSince] = useState('1h')
  const [error, setError] = useState<string | null>(null)

  useEffect(() => {
    let cancelled = false
    Promise.all([
      api.deviceSeries(deviceId, { since }),
      api.listAlerts(deviceId),
    ])
      .then(([s, a]) => {
        if (cancelled) return
        setSeries(s)
        setAlerts(a.alerts)
        setError(null)
      })
      .catch((err) => {
        if (!cancelled)
          setError(err instanceof Error ? err.message : String(err))
      })
    return () => {
      cancelled = true
    }
  }, [deviceId, since])

  return (
    <div className="space-y-4">
      <div className="flex flex-wrap items-center justify-between gap-2">
        <h3 className="font-display text-base font-semibold">Host metrics</h3>
        <select
          className="rounded border border-[var(--border)] bg-[var(--surface-2)] px-2 py-1 text-xs"
          value={since}
          onChange={(e) => setSince(e.target.value)}
        >
          <option value="15m">15m</option>
          <option value="1h">1h</option>
          <option value="6h">6h</option>
          <option value="24h">24h</option>
        </select>
      </div>
      {error ? (
        <div className="text-xs text-rose-300">{error}</div>
      ) : null}
      <div className="grid gap-3 md:grid-cols-3">
        <MiniChart
          title="CPU %"
          color="#38bdf8"
          data={toChart(series?.cpu_pct || [])}
        />
        <MiniChart
          title="Memory %"
          color="#a78bfa"
          data={toChart(series?.mem_pct || [])}
        />
        <MiniChart
          title="Disk %"
          color="#fbbf24"
          data={toChart(series?.disk_pct || [])}
        />
      </div>

      <div>
        <h3 className="mb-2 font-display text-base font-semibold">
          Active alerts
        </h3>
        {alerts.length === 0 ? (
          <div className="rounded border border-dashed border-[var(--border)] px-3 py-6 text-center text-xs text-[var(--text-muted)]">
            No active alerts for this device.
          </div>
        ) : (
          <ul className="space-y-2">
            {alerts.map((a) => (
              <li
                key={`${a.alertname}-${a.fingerprint || a.active_at}`}
                className="flex flex-wrap items-center justify-between gap-2 rounded border border-[var(--border)] bg-[var(--surface-1)] px-3 py-2"
              >
                <div>
                  <div className="text-sm font-medium">{a.alertname}</div>
                  <div className="text-xs text-[var(--text-muted)]">
                    {a.annotations?.summary || a.annotations?.description || ''}
                  </div>
                </div>
                <StatusBadge
                  label={a.state}
                  tone={
                    ['firing', 'active'].includes(a.state.toLowerCase())
                      ? 'bad'
                      : 'warn'
                  }
                  pulse
                />
              </li>
            ))}
          </ul>
        )}
      </div>
    </div>
  )
}
