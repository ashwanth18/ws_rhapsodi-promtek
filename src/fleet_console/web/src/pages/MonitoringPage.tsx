import { useEffect, useState } from 'react'
import { Link } from 'react-router-dom'
import { Activity, RefreshCw } from 'lucide-react'
import { api, type FleetAlert, type FleetSummary } from '../lib/api'
import {
  Button,
  DataTable,
  MetricCard,
  SectionHeader,
  StatusBadge,
} from '../components/ui'

function alertTone(state: string): 'good' | 'bad' | 'warn' | 'info' | 'neutral' {
  const s = state.toLowerCase()
  if (s === 'firing' || s === 'active') return 'bad'
  if (s === 'pending') return 'warn'
  if (s === 'silenced' || s === 'suppressed') return 'info'
  return 'neutral'
}

export default function MonitoringPage() {
  const [summary, setSummary] = useState<FleetSummary | null>(null)
  const [alerts, setAlerts] = useState<FleetAlert[]>([])
  const [error, setError] = useState<string | null>(null)
  const [loading, setLoading] = useState(true)

  const load = async () => {
    setLoading(true)
    setError(null)
    try {
      const [fleet, alertPayload] = await Promise.all([
        api.fleetMetrics(),
        api.listAlerts(),
      ])
      setSummary(fleet)
      setAlerts(alertPayload.alerts)
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err))
    } finally {
      setLoading(false)
    }
  }

  useEffect(() => {
    void load()
    const id = window.setInterval(() => void load(), 30000)
    return () => window.clearInterval(id)
  }, [])

  return (
    <div>
      <SectionHeader
        title="Monitoring"
        description="Fleet health from Prometheus + Alertmanager. For custom charts and log queries, use Dashboards."
        action={
          <div className="flex gap-2">
            <Link
              to="/dashboards"
              className="inline-flex items-center gap-2 rounded-[var(--radius-sm)] border border-[var(--border)] px-3 py-2 text-sm text-[var(--text-secondary)] hover:bg-white/5"
            >
              <Activity className="h-4 w-4" />
              Custom dashboards
            </Link>
            <Button variant="outline" onClick={() => void load()} disabled={loading}>
              <RefreshCw className="h-4 w-4" />
              Refresh
            </Button>
          </div>
        }
      />

      {error ? (
        <div className="mb-4 rounded border border-[var(--status-bad-fg)]/40 bg-[var(--status-bad-bg)] px-3 py-2 text-sm text-[var(--status-bad-fg)]">
          {error}
        </div>
      ) : null}

      <div className="mb-6 grid gap-3 sm:grid-cols-2 lg:grid-cols-4">
        <MetricCard
          label="Devices up"
          value={
            summary
              ? `${summary.devices_up}/${summary.devices_total}`
              : '—'
          }
        />
        <MetricCard
          label="Devices down"
          value={summary?.devices_down ?? '—'}
          help="node_exporter unreachable"
        />
        <MetricCard
          label="Active alerts"
          value={summary?.alerts_total ?? alerts.length}
        />
        <MetricCard
          label="Pressure"
          value={
            summary
              ? `${summary.high_mem.length} mem · ${summary.high_disk.length} disk`
              : '—'
          }
          help="Hosts ≥90% RAM or ≥85% disk"
        />
      </div>

      <SectionHeader
        title="Active alerts"
        description="Merged from Prometheus ALERTS and Alertmanager (silence-aware)."
      />
      <DataTable
        rows={alerts}
        rowKey={(a) =>
          `${a.fingerprint || a.alertname}-${a.instance}-${a.state}-${a.active_at || ''}`
        }
        emptyMessage={
          loading ? 'Loading alerts…' : 'No alerts — fleet looks quiet.'
        }
        columns={[
          {
            key: 'alert',
            header: 'Alert',
            render: (a) => (
              <div>
                <div className="font-medium">{a.alertname}</div>
                <div className="text-xs text-[var(--text-muted)]">
                  {a.annotations?.summary ||
                    a.annotations?.description ||
                    a.source}
                </div>
              </div>
            ),
          },
          {
            key: 'instance',
            header: 'Instance',
            render: (a) =>
              a.instance ? (
                <Link
                  className="font-mono text-xs text-[var(--accent)] hover:underline"
                  to={`/devices/${encodeURIComponent(a.instance)}`}
                >
                  {a.instance}
                </Link>
              ) : (
                <span className="text-xs text-[var(--text-muted)]">—</span>
              ),
          },
          {
            key: 'severity',
            header: 'Severity',
            render: (a) => (
              <StatusBadge
                label={a.severity || 'none'}
                tone={
                  a.severity === 'critical'
                    ? 'bad'
                    : a.severity === 'warning'
                      ? 'warn'
                      : 'neutral'
                }
              />
            ),
          },
          {
            key: 'state',
            header: 'State',
            render: (a) => (
              <StatusBadge
                label={a.state}
                tone={alertTone(a.state)}
                pulse={['firing', 'active'].includes(a.state.toLowerCase())}
              />
            ),
          },
          {
            key: 'since',
            header: 'Since',
            render: (a) => (
              <span className="text-xs text-[var(--text-muted)]">
                {a.active_at
                  ? typeof a.active_at === 'number'
                    ? new Date(a.active_at * 1000).toLocaleString()
                    : new Date(a.active_at).toLocaleString()
                  : '—'}
              </span>
            ),
          },
        ]}
      />
    </div>
  )
}
