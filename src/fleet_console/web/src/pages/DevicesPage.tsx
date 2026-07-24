import { useEffect, useState } from 'react'
import { useNavigate } from 'react-router-dom'
import { RefreshCw } from 'lucide-react'
import { api, type Device } from '../lib/api'
import {
  Button,
  DataTable,
  MetricCard,
  SectionHeader,
  StatusBadge,
  deployTone,
} from '../components/ui'

function aliveTone(device: Device): 'good' | 'bad' | 'warn' | 'neutral' {
  if (device.alive === true) return 'good'
  if (device.alive === false) return 'bad'
  if (device.online) return 'warn'
  return 'neutral'
}

export default function DevicesPage() {
  const navigate = useNavigate()
  const [devices, setDevices] = useState<Device[]>([])
  const [error, setError] = useState<string | null>(null)
  const [loading, setLoading] = useState(true)

  const load = async () => {
    setLoading(true)
    setError(null)
    try {
      const payload = await api.listDevices()
      setDevices(payload.devices)
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err))
    } finally {
      setLoading(false)
    }
  }

  useEffect(() => {
    load()
    const id = window.setInterval(load, 15000)
    return () => window.clearInterval(id)
  }, [])

  const aliveCount = devices.filter((d) => d.alive).length
  const updates = devices.filter((d) => d.update_available).length
  const driftCount = devices.filter((d) => d.drift?.any).length

  return (
    <div>
      <SectionHeader
        title="Fleet devices"
        description="Version (branch/SHA) and profile (runtime behavior) are independent axes."
        action={
          <Button variant="outline" onClick={load} disabled={loading}>
            <RefreshCw className="h-4 w-4" />
            Refresh
          </Button>
        }
      />

      <div className="mb-6 grid gap-3 sm:grid-cols-2 lg:grid-cols-4">
        <MetricCard label="Devices" value={devices.length} />
        <MetricCard label="Alive" value={aliveCount} />
        <MetricCard label="Updates available" value={updates} help="Tracked branch moved" />
        <MetricCard label="Drift" value={driftCount} help="Desired ≠ running" />
      </div>

      {error ? (
        <div className="mb-4 rounded-[var(--radius-md)] border border-[var(--status-bad-fg)]/40 bg-[var(--status-bad-bg)] px-4 py-3 text-sm text-[var(--status-bad-fg)]">
          {error}
        </div>
      ) : null}

      <DataTable
        rows={devices}
        rowKey={(d) => d.id}
        emptyMessage={loading ? 'Loading devices…' : 'No robot devices found on Tailscale.'}
        onRowClick={(d) => navigate(`/devices/${d.id}`)}
        columns={[
          {
            key: 'hostname',
            header: 'Device',
            render: (d) => (
              <div>
                <div className="font-medium text-[var(--text-primary)]">{d.hostname}</div>
                <div className="text-xs text-[var(--text-muted)]">{d.ip}</div>
              </div>
            ),
          },
          {
            key: 'alive',
            header: 'Alive / active',
            render: (d) => (
              <div className="flex flex-col gap-1">
                <StatusBadge
                  label={d.alive ? 'alive' : d.online ? 'online' : 'down'}
                  tone={aliveTone(d)}
                  pulse={Boolean(d.alive)}
                />
                <StatusBadge
                  label={d.active ? 'running' : 'idle'}
                  tone={d.active ? 'info' : 'neutral'}
                />
              </div>
            ),
          },
          {
            key: 'desired',
            header: 'Desired',
            render: (d) => (
              <div className="text-xs">
                <div className="text-[var(--text-secondary)]">
                  {d.desired_branch || 'main'}
                </div>
                <div className="text-[var(--text-muted)]">
                  {d.desired_profile_id || '—'}
                </div>
              </div>
            ),
          },
          {
            key: 'running',
            header: 'Running',
            render: (d) => (
              <div className="text-xs">
                <code className="text-[var(--accent)]">{d.image_tag || '—'}</code>
                <div className="text-[var(--text-muted)]">
                  {d.running_profile_id || (d.provisioned ? '—' : 'unprovisioned')}
                </div>
              </div>
            ),
          },
          {
            key: 'update',
            header: 'Update',
            render: (d) =>
              d.update_available ? (
                <StatusBadge
                  label={`→ ${d.latest_sha || 'new'}`}
                  tone="warn"
                  pulse
                />
              ) : d.drift?.any ? (
                <StatusBadge label="drift" tone="warn" />
              ) : (
                <StatusBadge label="current" tone="good" />
              ),
          },
          {
            key: 'metrics',
            header: 'CPU / RAM / Disk',
            render: (d) => (
              <div className="text-xs text-[var(--text-muted)]">
                {d.metrics?.cpu_pct != null ? `${d.metrics.cpu_pct}%` : '—'} /{' '}
                {d.metrics?.mem_pct != null ? `${d.metrics.mem_pct}%` : '—'} /{' '}
                {d.metrics?.disk_pct != null ? `${d.metrics.disk_pct}%` : '—'}
              </div>
            ),
          },
          {
            key: 'deploy',
            header: 'Last job',
            render: (d) =>
              d.last_deployment ? (
                <StatusBadge
                  label={`${d.last_deployment.action}:${d.last_deployment.status}`}
                  tone={deployTone(d.last_deployment.status)}
                  pulse={d.last_deployment.status === 'running'}
                />
              ) : (
                <span className="text-xs text-[var(--text-muted)]">—</span>
              ),
          },
        ]}
      />
    </div>
  )
}
