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
  const activeCount = devices.filter((d) => d.active).length
  const unprovisioned = devices.filter((d) => !d.provisioned).length

  return (
    <div>
      <SectionHeader
        title="Fleet devices"
        description="Tailscale robots with live health, version, and deploy status."
        action={
          <Button variant="outline" onClick={load} disabled={loading}>
            <RefreshCw className="h-4 w-4" />
            Refresh
          </Button>
        }
      />

      <div className="mb-6 grid gap-3 sm:grid-cols-2 lg:grid-cols-4">
        <MetricCard label="Devices" value={devices.length} />
        <MetricCard label="Alive" value={aliveCount} help="Prometheus / host_info" />
        <MetricCard label="Active run" value={activeCount} />
        <MetricCard label="Needs flash install" value={unprovisioned} />
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
            key: 'type',
            header: 'Robot / site',
            render: (d) => (
              <div className="text-sm">
                <div>{d.robot_type || (d.provisioned ? '—' : 'unprovisioned')}</div>
                <div className="text-xs text-[var(--text-muted)]">{d.site_id || '—'}</div>
              </div>
            ),
          },
          {
            key: 'alive',
            header: 'Alive',
            render: (d) => (
              <StatusBadge
                label={d.alive ? 'alive' : d.online ? 'online' : 'down'}
                tone={aliveTone(d)}
                pulse={Boolean(d.alive)}
              />
            ),
          },
          {
            key: 'active',
            header: 'Active',
            render: (d) => (
              <StatusBadge
                label={d.active ? 'running' : 'idle'}
                tone={d.active ? 'info' : 'neutral'}
                pulse={d.active}
              />
            ),
          },
          {
            key: 'version',
            header: 'Version',
            render: (d) => (
              <code className="text-xs text-[var(--accent)]">{d.image_tag || '—'}</code>
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
            header: 'Last deploy',
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
