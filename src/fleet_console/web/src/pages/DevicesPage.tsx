import { useEffect, useState } from 'react'
import { useNavigate } from 'react-router-dom'
import { ExternalLink, LayoutDashboard, RefreshCw } from 'lucide-react'
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
  const [alertCounts, setAlertCounts] = useState<Record<string, number>>({})
  const [error, setError] = useState<string | null>(null)
  const [loading, setLoading] = useState(true)

  const load = async () => {
    setLoading(true)
    setError(null)
    try {
      const [payload, alerts] = await Promise.all([
        api.listDevices(),
        api.listAlerts().catch(() => ({
          alerts: [],
          counts_by_instance: {} as Record<string, number>,
        })),
      ])
      setDevices(payload.devices)
      setAlertCounts(alerts.counts_by_instance || {})
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
  const needsBuild = devices.filter((d) => d.needs_build).length
  const driftCount = devices.filter((d) => d.drift?.any).length

  return (
    <div>
      <SectionHeader
        title="Fleet devices"
        description="Fleet management view: provision, deploy, and observe robots here. Open each cell’s operator dashboard for live weighment controls."
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
        <MetricCard
          label="Deployable updates"
          value={updates}
          help="Newer Release image than desired/running"
        />
        <MetricCard
          label={needsBuild ? 'Needs CI build' : 'Drift'}
          value={needsBuild || driftCount}
          help={
            needsBuild
              ? 'Branch tip has commits with no Release yet'
              : 'Desired ≠ running / agent'
          }
        />
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
                <div className="text-xs text-[var(--text-muted)]">
                  {d.ip}
                  {d.device_class || d.platform
                    ? ` · ${d.device_class || '?'}${d.platform ? ` / ${d.platform.replace('linux/', '')}` : ''}`
                    : ''}
                </div>
              </div>
            ),
          },
          {
            key: 'dashboard',
            header: 'Robot UI',
            render: (d) =>
              d.dashboard_url ? (
                <div className="flex flex-col gap-1" onClick={(e) => e.stopPropagation()}>
                  <a
                    href={d.dashboard_url}
                    target="_blank"
                    rel="noreferrer"
                    className="inline-flex items-center gap-1 text-sm font-medium text-[var(--accent)] hover:underline"
                    title={d.dashboard_url}
                  >
                    <LayoutDashboard className="h-3.5 w-3.5" />
                    Dashboard
                    <ExternalLink className="h-3 w-3 opacity-70" />
                  </a>
                  {d.dashboard_url_ip && d.dashboard_url_ip !== d.dashboard_url ? (
                    <a
                      href={d.dashboard_url_ip}
                      target="_blank"
                      rel="noreferrer"
                      className="text-[11px] text-[var(--text-muted)] hover:text-[var(--accent)]"
                      title="Tailscale IP fallback"
                    >
                      via IP
                    </a>
                  ) : (
                    <span className="text-[11px] text-[var(--text-faint)]">:8080</span>
                  )}
                </div>
              ) : (
                <span className="text-xs text-[var(--text-muted)]">—</span>
              ),
          },
          {
            key: 'alive',
            header: 'Alive / active',
            render: (d) => {
              const alerts =
                alertCounts[d.hostname] ||
                alertCounts[d.id] ||
                0
              return (
                <div className="flex flex-col gap-1">
                  <div className="flex flex-wrap items-center gap-1">
                    <StatusBadge
                      label={d.alive ? 'alive' : d.online ? 'online' : 'down'}
                      tone={aliveTone(d)}
                      pulse={Boolean(d.alive)}
                    />
                    {alerts > 0 ? (
                      <StatusBadge
                        label={`${alerts} alert${alerts === 1 ? '' : 's'}`}
                        tone="bad"
                        pulse
                      />
                    ) : null}
                  </div>
                  <StatusBadge
                    label={d.active ? 'running' : 'idle'}
                    tone={d.active ? 'info' : 'neutral'}
                  />
                </div>
              )
            },
          },
          {
            key: 'desired',
            header: 'Desired',
            render: (d) => (
              <div className="text-xs">
                <code className="text-[var(--accent)]">
                  {d.desired_image_tag || '—'}
                </code>
                <div className="text-[var(--text-muted)]">
                  {d.desired_branch || 'main'} · {d.desired_profile_id || '—'}
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
                  {d.running_source === 'agent' ? ' · via agent' : ''}
                </div>
              </div>
            ),
          },
          {
            key: 'agent',
            header: 'Agent',
            render: (d) => {
              const s = d.agent_status
              const tone =
                s === 'success' || s === 'converged'
                  ? 'good'
                  : s === 'applying'
                    ? 'info'
                    : s === 'rolled_back'
                      ? 'warn'
                      : s === 'failed'
                        ? 'bad'
                        : 'neutral'
              return (
                <StatusBadge
                  label={s || '—'}
                  tone={tone}
                  pulse={s === 'applying'}
                />
              )
            },
          },
          {
            key: 'update',
            header: 'Update',
            render: (d) => {
              if (d.update_available) {
                return (
                  <div className="text-xs">
                    <StatusBadge
                      label={`deploy ${d.latest_sha || 'new'}`}
                      tone="warn"
                      pulse
                    />
                    <div className="mt-1 text-[var(--text-muted)]">
                      {d.latest_release?.subject || 'newer Release ready'}
                    </div>
                  </div>
                )
              }
              if (d.needs_build) {
                return (
                  <div className="text-xs">
                    <StatusBadge
                      label={`build ${d.branch_tip_sha || 'tip'}`}
                      tone="info"
                    />
                    <div className="mt-1 max-w-[12rem] truncate text-[var(--text-muted)]">
                      {d.branch_tip_message || 'branch tip not built yet'}
                    </div>
                  </div>
                )
              }
              if (d.drift?.any) {
                return <StatusBadge label="drift" tone="warn" />
              }
              return <StatusBadge label="current" tone="good" />
            },
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
