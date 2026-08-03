import { useEffect, useState } from 'react'
import { Link, useNavigate } from 'react-router-dom'
import { ExternalLink, LayoutDashboard, RefreshCw } from 'lucide-react'
import {
  api,
  releaseSummary,
  releaseVersion,
  shortSha,
  type Device,
} from '../lib/api'
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
        description="Fleet management view: provision, deploy, and observe robots here. Open each cell’s operator dashboard for live weighment controls. Versions are Release #N (git sha) — not SemVer. Devices track a git branch (usually main); CI publishes to the orphan deploy branch as deploy-<sha> tags."
        action={
          <Button variant="outline" onClick={load} disabled={loading}>
            <RefreshCw className="h-4 w-4" />
            Refresh
          </Button>
        }
      />

      {updates > 0 ? (
        <div className="mb-4 rounded-[var(--radius-md)] border border-[var(--status-warn-fg)]/40 bg-[var(--status-warn-bg)] px-4 py-3 text-sm text-[var(--status-warn-fg)]">
          {updates === 1
            ? '1 device has a newer verified Release ready to deploy. '
            : `${updates} devices have a newer verified Release ready to deploy. `}
          <Link
            className="font-medium underline underline-offset-2"
            to={(() => {
              const latest = devices.find((d) => d.update_available)?.latest_release
              if (latest?.id) return `/releases?focus=${latest.id}`
              return '/releases'
            })()}
          >
            See what changed on Releases
          </Link>
          , then open the device and Deploy.
        </div>
      ) : null}

      <div className="mb-6 grid gap-3 sm:grid-cols-2 lg:grid-cols-4">
        <MetricCard label="Devices" value={devices.length} />
        <MetricCard label="Alive" value={aliveCount} />
        <MetricCard
          label="Updates ready"
          value={updates}
          help="Newer verified Release than desired/running — open device to deploy"
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
            key: 'mode',
            header: 'Mode / env',
            render: (d) => (
              <div className="text-xs">
                <div className="font-medium text-[var(--text-secondary)]">
                  {d.active_mode || '—'}
                </div>
                <div className="text-[var(--text-muted)]">
                  {d.environment || '—'}
                </div>
              </div>
            ),
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
            header: 'Running version',
            render: (d) => {
              const desired = d.desired_release
              const label = desired
                ? releaseVersion(desired)
                : d.image_tag
                  ? shortSha(d.image_tag)
                  : '—'
              return (
                <div className="text-xs">
                  <div className="font-medium text-[var(--text-secondary)]">
                    {label}
                  </div>
                  <code className="text-[10px] text-[var(--text-muted)]">
                    {d.image_tag ? shortSha(d.image_tag) : '—'}
                  </code>
                  <div className="text-[var(--text-muted)]">
                    {d.running_profile_id ||
                      (d.provisioned ? '—' : 'unprovisioned')}
                    {d.running_source === 'agent' ? ' · via agent' : ''}
                  </div>
                </div>
              )
            },
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
              if (d.update_available && d.latest_release) {
                const latest = d.latest_release
                return (
                  <Link
                    to={`/releases?focus=${latest.id}`}
                    className="block max-w-[16rem] text-xs hover:opacity-90"
                    onClick={(e) => e.stopPropagation()}
                  >
                    <StatusBadge label="Update available" tone="warn" pulse />
                    <div className="mt-1 font-medium text-[var(--status-warn-fg)]">
                      → {releaseVersion(latest)}
                    </div>
                    <div
                      className="mt-0.5 truncate text-[var(--text-muted)]"
                      title={releaseSummary(latest)}
                    >
                      {releaseSummary(latest)}
                    </div>
                    <div className="mt-0.5 text-[10px] text-[var(--accent)] underline underline-offset-2">
                      View release notes
                    </div>
                  </Link>
                )
              }
              if (d.needs_build) {
                return (
                  <div className="max-w-[16rem] text-xs">
                    <StatusBadge label="Needs CI build" tone="info" />
                    <div className="mt-1 font-medium text-[var(--text-secondary)]">
                      tip {shortSha(d.branch_tip_sha)}
                    </div>
                    <div
                      className="mt-0.5 truncate text-[var(--text-muted)]"
                      title={d.branch_tip_message || undefined}
                    >
                      {d.branch_tip_message || 'branch tip not built yet'}
                    </div>
                  </div>
                )
              }
              if (d.drift?.any) {
                return <StatusBadge label="drift" tone="warn" />
              }
              return <StatusBadge label="Up to date" tone="good" />
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
