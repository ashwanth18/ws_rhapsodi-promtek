import { useCallback, useEffect, useMemo, useState } from 'react'
import type { FormEvent } from 'react'
import { Link, useParams } from 'react-router-dom'
import { ArrowLeft, ChevronLeft, ChevronRight, ExternalLink } from 'lucide-react'
import {
  api,
  formatDuration,
  releaseSummary,
  releaseVersion,
  shortSha,
  type Branch,
  type CustomDashboard,
  type Deployment,
  type Device,
  type Profile,
  type Release,
} from '../lib/api'
import LogViewer from '../components/LogViewer'
import ContainerLogPanel from '../components/ContainerLogPanel'
import DeviceMetricsCharts from '../components/DeviceMetricsCharts'
import DashboardView from '../components/dashboard/DashboardView'
import {
  Button,
  ConfirmDialog,
  DataTable,
  MetricCard,
  SectionHeader,
  StatusBadge,
  deployTone,
} from '../components/ui'

function agentTone(status?: string | null): 'good' | 'bad' | 'warn' | 'info' | 'neutral' {
  if (!status) return 'neutral'
  if (status === 'success' || status === 'converged') return 'good'
  if (status === 'applying') return 'info'
  if (status === 'rolled_back') return 'warn'
  if (status === 'failed') return 'bad'
  return 'neutral'
}

export default function DeviceDetailPage() {
  const { deviceId = '' } = useParams()
  const [device, setDevice] = useState<Device | null>(null)
  const [profiles, setProfiles] = useState<Profile[]>([])
  const [robotTypes, setRobotTypes] = useState<string[]>([])
  const [branches, setBranches] = useState<Branch[]>([])
  const [releases, setReleases] = useState<Release[]>([])
  const [error, setError] = useState<string | null>(null)
  const [releaseId, setReleaseId] = useState<number | ''>('')
  const [robotType, setRobotType] = useState('')
  const [siteId, setSiteId] = useState('site-1')
  const [profileId, setProfileId] = useState('prod-niryo')
  const [trackedBranch, setTrackedBranch] = useState('main')
  const [busy, setBusy] = useState(false)
  const [confirm, setConfirm] = useState<'provision' | 'deploy' | null>(null)
  const [activeDeploymentId, setActiveDeploymentId] = useState<number | null>(null)
  const [history, setHistory] = useState<Deployment[]>([])
  const [historyPage, setHistoryPage] = useState(1)
  const [historyPages, setHistoryPages] = useState(1)
  const [historyTotal, setHistoryTotal] = useState(0)
  const [deviceDashboards, setDeviceDashboards] = useState<CustomDashboard[]>(
    [],
  )
  const [selectedDashId, setSelectedDashId] = useState<number | null>(null)

  const loadHistory = useCallback(
    async (page = historyPage) => {
      if (!deviceId) return
      try {
        const payload = await api.listDeployments({
          device_id: deviceId,
          page,
          limit: 20,
        })
        setHistory(payload.deployments)
        setHistoryPages(payload.pages)
        setHistoryTotal(payload.total)
        if (payload.page !== page) setHistoryPage(payload.page)
      } catch {
        /* device load surfaces primary errors */
      }
    },
    [deviceId, historyPage],
  )

  const load = useCallback(async (opts?: { syncForm?: boolean }) => {
    if (!deviceId) return
    const syncForm = opts?.syncForm ?? false
    try {
      const payload = await api.getDevice(deviceId)
      setDevice(payload.device)
      setError(null)
      void loadHistory(historyPage)
      if (syncForm) {
        const rt = payload.device.robot_type || payload.device.target?.robot_type
        if (rt) setRobotType(rt)
        if (payload.device.site_id) setSiteId(payload.device.site_id)
        if (payload.device.desired_profile_id) {
          setProfileId(payload.device.desired_profile_id)
        }
        if (payload.device.desired_branch) setTrackedBranch(payload.device.desired_branch)
        if (payload.device.target?.release_id) {
          setReleaseId(payload.device.target.release_id)
        } else if (payload.device.desired_release?.id) {
          setReleaseId(payload.device.desired_release.id)
        }
      }
      const agentDone = ['converged', 'success', 'failed', 'rolled_back'].includes(
        String(payload.device.agent_status || ''),
      )
      const running = payload.device.deployments?.find((d) => d.status === 'running')
      // Prefer a truly running job only while the agent is still applying.
      // After converge, stale "running" reconcile rows must not stick in the UI.
      if (running && !agentDone) {
        setActiveDeploymentId(running.id)
      } else if (payload.device.last_deployment) {
        const last = payload.device.last_deployment
        setActiveDeploymentId((prev) => {
          if (prev == null) return last.id
          // If we were watching a stale running job, jump to latest finished.
          const stillRunning = payload.device.deployments?.find(
            (d) => d.id === prev && d.status === 'running',
          )
          if (stillRunning && agentDone) return last.id
          return prev
        })
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err))
    }
  }, [deviceId, historyPage, loadHistory])

  const onJobStatus = useCallback(
    (status: string) => {
      // Only refresh device metadata when a job finishes — never while streaming.
      if (status === 'running' || status === 'connecting') return
      void load({ syncForm: false })
    },
    [load],
  )

  useEffect(() => {
    void loadHistory(historyPage)
  }, [historyPage, loadHistory])

  useEffect(() => {
    void load({ syncForm: true })
    api.listRobotTypes().then((p) => setRobotTypes(p.robot_types)).catch(() => undefined)
    api.listBranches().then((p) => setBranches(p.branches)).catch(() => undefined)
    // Poll device status quietly; do not reset form fields or the active log job.
    const id = window.setInterval(() => void load({ syncForm: false }), 15000)
    return () => window.clearInterval(id)
  }, [load])

  useEffect(() => {
    const deviceClass = device?.device_class || device?.target?.device_class || undefined
    api
      .listProfiles({
        robotType: robotType || undefined,
        deviceClass: deviceClass || undefined,
      })
      .then((p) => setProfiles(p.profiles))
      .catch(() => setProfiles([]))
  }, [robotType, device?.device_class, device?.target?.device_class])

  const loadReleases = useCallback(
    async (sync = true) => {
      if (!deviceId) return
      try {
        // Only releases built for this device's platform (pi5/jetson → arm64).
        const payload = await api.listReleases({
          status: 'success',
          sync,
          device_id: deviceId,
        })
        setReleases(payload.releases)
        setReleaseId((current) => {
          if (current && payload.releases.some((r) => r.id === current)) return current
          const preferred =
            payload.releases.find((r) => r.git_sha !== 'deadbee') || payload.releases[0]
          return preferred ? preferred.id : ''
        })
      } catch {
        setReleases([])
      }
    },
    [deviceId],
  )

  useEffect(() => {
    void loadReleases(true)
  }, [loadReleases, device?.platform, device?.device_class])

  useEffect(() => {
    api
      .listDashboards('device_template')
      .then((payload) => {
        setDeviceDashboards(payload.dashboards)
        setSelectedDashId((current) => {
          if (current && payload.dashboards.some((d) => d.id === current)) {
            return current
          }
          return payload.dashboards[0]?.id ?? null
        })
      })
      .catch(() => setDeviceDashboards([]))
  }, [deviceId])

  const selectedRelease = useMemo(
    () => releases.find((r) => r.id === releaseId) || device?.desired_release || null,
    [releases, releaseId, device],
  )

  const robotTypeLocked = Boolean(device?.robot_type || device?.target?.robot_type)

  const saveTarget = async () => {
    if (!device) return
    setBusy(true)
    try {
      await api.updateTarget(device.id, {
        tracked_branch: trackedBranch,
        profile_id: profileId,
        release_id: typeof releaseId === 'number' ? releaseId : undefined,
        site_id: siteId,
        ...(robotTypeLocked ? {} : { robot_type: robotType }),
      })
      await load()
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err))
    } finally {
      setBusy(false)
    }
  }

  const runAction = async () => {
    if (!device || !confirm) return
    setBusy(true)
    setError(null)
    try {
      let deployment: Deployment
      if (confirm === 'provision') {
        if (typeof releaseId !== 'number') throw new Error('Select a verified release')
        if (!robotType) throw new Error('Select a robot type')
        const resp = await api.provision(device.id, {
          robot_type: robotType,
          site_id: siteId,
          release_id: releaseId,
          profile_id: profileId,
          tracked_branch: trackedBranch,
        })
        deployment = resp.deployment
      } else {
        if (typeof releaseId !== 'number') throw new Error('Select a verified release')
        const resp = await api.deploy(device.id, {
          release_id: releaseId,
          profile_id: profileId,
          tracked_branch: trackedBranch,
        })
        deployment = resp.deployment
      }
      setActiveDeploymentId(deployment.id)
      setConfirm(null)
      await load()
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err))
    } finally {
      setBusy(false)
    }
  }

  const onSubmit = (e: FormEvent, kind: typeof confirm) => {
    e.preventDefault()
    if (typeof releaseId !== 'number') {
      setError('Select a verified Release from the list (built by CI)')
      return
    }
    if (kind === 'provision' && !robotType) {
      setError('Select a robot type')
      return
    }
    setConfirm(kind)
  }

  if (!device && !error) {
    return <div className="text-sm text-[var(--text-muted)]">Loading device…</div>
  }

  return (
    <div>
      <Link
        to="/"
        className="mb-4 inline-flex items-center gap-1 text-sm text-[var(--text-muted)] hover:text-[var(--accent)]"
      >
        <ArrowLeft className="h-4 w-4" /> Back to fleet
      </Link>

      <SectionHeader
        title={device?.hostname || deviceId}
        description={`${device?.ip || ''} · role ${device?.role || 'robot'} · fleet control plane for this cell`}
        action={
          <div className="flex flex-wrap items-center gap-3">
            {device?.dashboard_url ? (
              <a
                href={device.dashboard_url}
                target="_blank"
                rel="noreferrer"
                className="inline-flex items-center gap-1.5 rounded-[var(--radius-sm)] border border-[var(--accent)]/40 bg-[var(--accent)]/10 px-3 py-1.5 text-sm font-medium text-[var(--accent)] hover:bg-[var(--accent)]/20"
              >
                Open robot dashboard <ExternalLink className="h-3.5 w-3.5" />
              </a>
            ) : null}
            {device?.api_url ? (
              <a
                href={`${device.api_url}/host_info`}
                target="_blank"
                rel="noreferrer"
                className="inline-flex items-center gap-1 text-sm text-[var(--text-muted)] hover:text-[var(--accent)]"
                title={device.api_url}
              >
                API <ExternalLink className="h-3.5 w-3.5" />
              </a>
            ) : null}
            {device?.grafana_url ? (
              <a
                href={device.grafana_url}
                target="_blank"
                rel="noreferrer"
                className="inline-flex items-center gap-1 text-sm text-[var(--text-muted)] hover:text-[var(--accent)]"
              >
                Grafana <ExternalLink className="h-3.5 w-3.5" />
              </a>
            ) : null}
          </div>
        }
      />

      {error ? (
        <div className="mb-4 rounded-[var(--radius-md)] border border-[var(--status-bad-fg)]/40 bg-[var(--status-bad-bg)] px-4 py-3 text-sm text-[var(--status-bad-fg)]">
          {error}
        </div>
      ) : null}

      {device?.update_available && device.latest_release ? (
        <div className="mb-4 rounded-[var(--radius-md)] border border-[var(--status-warn-fg)]/40 bg-[var(--status-warn-bg)] px-4 py-3 text-sm text-[var(--status-warn-fg)]">
          <div className="font-medium">Update available</div>
          <div className="mt-1">
            New version{' '}
            <strong>{releaseVersion(device.latest_release)}</strong>
            {device.image_tag ? (
              <>
                {' '}
                (running {shortSha(device.image_tag)})
              </>
            ) : null}
          </div>
          <div className="mt-1 text-[var(--text-secondary)]">
            What&apos;s in it: {releaseSummary(device.latest_release)}
            {device.latest_release.reported_at
              ? ` · built ${new Date(device.latest_release.reported_at).toLocaleString()}`
              : ''}
          </div>
          <div className="mt-2 text-xs">
            Select{' '}
            <strong>{releaseVersion(device.latest_release)}</strong> in Release
            below, then Deploy. Tracked branch:{' '}
            <code>{trackedBranch || 'main'}</code>
          </div>
        </div>
      ) : null}

      {device?.needs_build && !device?.update_available ? (
        <div className="mb-4 rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--surface-1)] px-4 py-3 text-sm text-[var(--text-secondary)]">
          Branch tip <code>{device.branch_tip_sha}</code>
          {device.branch_tip_message ? ` (“${device.branch_tip_message}”)` : ''}{' '}
          is ahead of the running image, but there is no CI Release for it yet.
          Build it on the{' '}
          <Link
            className="text-[var(--accent)] hover:underline"
            to={`/releases?branch=${encodeURIComponent(trackedBranch || 'main')}`}
          >
            Releases
          </Link>{' '}
          page, then Refresh list here.
        </div>
      ) : null}

      <div className="mb-6 grid gap-3 sm:grid-cols-2 lg:grid-cols-4">
        <MetricCard
          label="Alive"
          value={
            <StatusBadge
              label={device?.alive ? 'alive' : 'down'}
              tone={device?.alive ? 'good' : 'bad'}
              pulse={Boolean(device?.alive)}
            />
          }
        />
        <MetricCard
          label="Agent"
          value={
            <StatusBadge
              label={device?.agent_status || 'unknown'}
              tone={agentTone(device?.agent_status)}
              pulse={device?.agent_status === 'applying'}
            />
          }
        />
        <MetricCard
          label="Running version"
          value={
            <div>
              <div className="text-base font-medium text-[var(--accent)]">
                {device?.desired_release
                  ? releaseVersion(device.desired_release)
                  : device?.image_tag
                    ? shortSha(device.image_tag)
                    : '—'}
              </div>
              <code className="text-xs text-[var(--text-muted)]">
                {device?.image_tag || '—'}
              </code>
            </div>
          }
          help="Release #N is the console version; full git sha is the image tag."
        />
        <MetricCard
          label="Device / platform"
          value={
            <div className="text-base">
              <span className="font-medium">
                {device?.device_class || device?.target?.device_class || '—'}
              </span>
              <div className="mt-0.5 font-mono text-xs text-[var(--text-muted)]">
                {device?.platform || device?.target?.platform || 'awaiting agent'}
              </div>
              <div className="mt-0.5 font-mono text-[10px] text-[var(--text-muted)]">
                {device?.target?.compose_file || 'compose unresolved'}
              </div>
            </div>
          }
          help="device_class selects compose; Releases must match platform and class allow-list."
        />
      </div>

      {device?.agent_message ? (
        <p className="mb-4 text-xs text-[var(--text-muted)]">
          Agent: {device.agent_message}
          {device.agent_reported_at
            ? ` · ${new Date(device.agent_reported_at).toLocaleString()}`
            : ''}
        </p>
      ) : null}

      <div className="mb-8 grid gap-6 lg:grid-cols-2">
        <div className="rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--card-surface)] p-4">
          <h3 className="font-display text-base font-semibold">Desired state</h3>
          <p className="mt-1 text-sm text-[var(--text-muted)]">
            Only Releases with Hub images built for this device&apos;s platform
            appear here (e.g. Pi5 → linux/arm64). Deploy writes desired state;
            the on-device agent pulls and converges.
          </p>

          <div className="mt-4 space-y-3">
            <label className="block text-xs text-[var(--text-muted)]">
              Tracked branch
              <select
                className="mt-1 w-full rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-1)] px-3 py-2 text-sm"
                value={trackedBranch}
                onChange={(e) => setTrackedBranch(e.target.value)}
              >
                {branches.length === 0 ? (
                  <option value={trackedBranch}>{trackedBranch}</option>
                ) : (
                  branches.map((b) => (
                    <option key={b.name} value={b.name}>
                      {b.name}
                      {b.sha ? ` (${b.sha})` : ''}
                    </option>
                  ))
                )}
              </select>
            </label>

            <label className="block text-xs text-[var(--text-muted)]">
              Robot type
              <select
                className="mt-1 w-full rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-1)] px-3 py-2 text-sm disabled:opacity-60"
                value={robotType}
                disabled={robotTypeLocked && Boolean(device?.provisioned)}
                onChange={(e) => setRobotType(e.target.value)}
              >
                <option value="">Select…</option>
                {robotTypes.map((t) => (
                  <option key={t} value={t}>
                    {t}
                  </option>
                ))}
              </select>
              {robotTypeLocked && device?.provisioned ? (
                <span className="mt-1 block text-[10px] text-[var(--text-muted)]">
                  Locked after first provision
                </span>
              ) : null}
            </label>

            <label className="block text-xs text-[var(--text-muted)]">
              Site ID
              <input
                className="mt-1 w-full rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-1)] px-3 py-2 text-sm"
                value={siteId}
                onChange={(e) => setSiteId(e.target.value)}
              />
            </label>

            <label className="block text-xs text-[var(--text-muted)]">
              Profile
              <select
                className="mt-1 w-full rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-1)] px-3 py-2 text-sm"
                value={profileId}
                onChange={(e) => setProfileId(e.target.value)}
              >
                {profiles.map((p) => (
                  <option key={p.id} value={p.id}>
                    {p.id} — {p.description}
                  </option>
                ))}
              </select>
            </label>

            <label className="block text-xs text-[var(--text-muted)]">
              <span className="flex items-center justify-between gap-2">
                Release (on Docker Hub)
                <button
                  type="button"
                  className="text-[11px] text-[var(--accent)] hover:underline"
                  onClick={() => void loadReleases(true)}
                >
                  Refresh list
                </button>
              </span>
              <select
                className="mt-1 w-full rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-1)] px-3 py-2 text-sm"
                value={releaseId === '' ? '' : String(releaseId)}
                onChange={(e) =>
                  setReleaseId(e.target.value ? Number(e.target.value) : '')
                }
              >
                <option value="">Select a release…</option>
                {releases.map((r) => {
                  const marks: string[] = []
                  if (r.demo || r.git_sha === 'deadbee') marks.push('demo — skip')
                  if (r.status && r.status !== 'success') marks.push(r.status)
                  if (device?.image_tag && r.git_sha === device.image_tag) {
                    marks.push('running')
                  }
                  if (
                    device?.desired_image_tag &&
                    r.git_sha === device.desired_image_tag
                  ) {
                    marks.push('desired')
                  }
                  if (
                    device?.update_available &&
                    device.latest_release &&
                    r.id === device.latest_release.id
                  ) {
                    marks.push('NEW UPDATE')
                  }
                  const plats = (r.platforms || [])
                    .map((p) => p.replace('linux/', ''))
                    .join('+')
                  if (plats) marks.push(plats)
                  const subject = r.subject ? ` — ${r.subject}` : ''
                  const when = r.reported_at
                    ? ` · ${new Date(r.reported_at).toLocaleString()}`
                    : ''
                  const dur = formatDuration(r.duration_seconds)
                  const build = dur ? ` · build ${dur}` : ''
                  const mark = marks.length ? ` [${marks.join(', ')}]` : ''
                  return (
                    <option key={r.id} value={r.id}>
                      {releaseVersion(r)} · {r.branch}
                      {subject}
                      {when}
                      {build}
                      {mark}
                    </option>
                  )
                })}
              </select>
              {selectedRelease ? (
                <span className="mt-1 block text-[11px] text-[var(--text-secondary)]">
                  <span className="font-medium">{releaseVersion(selectedRelease)}</span>
                  {' — '}
                  {releaseSummary(selectedRelease)}
                  {selectedRelease.duration_seconds != null
                    ? ` · CI build ${formatDuration(selectedRelease.duration_seconds)}`
                    : ''}
                </span>
              ) : null}
              {selectedRelease?.workflow_run_url ? (
                <a
                  href={selectedRelease.workflow_run_url}
                  target="_blank"
                  rel="noreferrer"
                  className="mt-1 inline-flex items-center gap-1 text-[11px] text-[var(--accent)]"
                >
                  CI run <ExternalLink className="h-3 w-3" />
                </a>
              ) : null}
              <span className="mt-1 block text-[10px] text-[var(--text-muted)]">
                Local commits (e.g. fleet-console work) do not appear until CI
                builds them into a Release / deploy-* tag.
              </span>
              {releases.length === 0 ? (
                <span className="mt-1 block text-[10px] text-[var(--status-warn-fg)]">
                  No releases yet — Refresh list, or{' '}
                  <Link
                    className="text-[var(--accent)] hover:underline"
                    to={`/releases?branch=${encodeURIComponent(trackedBranch || 'main')}`}
                  >
                    Build CI
                  </Link>
                  .
                </span>
              ) : null}
            </label>

            <div className="flex flex-wrap gap-2 pt-1">
              <Button type="button" variant="outline" onClick={saveTarget} disabled={busy}>
                Save target
              </Button>
              {!device?.provisioned ? (
                <Button type="button" onClick={(e) => onSubmit(e, 'provision')}>
                  Flash install
                </Button>
              ) : (
                <Button type="button" onClick={(e) => onSubmit(e, 'deploy')}>
                  Deploy (set desired)
                </Button>
              )}
              <Link
                to={`/releases?branch=${encodeURIComponent(trackedBranch || 'main')}`}
                className="inline-flex items-center justify-center gap-2 rounded-[var(--radius-sm)] border border-[var(--border)] px-3.5 py-2 text-sm font-medium text-[var(--text-secondary)] transition hover:bg-white/5"
              >
                Build CI →
              </Link>
              {device?.provisioned ? (
                <Button
                  type="button"
                  variant="ghost"
                  onClick={(e) => onSubmit(e, 'provision')}
                >
                  Re-provision
                </Button>
              ) : null}
            </div>
          </div>
        </div>

        <div>
          <h3 className="mb-2 font-display text-base font-semibold">
            Job console
          </h3>
          <p className="mb-2 text-xs text-[var(--text-muted)]">
            Deploy stays running until fleet-agent on the device applies the
            release and reports back (lines appear here as AGENT […] updates).
            Flash install still streams Ansible. If a deploy never advances,
            re-provision so the agent is installed.
          </p>
          <LogViewer
            deploymentId={activeDeploymentId}
            deployment={
              history.find((d) => d.id === activeDeploymentId) ||
              device?.deployments?.find((d) => d.id === activeDeploymentId) ||
              device?.last_deployment ||
              null
            }
            onStatus={onJobStatus}
          />
        </div>
      </div>

      <div className="mb-8 mt-8">
        <DeviceMetricsCharts deviceId={deviceId} />
      </div>

      <SectionHeader
        title="Container logs"
        description="Live stdout/stderr from Promtail → Loki. Pick condor-agent, backend, ROS services, etc."
      />
      <div className="mb-8">
        <ContainerLogPanel deviceId={deviceId} />
      </div>

      <SectionHeader
        title="Device dashboards"
        description="Device-template dashboards with $device_id substituted. Create more under Dashboards."
        action={
          <Link
            to="/dashboards"
            className="text-sm text-[var(--accent)] hover:underline"
          >
            Manage dashboards
          </Link>
        }
      />
      {deviceDashboards.length === 0 ? (
        <div className="mb-8 rounded border border-dashed border-[var(--border)] px-4 py-8 text-center text-sm text-[var(--text-muted)]">
          No device-template dashboards yet. Create one with scope “Device
          template” and use <code className="text-[var(--accent)]">$device_id</code>{' '}
          in panel queries.
        </div>
      ) : (
        <div className="mb-8 space-y-3">
          <select
            className="rounded border border-[var(--border)] bg-[var(--surface-2)] px-2 py-1.5 text-sm"
            value={selectedDashId ?? ''}
            onChange={(e) =>
              setSelectedDashId(e.target.value ? Number(e.target.value) : null)
            }
          >
            {deviceDashboards.map((d) => (
              <option key={d.id} value={d.id}>
                {d.name}
              </option>
            ))}
          </select>
          {selectedDashId ? (
            <DashboardView
              dashboardId={selectedDashId}
              deviceId={deviceId}
              embed
            />
          ) : null}
        </div>
      )}

      <SectionHeader
        title="Job history"
        description="Provision, desired-state deploys, and agent reconciles. 20 per page."
      />
      <DataTable
        rows={history}
        rowKey={(d) => d.id}
        emptyMessage="No deployments yet."
        onRowClick={(d) => setActiveDeploymentId(d.id)}
        columns={[
          {
            key: 'id',
            header: 'ID',
            render: (d) => <span className="text-xs">#{d.id}</span>,
          },
          {
            key: 'action',
            header: 'Action',
            render: (d) => d.action,
          },
          {
            key: 'profile',
            header: 'Profile',
            render: (d) => d.profile_id || '—',
          },
          {
            key: 'tag',
            header: 'Release / branch',
            render: (d) => (
              <div className="text-xs">
                <code className="text-[var(--accent)]">
                  {d.release_id ? `#${d.release_id} ` : ''}
                  {d.image_tag || '—'}
                </code>
                <div className="text-[var(--text-muted)]">{d.tracked_branch || ''}</div>
              </div>
            ),
          },
          {
            key: 'status',
            header: 'Status',
            render: (d) => (
              <StatusBadge
                label={d.status}
                tone={deployTone(d.status)}
                pulse={d.status === 'running'}
              />
            ),
          },
          {
            key: 'started',
            header: 'Started',
            render: (d) => (
              <span className="text-xs text-[var(--text-muted)]">
                {d.started_at ? new Date(d.started_at).toLocaleString() : '—'}
              </span>
            ),
          },
        ]}
      />
      <div className="mt-3 flex flex-wrap items-center justify-between gap-3 text-sm text-[var(--text-muted)]">
        <div>
          {historyTotal} job{historyTotal === 1 ? '' : 's'} · page {historyPage} of{' '}
          {historyPages}
        </div>
        <div className="flex items-center gap-2">
          <Button
            variant="outline"
            size="sm"
            disabled={historyPage <= 1}
            onClick={() => setHistoryPage((p) => Math.max(1, p - 1))}
          >
            <ChevronLeft className="h-4 w-4" />
            Prev
          </Button>
          <Button
            variant="outline"
            size="sm"
            disabled={historyPage >= historyPages}
            onClick={() => setHistoryPage((p) => p + 1)}
          >
            Next
            <ChevronRight className="h-4 w-4" />
          </Button>
        </div>
      </div>

      <ConfirmDialog
        open={confirm !== null}
        title={
          confirm === 'provision'
            ? 'Flash install this device?'
            : 'Set desired state (agent will pull)?'
        }
        message={
          confirm === 'provision'
            ? `ansible/provision.yml on ${deviceId}: robot_type=${robotType}, profile=${profileId}, release=#${releaseId} (${selectedRelease?.git_sha || '?'}). Installs fleet-agent.`
            : `Set desired release=#${releaseId} (${selectedRelease?.git_sha || '?'}) profile=${profileId}. Agent on ${deviceId} will pull and converge.`
        }
        confirmLabel={confirm === 'provision' ? 'Flash install' : 'Set desired'}
        loading={busy}
        onCancel={() => setConfirm(null)}
        onConfirm={runAction}
      />
    </div>
  )
}
