import { useCallback, useEffect, useState } from 'react'
import type { FormEvent } from 'react'
import { Link, useParams } from 'react-router-dom'
import { ArrowLeft, ExternalLink } from 'lucide-react'
import { api, type Deployment, type Device, type Profile } from '../lib/api'
import LogViewer from '../components/LogViewer'
import {
  Button,
  ConfirmDialog,
  DataTable,
  MetricCard,
  SectionHeader,
  StatusBadge,
  deployTone,
} from '../components/ui'

export default function DeviceDetailPage() {
  const { deviceId = '' } = useParams()
  const [device, setDevice] = useState<Device | null>(null)
  const [profiles, setProfiles] = useState<Profile[]>([])
  const [error, setError] = useState<string | null>(null)
  const [imageTag, setImageTag] = useState('')
  const [robotType, setRobotType] = useState<'niryo' | 'jaka'>('niryo')
  const [siteId, setSiteId] = useState('site-1')
  const [profileId, setProfileId] = useState('prod-niryo')
  const [trackedBranch, setTrackedBranch] = useState('main')
  const [tags, setTags] = useState<string[]>([])
  const [busy, setBusy] = useState(false)
  const [confirm, setConfirm] = useState<
    'provision' | 'deploy' | 'deploy-latest' | 'build-deploy' | null
  >(null)
  const [activeDeploymentId, setActiveDeploymentId] = useState<number | null>(null)

  const load = useCallback(async () => {
    if (!deviceId) return
    try {
      const payload = await api.getDevice(deviceId)
      setDevice(payload.device)
      setError(null)
      const rt = payload.device.robot_type
      if (rt === 'jaka' || rt === 'niryo') setRobotType(rt)
      if (payload.device.site_id) setSiteId(payload.device.site_id)
      if (payload.device.desired_profile_id) setProfileId(payload.device.desired_profile_id)
      if (payload.device.desired_branch) setTrackedBranch(payload.device.desired_branch)
      if (payload.device.latest_sha && !imageTag) setImageTag(payload.device.latest_sha)
      const running = payload.device.deployments?.find((d) => d.status === 'running')
      if (running) setActiveDeploymentId(running.id)
      else if (payload.device.last_deployment) {
        setActiveDeploymentId((prev) => prev ?? payload.device.last_deployment!.id)
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err))
    }
  }, [deviceId, imageTag])

  useEffect(() => {
    load()
    api.imageTags().then((p) => setTags(p.image_tags)).catch(() => undefined)
    const id = window.setInterval(load, 10000)
    return () => window.clearInterval(id)
  }, [load])

  useEffect(() => {
    api
      .listProfiles(robotType)
      .then((p) => setProfiles(p.profiles))
      .catch(() => setProfiles([]))
  }, [robotType])

  const saveTarget = async () => {
    if (!device) return
    setBusy(true)
    try {
      await api.updateTarget(device.id, {
        tracked_branch: trackedBranch,
        profile_id: profileId,
        robot_type: robotType,
        site_id: siteId,
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
        const resp = await api.provision(device.id, {
          robot_type: robotType,
          site_id: siteId,
          image_tag: imageTag,
          profile_id: profileId,
          tracked_branch: trackedBranch,
        })
        deployment = resp.deployment
      } else if (confirm === 'build-deploy') {
        const resp = await api.build(device.id, {
          branch: trackedBranch,
          deploy_after: true,
          profile_id: profileId,
        })
        deployment = resp.deployment
      } else {
        const tag =
          confirm === 'deploy-latest'
            ? device.latest_sha || imageTag
            : imageTag
        const resp = await api.deploy(device.id, {
          image_tag: tag,
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
    if (kind === 'build-deploy') {
      setConfirm(kind)
      return
    }
    const tag = kind === 'deploy-latest' ? device?.latest_sha || imageTag : imageTag
    if (!tag?.trim()) {
      setError('image_tag is required (or use Build & deploy)')
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
        description={`${device?.ip || ''} · role ${device?.role || 'robot'}`}
        action={
          device?.grafana_url ? (
            <a
              href={device.grafana_url}
              target="_blank"
              rel="noreferrer"
              className="inline-flex items-center gap-1 text-sm text-[var(--accent)]"
            >
              Grafana Pi Overview <ExternalLink className="h-3.5 w-3.5" />
            </a>
          ) : null
        }
      />

      {error ? (
        <div className="mb-4 rounded-[var(--radius-md)] border border-[var(--status-bad-fg)]/40 bg-[var(--status-bad-bg)] px-4 py-3 text-sm text-[var(--status-bad-fg)]">
          {error}
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
          label="Update"
          value={
            device?.update_available ? (
              <StatusBadge label={`→ ${device.latest_sha}`} tone="warn" pulse />
            ) : (
              <StatusBadge label="current" tone="good" />
            )
          }
        />
        <MetricCard
          label="Running version"
          value={<code className="text-base text-[var(--accent)]">{device?.image_tag || '—'}</code>}
        />
        <MetricCard label="Running profile" value={device?.running_profile_id || '—'} />
      </div>

      <div className="mb-8 grid gap-6 lg:grid-cols-2">
        <div className="rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--card-surface)] p-4">
          <h3 className="font-display text-base font-semibold">Desired state</h3>
          <p className="mt-1 text-sm text-[var(--text-muted)]">
            Tracked branch (updates) and profile (runtime layout/mode) are independent.
          </p>

          <div className="mt-4 space-y-3">
            <label className="block text-xs text-[var(--text-muted)]">
              Tracked branch
              <input
                className="mt-1 w-full rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-1)] px-3 py-2 text-sm"
                value={trackedBranch}
                onChange={(e) => setTrackedBranch(e.target.value)}
                placeholder="main"
              />
            </label>

            <label className="block text-xs text-[var(--text-muted)]">
              Robot type
              <select
                className="mt-1 w-full rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-1)] px-3 py-2 text-sm"
                value={robotType}
                onChange={(e) => setRobotType(e.target.value as 'niryo' | 'jaka')}
              >
                <option value="niryo">niryo</option>
                <option value="jaka">jaka (amd64 only)</option>
              </select>
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
              Image tag (git short SHA)
              <input
                list="image-tags"
                className="mt-1 w-full rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-1)] px-3 py-2 text-sm"
                value={imageTag}
                onChange={(e) => setImageTag(e.target.value)}
                placeholder="e.g. ccce58f"
              />
              <datalist id="image-tags">
                {tags.map((t) => (
                  <option key={t} value={t} />
                ))}
              </datalist>
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
                <>
                  <Button type="button" onClick={(e) => onSubmit(e, 'deploy')}>
                    Deploy
                  </Button>
                  {device?.update_available ? (
                    <Button
                      type="button"
                      variant="outline"
                      onClick={(e) => onSubmit(e, 'deploy-latest')}
                    >
                      Deploy latest ({device.latest_sha})
                    </Button>
                  ) : null}
                  <Button
                    type="button"
                    variant="outline"
                    onClick={(e) => onSubmit(e, 'build-deploy')}
                  >
                    Build & deploy branch
                  </Button>
                </>
              )}
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
          <h3 className="mb-2 font-display text-base font-semibold">Live job logs</h3>
          <LogViewer
            deploymentId={activeDeploymentId}
            onStatus={() => {
              load()
            }}
          />
        </div>
      </div>

      <SectionHeader title="Job history" description="Provision / deploy / build jobs for this device." />
      <DataTable
        rows={device?.deployments || []}
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
            header: 'Tag / branch',
            render: (d) => (
              <div className="text-xs">
                <code className="text-[var(--accent)]">{d.image_tag || '—'}</code>
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

      <ConfirmDialog
        open={confirm !== null}
        title={
          confirm === 'provision'
            ? 'Flash install this device?'
            : confirm === 'build-deploy'
              ? 'Build branch and deploy?'
              : confirm === 'deploy-latest'
                ? 'Deploy latest from tracked branch?'
                : 'Deploy update?'
        }
        message={
          confirm === 'build-deploy'
            ? `This will build images for branch ${trackedBranch}, publish the deploy bundle, then deploy to ${deviceId} with profile=${profileId}. Builds can take a long time.`
            : confirm === 'provision'
              ? `ansible/provision.yml on ${deviceId}: robot_type=${robotType}, profile=${profileId}, image_tag=${imageTag}.`
              : `ansible/deploy.yml on ${deviceId}: profile=${profileId}, image_tag=${confirm === 'deploy-latest' ? device?.latest_sha : imageTag}.`
        }
        confirmLabel={
          confirm === 'build-deploy'
            ? 'Build & deploy'
            : confirm === 'provision'
              ? 'Flash install'
              : 'Deploy'
        }
        loading={busy}
        onCancel={() => setConfirm(null)}
        onConfirm={runAction}
      />
    </div>
  )
}
