import { useCallback, useEffect, useState } from 'react'
import type { FormEvent } from 'react'
import { Link, useParams } from 'react-router-dom'
import { ArrowLeft, ExternalLink } from 'lucide-react'
import { api, type Deployment, type Device } from '../lib/api'
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
  const [error, setError] = useState<string | null>(null)
  const [imageTag, setImageTag] = useState('')
  const [robotType, setRobotType] = useState<'niryo' | 'jaka'>('niryo')
  const [siteId, setSiteId] = useState('site-1')
  const [tags, setTags] = useState<string[]>([])
  const [busy, setBusy] = useState(false)
  const [confirm, setConfirm] = useState<'provision' | 'deploy' | null>(null)
  const [activeDeploymentId, setActiveDeploymentId] = useState<number | null>(null)

  const load = useCallback(async () => {
    if (!deviceId) return
    try {
      const payload = await api.getDevice(deviceId)
      setDevice(payload.device)
      setError(null)
      if (payload.device.robot_type === 'jaka' || payload.device.robot_type === 'niryo') {
        setRobotType(payload.device.robot_type)
      }
      if (payload.device.site_id) setSiteId(payload.device.site_id)
      const running = payload.device.deployments?.find((d) => d.status === 'running')
      if (running) setActiveDeploymentId(running.id)
      else if (payload.device.last_deployment) {
        setActiveDeploymentId((prev) => prev ?? payload.device.last_deployment!.id)
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err))
    }
  }, [deviceId])

  useEffect(() => {
    load()
    api.imageTags().then((p) => setTags(p.image_tags)).catch(() => undefined)
    const id = window.setInterval(load, 10000)
    return () => window.clearInterval(id)
  }, [load])

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
        })
        deployment = resp.deployment
      } else {
        const resp = await api.deploy(device.id, { image_tag: imageTag })
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

  const onSubmit = (e: FormEvent, kind: 'provision' | 'deploy') => {
    e.preventDefault()
    if (!imageTag.trim()) {
      setError('image_tag is required')
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
          label="Active run"
          value={
            <StatusBadge
              label={device?.active ? 'running' : 'idle'}
              tone={device?.active ? 'info' : 'neutral'}
            />
          }
        />
        <MetricCard label="Robot type" value={device?.robot_type || '—'} />
        <MetricCard
          label="Version"
          value={<code className="text-base text-[var(--accent)]">{device?.image_tag || '—'}</code>}
        />
      </div>

      <div className="mb-6 grid gap-3 sm:grid-cols-3">
        <MetricCard label="CPU" value={device?.metrics?.cpu_pct ?? '—'} unit="%" />
        <MetricCard label="Memory" value={device?.metrics?.mem_pct ?? '—'} unit="%" />
        <MetricCard label="Disk" value={device?.metrics?.disk_pct ?? '—'} unit="%" />
      </div>

      <div className="mb-8 grid gap-6 lg:grid-cols-2">
        <div className="rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--card-surface)] p-4">
          <h3 className="font-display text-base font-semibold">
            {device?.provisioned ? 'Deploy update' : 'Flash install'}
          </h3>
          <p className="mt-1 text-sm text-[var(--text-muted)]">
            {device?.provisioned
              ? 'Pin a new image tag and roll this device via Ansible deploy.yml.'
              : 'First install: pick robot type, site, and image tag. Uses ansible/provision.yml.'}
          </p>

          <form className="mt-4 space-y-3">
            {!device?.provisioned ? (
              <>
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
              </>
            ) : null}

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
              {!device?.provisioned ? (
                <Button type="button" onClick={(e) => onSubmit(e as unknown as FormEvent, 'provision')}>
                  Flash install
                </Button>
              ) : (
                <Button type="button" onClick={(e) => onSubmit(e as unknown as FormEvent, 'deploy')}>
                  Deploy update
                </Button>
              )}
              {device?.provisioned ? (
                <Button
                  type="button"
                  variant="outline"
                  onClick={(e) => onSubmit(e as unknown as FormEvent, 'provision')}
                >
                  Re-provision
                </Button>
              ) : null}
            </div>
          </form>
        </div>

        <div>
          <h3 className="mb-2 font-display text-base font-semibold">Live deploy logs</h3>
          <LogViewer
            deploymentId={activeDeploymentId}
            onStatus={() => {
              load()
            }}
          />
        </div>
      </div>

      <SectionHeader title="Deployment history" description="Recent provision/deploy jobs for this device." />
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
            key: 'tag',
            header: 'Tag',
            render: (d) => <code className="text-xs text-[var(--accent)]">{d.image_tag}</code>,
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
        title={confirm === 'provision' ? 'Flash install this device?' : 'Deploy update?'}
        message={
          confirm === 'provision'
            ? `This will run ansible/provision.yml on ${deviceId} with robot_type=${robotType}, site_id=${siteId}, image_tag=${imageTag}. Jaka on arm64 will be rejected.`
            : `This will run ansible/deploy.yml on ${deviceId} with image_tag=${imageTag}. Health-check failure triggers automatic rollback.`
        }
        confirmLabel={confirm === 'provision' ? 'Flash install' : 'Deploy'}
        loading={busy}
        onCancel={() => setConfirm(null)}
        onConfirm={runAction}
      />
    </div>
  )
}
