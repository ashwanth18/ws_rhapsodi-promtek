import { useCallback, useEffect, useMemo, useState } from 'react'
import { Link, useSearchParams } from 'react-router-dom'
import { ExternalLink, RefreshCw, Rocket } from 'lucide-react'
import {
  api,
  formatDuration,
  releaseSummary,
  releaseVersion,
  shortSha,
  type Branch,
  type BranchTip,
  type Device,
  type Release,
} from '../lib/api'
import {
  Button,
  ConfirmDialog,
  DataTable,
  SectionHeader,
  StatusBadge,
  deployTone,
} from '../components/ui'

type WorkflowRun = {
  id?: number
  status?: string
  conclusion?: string | null
  html_url?: string
  head_branch?: string
  head_sha?: string
  created_at?: string
  updated_at?: string
}

function runTone(
  status?: string | null,
  conclusion?: string | null,
): 'good' | 'warn' | 'bad' | 'info' | 'neutral' {
  if (status === 'in_progress' || status === 'queued') return 'info'
  if (conclusion === 'success') return 'good'
  if (conclusion === 'failure' || conclusion === 'timed_out') return 'bad'
  if (conclusion === 'cancelled') return 'warn'
  return deployTone(status)
}

function devicePlatform(d: Device): string | null {
  if (d.platform) return d.platform
  if (d.target?.platform) return d.target.platform
  const cls = d.device_class || d.target?.device_class
  if (cls === 'pi5' || cls === 'jetson') return 'linux/arm64'
  if (cls === 'x86') return 'linux/amd64'
  const blob = `${d.id} ${d.hostname || ''}`.toLowerCase()
  if (/(pi5|raspberry|rpi|jetson|orin)/.test(blob)) return 'linux/arm64'
  return null
}

/** Human labels for what CI actually built (OCI platforms), not form-factor guesses. */
function platformLabels(platforms: string[] | undefined): string[] {
  const plats = platforms || []
  const out: string[] = []
  if (plats.some((p) => p.includes('arm64'))) out.push('arm64')
  if (plats.some((p) => p.includes('amd64'))) out.push('amd64')
  if (plats.some((p) => p.includes('arm/v7') || p.endsWith('/arm'))) out.push('arm32')
  return out
}

function matchingDevices(release: Release, devices: Device[]): Device[] {
  const plats = release.platforms || []
  if (!plats.length) {
    // Legacy release with unknown platforms — do not invent targets.
    return []
  }
  const classes = release.device_classes || []
  return devices.filter((d) => {
    const plat = devicePlatform(d)
    if (!plat || !plats.includes(plat)) return false
    if (!classes.length) return true
    const cls = d.device_class || d.target?.device_class
    return cls ? classes.includes(cls) : false
  })
}

export default function ReleasesPage() {
  const [searchParams, setSearchParams] = useSearchParams()
  const [branches, setBranches] = useState<Branch[]>([])
  const [branch, setBranch] = useState(searchParams.get('branch') || 'main')
  const [releases, setReleases] = useState<Release[]>([])
  const [devices, setDevices] = useState<Device[]>([])
  const [runs, setRuns] = useState<WorkflowRun[]>([])
  const [error, setError] = useState<string | null>(null)
  const [info, setInfo] = useState<string | null>(null)
  const [loading, setLoading] = useState(true)
  const [busy, setBusy] = useState(false)
  const [confirmBuild, setConfirmBuild] = useState(false)
  const [branchTip, setBranchTip] = useState<BranchTip | null>(null)

  /** Deep-link from Devices “Update available”: ?focus=<releaseId|sha> */
  const focusKey = (searchParams.get('focus') || '').trim()

  const load = useCallback(async () => {
    setLoading(true)
    setError(null)
    try {
      const [b, r, w, d] = await Promise.all([
        api.listBranches(),
        api.listReleases({ status: 'success', sync: true }),
        api.listWorkflowRuns({ branch: branch || undefined, limit: 20 }),
        api.listDevices(),
      ])
      setBranches(b.branches)
      setReleases(r.releases)
      setRuns(w.runs)
      setDevices(d.devices)
      const selected = branch || b.branches[0]?.name
      if (!branch && b.branches[0]?.name) {
        setBranch(b.branches[0].name)
      }
      if (selected) {
        try {
          const tip = await api.getBranchTip(selected)
          setBranchTip(tip.tip)
        } catch {
          setBranchTip(null)
        }
      } else {
        setBranchTip(null)
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err))
    } finally {
      setLoading(false)
    }
  }, [branch])

  const robots = useMemo(
    () => devices.filter((d) => d.role !== 'builder'),
    [devices],
  )

  const latestRelease = releases[0] || null

  const focusedRelease = useMemo(() => {
    if (!focusKey || !releases.length) return null
    const byId = Number(focusKey)
    if (Number.isFinite(byId) && byId > 0) {
      const hit = releases.find((r) => r.id === byId)
      if (hit) return hit
    }
    const key = focusKey.toLowerCase()
    return (
      releases.find(
        (r) =>
          r.git_sha?.toLowerCase() === key ||
          r.git_sha?.toLowerCase().startsWith(key) ||
          (r.short_sha && r.short_sha.toLowerCase() === key),
      ) || null
    )
  }, [focusKey, releases])

  const highlightRelease = focusedRelease || latestRelease

  const previousRelease = useMemo(() => {
    if (!highlightRelease) return null
    const idx = releases.findIndex((r) => r.id === highlightRelease.id)
    return idx >= 0 && idx + 1 < releases.length ? releases[idx + 1] : null
  }, [highlightRelease, releases])

  const tipSha = shortSha(branchTip?.sha || branchTip?.short_sha)
  const tipHasSuccessfulRelease = useMemo(() => {
    if (!tipSha || tipSha === '—') return true
    return releases.some((r) => {
      const rs = shortSha(r.git_sha)
      return (
        rs === tipSha ||
        (r.git_sha || '').toLowerCase().startsWith(tipSha.toLowerCase()) ||
        tipSha.toLowerCase().startsWith(rs.toLowerCase())
      )
    })
  }, [releases, tipSha])

  const needsCiBuild = Boolean(branchTip?.sha) && !tipHasSuccessfulRelease

  useEffect(() => {
    void load()
    const id = window.setInterval(() => void load(), 20000)
    return () => window.clearInterval(id)
  }, [load])

  useEffect(() => {
    const q = searchParams.get('branch')
    if (q && q !== branch) setBranch(q)
  }, [searchParams])

  useEffect(() => {
    if (!focusedRelease) return
    const el = document.getElementById(`release-row-${focusedRelease.id}`)
    el?.scrollIntoView({ behavior: 'smooth', block: 'center' })
  }, [focusedRelease, loading])

  const onBranchChange = (next: string) => {
    setBranch(next)
    const nextParams: Record<string, string> = {}
    if (next && next !== 'main') nextParams.branch = next
    if (focusKey) nextParams.focus = focusKey
    setSearchParams(nextParams)
  }

  const triggerBuild = async () => {
    setBusy(true)
    setError(null)
    setInfo(null)
    try {
      const resp = await api.triggerBuild({ branch })
      const url = resp.workflow?.actions_url
      setInfo(
        url
          ? `CI dispatched for ${branch}. Watch ${String(url)} — a successful Release appears below when images are pushed.`
          : `CI dispatched for ${branch}.`,
      )
      setConfirmBuild(false)
      await load()
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err))
    } finally {
      setBusy(false)
    }
  }

  return (
    <div>
      <SectionHeader
        title="Releases & builds"
        description="Build CI for a tracked git branch (usually main). Successful runs create Releases with version #N. Edge devices pull the orphan deploy branch via deploy-<sha> tags — you do not develop on deploy."
        action={
          <Button variant="outline" onClick={() => void load()} disabled={loading}>
            <RefreshCw className="h-4 w-4" />
            Refresh
          </Button>
        }
      />

      {error ? (
        <div className="mb-4 rounded-[var(--radius-md)] border border-[var(--status-bad-fg)]/40 bg-[var(--status-bad-bg)] px-4 py-3 text-sm text-[var(--status-bad-fg)]">
          {error}
        </div>
      ) : null}
      {info ? (
        <div className="mb-4 rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--surface-1)] px-4 py-3 text-sm text-[var(--text-secondary)]">
          {info}
        </div>
      ) : null}

      <div className="mb-8 rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--card-surface)] p-4">
        <h3 className="font-display text-base font-semibold">Build CI</h3>
        <p className="mt-1 text-sm text-[var(--text-muted)]">
          Dispatches GitHub Actions <code>build-and-release.yml</code> (multi-arch).
          When it finishes, the Release shows up for device Deploy.
        </p>

        {needsCiBuild ? (
          <div className="mt-3 rounded-[var(--radius-sm)] border border-[var(--status-warn-fg)]/40 bg-[var(--status-warn-bg)] px-3 py-2.5 text-sm text-[var(--status-warn-fg)]">
            <div className="flex flex-wrap items-center gap-2">
              <StatusBadge label="Needs CI build" tone="warn" pulse />
              <span className="font-medium text-[var(--text-primary)]">
                Newest git on <code>{branch}</code> is not a Release yet
              </span>
            </div>
            <div className="mt-1 text-[var(--text-primary)]">
              Tip <code>{tipSha}</code>
              {branchTip?.message ? (
                <>
                  {' '}
                  — {branchTip.message}
                </>
              ) : null}
            </div>
            <div className="mt-1 text-xs text-[var(--text-muted)]">
              Latest verified Release is still{' '}
              {latestRelease ? (
                <strong>{releaseVersion(latestRelease)}</strong>
              ) : (
                'none'
              )}
              . Click <strong>Build CI</strong> for this tip; after success, Latest
              becomes this sha and Devices show Update available.
            </div>
          </div>
        ) : branchTip?.sha ? (
          <div className="mt-3 text-xs text-[var(--text-muted)]">
            Branch tip <code>{tipSha}</code> already has a successful Release
            {latestRelease ? ` (${releaseVersion(latestRelease)})` : ''}.
            {branchTip.message ? ` “${branchTip.message}”` : ''}
          </div>
        ) : null}

        <div className="mt-4 flex flex-wrap items-end gap-3">
          <label className="block min-w-[220px] text-xs text-[var(--text-muted)]">
            Branch
            <select
              className="mt-1 w-full rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-1)] px-3 py-2 text-sm"
              value={branch}
              onChange={(e) => onBranchChange(e.target.value)}
            >
              {branches.length === 0 ? (
                <option value={branch}>{branch}</option>
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
          <Button onClick={() => setConfirmBuild(true)} disabled={busy || !branch}>
            <Rocket className="h-4 w-4" />
            {busy ? 'Dispatching…' : needsCiBuild ? 'Build CI (tip)' : 'Build CI'}
          </Button>
        </div>
      </div>

      <SectionHeader
        title="Recent CI runs"
        description="Latest GitHub Actions runs for build-and-release (max 20)."
      />
      <div className="mb-8">
        <DataTable
          rows={runs}
          rowKey={(r) => r.id || `${r.head_sha}-${r.created_at}`}
          emptyMessage={loading ? 'Loading…' : 'No workflow runs yet.'}
          columns={[
            {
              key: 'branch',
              header: 'Branch',
              render: (r) => (
                <div className="text-sm">
                  <div className="font-medium">{r.head_branch || '—'}</div>
                  <code className="text-xs text-[var(--accent)]">{r.head_sha || ''}</code>
                </div>
              ),
            },
            {
              key: 'status',
              header: 'Status',
              render: (r) => (
                <StatusBadge
                  label={r.conclusion || r.status || '—'}
                  tone={runTone(r.status, r.conclusion)}
                  pulse={r.status === 'in_progress' || r.status === 'queued'}
                />
              ),
            },
            {
              key: 'when',
              header: 'Started',
              render: (r) => (
                <span className="text-xs text-[var(--text-muted)]">
                  {r.created_at ? new Date(r.created_at).toLocaleString() : '—'}
                </span>
              ),
            },
            {
              key: 'link',
              header: '',
              render: (r) =>
                r.html_url ? (
                  <a
                    href={r.html_url}
                    target="_blank"
                    rel="noreferrer"
                    className="inline-flex items-center gap-1 text-xs text-[var(--accent)]"
                  >
                    Open <ExternalLink className="h-3 w-3" />
                  </a>
                ) : (
                  '—'
                ),
            },
          ]}
        />
      </div>

      <SectionHeader
        title="Verified releases"
        description="Each row is a deployable version (#N). Newest is at the top — look for the Latest badge. Open from Devices “Update available” to jump here with notes."
      />

      {highlightRelease ? (
        <div
          id={`release-spotlight-${highlightRelease.id}`}
          className={`mb-4 rounded-[var(--radius-md)] border px-4 py-3 text-sm ${
            focusedRelease
              ? 'border-[var(--status-warn-fg)]/50 bg-[var(--status-warn-bg)] text-[var(--status-warn-fg)]'
              : 'border-[var(--accent)]/40 bg-[var(--surface-1)] text-[var(--text-secondary)]'
          }`}
        >
          <div className="flex flex-wrap items-center gap-2">
            <StatusBadge
              label={focusedRelease ? 'Opened from Update available' : 'Latest release'}
              tone={focusedRelease ? 'warn' : 'info'}
              pulse={Boolean(focusedRelease)}
            />
            <span className="font-display font-semibold text-[var(--text-primary)]">
              {releaseVersion(highlightRelease)}
            </span>
            <code className="text-xs text-[var(--text-muted)]">
              {highlightRelease.git_sha}
            </code>
          </div>
          <div className="mt-2">
            <div className="text-[10px] uppercase tracking-wide text-[var(--text-muted)]">
              What changed (commit subject)
            </div>
            <p className="mt-0.5 whitespace-pre-wrap text-[var(--text-primary)]">
              {releaseSummary(highlightRelease) || 'No subject recorded for this Release.'}
            </p>
          </div>
          {previousRelease ? (
            <div className="mt-2 text-xs text-[var(--text-muted)]">
              Previous on this list:{' '}
              <span className="text-[var(--text-secondary)]">
                {releaseVersion(previousRelease)}
              </span>
              {' — '}
              {releaseSummary(previousRelease) || 'no subject'}
            </div>
          ) : null}
          <div className="mt-2 text-xs text-[var(--text-muted)]">
            Branch <code>{highlightRelease.branch}</code>
            {highlightRelease.reported_at
              ? ` · reported ${new Date(highlightRelease.reported_at).toLocaleString()}`
              : ''}
            {' · '}
            {
              robots.filter(
                (d) =>
                  d.image_tag &&
                  d.image_tag === highlightRelease.git_sha,
              ).length
            }{' '}
            device(s) running this ·{' '}
            {
              robots.filter((d) => d.update_available && d.latest_release?.id === highlightRelease.id)
                .length
            }{' '}
            waiting to update to it
          </div>
          <div className="mt-2 text-xs">
            To apply: open the device → select this Release → Deploy. This page does not
            deploy by itself.
          </div>
        </div>
      ) : null}

      <DataTable
        rows={releases}
        rowKey={(r) => r.id}
        emptyMessage={loading ? 'Loading…' : 'No deployable releases yet. Run Build CI.'}
        columns={[
          {
            key: 'version',
            header: 'Version',
            render: (r) => {
              const isLatest = latestRelease?.id === r.id
              const isFocus = focusedRelease?.id === r.id
              const runningN = robots.filter(
                (d) => d.image_tag && d.image_tag === r.git_sha,
              ).length
              return (
                <div id={`release-row-${r.id}`}>
                  <div className="flex flex-wrap items-center gap-1.5">
                    <div
                      className={`font-medium ${
                        isLatest || isFocus
                          ? 'text-[var(--accent)]'
                          : 'text-[var(--text-primary)]'
                      }`}
                    >
                      {releaseVersion(r)}
                    </div>
                    {isLatest ? (
                      <StatusBadge label="Latest" tone="info" />
                    ) : null}
                    {isFocus && !isLatest ? (
                      <StatusBadge label="Focused" tone="warn" />
                    ) : null}
                    {runningN > 0 ? (
                      <StatusBadge
                        label={`Running ×${runningN}`}
                        tone="good"
                      />
                    ) : (
                      <StatusBadge label="Not on fleet yet" tone="neutral" />
                    )}
                  </div>
                  <div
                    className={`mt-0.5 text-xs text-[var(--text-secondary)] ${
                      isLatest || isFocus ? 'max-w-xl whitespace-pre-wrap' : 'max-w-[22rem] truncate'
                    }`}
                    title={releaseSummary(r)}
                  >
                    {releaseSummary(r)}
                  </div>
                  <div className="mt-0.5 text-[10px] text-[var(--text-muted)]">
                    branch {r.branch}
                    {r.reported_at
                      ? ` · ${new Date(r.reported_at).toLocaleString()}`
                      : ''}
                  </div>
                </div>
              )
            },
          },
          {
            key: 'sha',
            header: 'Image tag',
            render: (r) => (
              <code className="text-xs text-[var(--text-muted)]">
                {r.git_sha}
              </code>
            ),
          },
          {
            key: 'platforms',
            header: 'Built for',
            render: (r) => {
              const labels = platformLabels(r.platforms)
              const classes = r.device_classes || []
              return (
                <div className="text-xs">
                  <div className="text-[var(--text-secondary)]">
                    {labels.length ? labels.join(' · ') : 'unknown'}
                  </div>
                  <div className="mt-0.5 text-[10px] text-[var(--text-muted)]">
                    {classes.length
                      ? `Fleet classes: ${classes.join(', ')}`
                      : labels.includes('arm64') && labels.includes('amd64')
                        ? 'Multi-arch Hub image — no class allow-list'
                        : labels.includes('arm64')
                          ? 'arm64 only — no class allow-list'
                          : labels.includes('amd64')
                            ? 'amd64 only'
                            : 'Platforms not recorded on this Release'}
                  </div>
                </div>
              )
            },
          },
          {
            key: 'intended',
            header: 'Fleet devices',
            render: (r) => {
              const matches = matchingDevices(r, robots)
              const desiredHere = matches.filter(
                (d) => d.desired_image_tag && d.desired_image_tag === r.git_sha,
              )
              const runningHere = matches.filter(
                (d) => d.image_tag && d.image_tag === r.git_sha,
              )
              if (!r.platforms?.length) {
                return (
                  <span className="text-xs text-[var(--text-muted)]">
                    Unknown — no platform metadata
                  </span>
                )
              }
              if (!matches.length) {
                return (
                  <span className="text-xs text-[var(--text-muted)]">
                    No matching devices in fleet
                  </span>
                )
              }
              return (
                <div className="flex max-w-[280px] flex-wrap gap-x-2 gap-y-0.5 text-xs">
                  {matches.map((d) => {
                    const tags: string[] = []
                    if (runningHere.some((x) => x.id === d.id)) tags.push('running')
                    if (desiredHere.some((x) => x.id === d.id)) tags.push('desired')
                    const cls = d.device_class || d.target?.device_class
                    return (
                      <Link
                        key={d.id}
                        to={`/devices/${d.id}`}
                        className="text-[var(--accent)] hover:underline"
                        title={`${cls || '?'} / ${devicePlatform(d) || '?'}`}
                      >
                        {d.hostname || d.id}
                        {cls ? (
                          <span className="text-[var(--text-muted)]"> ({cls})</span>
                        ) : null}
                        {tags.length ? (
                          <span className="text-[var(--text-muted)]">
                            {' '}
                            · {tags.join(', ')}
                          </span>
                        ) : null}
                      </Link>
                    )
                  })}
                </div>
              )
            },
          },
          {
            key: 'build',
            header: 'CI build',
            render: (r) => (
              <span className="text-xs text-[var(--text-muted)]">
                {formatDuration(r.duration_seconds) || '—'}
              </span>
            ),
          },
          {
            key: 'when',
            header: 'Reported',
            render: (r) => (
              <span className="text-xs text-[var(--text-muted)]">
                {r.reported_at ? new Date(r.reported_at).toLocaleString() : '—'}
              </span>
            ),
          },
          {
            key: 'link',
            header: '',
            render: (r) =>
              r.workflow_run_url ? (
                <a
                  href={r.workflow_run_url}
                  target="_blank"
                  rel="noreferrer"
                  className="inline-flex items-center gap-1 text-xs text-[var(--accent)]"
                >
                  CI <ExternalLink className="h-3 w-3" />
                </a>
              ) : (
                '—'
              ),
          },
        ]}
      />

      <ConfirmDialog
        open={confirmBuild}
        title="Trigger CI build?"
        message={`Dispatch build-and-release for branch ${branch}. Multi-arch images will be pushed; a Release appears here when CI reports success.`}
        confirmLabel="Build CI"
        loading={busy}
        onCancel={() => setConfirmBuild(false)}
        onConfirm={() => void triggerBuild()}
      />
    </div>
  )
}
