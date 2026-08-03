import { useCallback, useEffect, useMemo, useState } from 'react'
import { GraduationCap } from 'lucide-react'
import { toast } from 'sonner'
import GlassCard from '../components/GlassCard'
import Button from '../components/ui/button'
import StatusBadge from '../components/ui/StatusBadge'
import { EmptyState, SectionHeader } from '../components/ui/SectionHeader'
import { useRuntimeConfig } from '../config/RuntimeConfig'

type RuntimeMode = {
  mode: string
  environment: string
}

type CapabilityMode = {
  mode: string
  label?: string
  description?: string
}

type Capabilities = {
  modes: CapabilityMode[]
}

type LightsoutRunResponse = {
  ok: boolean
  accepted: boolean
  message: string
  request?: {
    powder_name: string
    target_weight_g: number
    episodes: number
    batch_id: string
    enable_scoop: boolean
  }
}

export default function TrainingPage() {
  const { apiBase } = useRuntimeConfig()
  const [open, setOpen] = useState(false)
  const [powderName, setPowderName] = useState('boxA')
  const [cycleEndLimit, setCycleEndLimit] = useState('')
  const [targetWeight, setTargetWeight] = useState('250.0')
  const [episodes, setEpisodes] = useState('10')
  const [batchId, setBatchId] = useState('sim-007')
  const [enableScoop, setEnableScoop] = useState(false)
  const [startMsg, setStartMsg] = useState('')
  const [runtime, setRuntime] = useState<RuntimeMode | null>(null)
  const [capabilities, setCapabilities] = useState<Capabilities | null>(null)
  const [loading, setLoading] = useState(true)
  const [switching, setSwitching] = useState(false)
  const [starting, setStarting] = useState(false)
  const [error, setError] = useState<string | null>(null)
  const [lastStart, setLastStart] = useState<LightsoutRunResponse | null>(null)

  const lightsoutCapability = useMemo(
    () => capabilities?.modes.find((m) => m.mode === 'lightsout') ?? null,
    [capabilities]
  )
  const isLightsoutMode = runtime?.mode === 'lightsout'

  const startDisabled = useMemo(() => {
    if (starting) return true
    if (!powderName.trim()) return true
    if (!targetWeight.trim()) return true
    if (!episodes.trim()) return true
    return false
  }, [powderName, targetWeight, episodes, starting])

  const loadRuntime = useCallback(async () => {
    const res = await fetch(`${apiBase}/runtime/mode`)
    if (!res.ok) throw new Error('Failed to load runtime mode')
    setRuntime((await res.json()) as RuntimeMode)
  }, [apiBase])

  const loadCapabilities = useCallback(async () => {
    const res = await fetch(`${apiBase}/runtime/capabilities`)
    if (!res.ok) throw new Error('Failed to load capabilities')
    setCapabilities((await res.json()) as Capabilities)
  }, [apiBase])

  const refreshAll = useCallback(async () => {
    setLoading(true)
    setError(null)
    try {
      await Promise.all([loadRuntime(), loadCapabilities()])
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to load training page')
    } finally {
      setLoading(false)
    }
  }, [loadCapabilities, loadRuntime])

  useEffect(() => {
    void refreshAll()
  }, [refreshAll])

  const resetForm = () => {
    setPowderName('boxA')
    setCycleEndLimit('')
    setTargetWeight('250.0')
    setEpisodes('10')
    setBatchId('sim-007')
    setEnableScoop(false)
    setStartMsg('')
  }

  const switchToLightsout = async () => {
    setSwitching(true)
    setError(null)
    try {
      const res = await fetch(`${apiBase}/runtime/mode`, {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          mode: 'lightsout',
          environment: runtime?.environment || 'real',
        }),
      })
      const json = await res.json().catch(() => ({}))
      if (!res.ok) {
        const detail =
          typeof json.detail === 'string'
            ? json.detail
            : json.detail?.message || `Mode switch failed (${res.status})`
        throw new Error(detail)
      }
      setRuntime(json as RuntimeMode)
      toast.success('Mode set to lightsout')
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Mode switch failed'
      setError(message)
      toast.error(message)
    } finally {
      setSwitching(false)
    }
  }

  const startLightsOut = async () => {
    setStartMsg('')
    setError(null)
    const targetWeightG = Number.parseFloat(targetWeight)
    const episodeCount = Number.parseInt(episodes, 10)
    if (!Number.isFinite(targetWeightG) || targetWeightG <= 0) {
      setStartMsg('Target weight must be a positive number')
      return
    }
    if (!Number.isFinite(episodeCount) || episodeCount <= 0) {
      setStartMsg('Episodes must be a positive integer')
      return
    }
    setStarting(true)
    try {
      const res = await fetch(`${apiBase}/modes/lightsout/runs`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          powder_name: powderName.trim(),
          cycle_end_limit: cycleEndLimit.trim(),
          target_weight_g: targetWeightG,
          episodes: episodeCount,
          batch_id: batchId.trim(),
          enable_scoop: enableScoop,
        }),
      })
      const json = await res.json().catch(() => ({}))
      if (!res.ok) {
        const detail =
          typeof json.detail === 'string'
            ? json.detail
            : json.detail?.message || `Start failed (${res.status})`
        throw new Error(detail)
      }
      setLastStart(json as LightsoutRunResponse)
      setStartMsg(json.message || 'Lights-out training accepted')
      toast.success('Lights-out training started')
      setOpen(false)
      await loadRuntime()
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Failed to start lights-out'
      setStartMsg(message)
      setError(message)
      toast.error(message)
    } finally {
      setStarting(false)
    }
  }

  if (loading) {
    return (
      <div className="px-6 py-6">
        <SectionHeader title="Training" description="Lights-out training runs." />
        <div className="py-12 text-center text-sm text-[var(--text-muted)]">Loading…</div>
      </div>
    )
  }

  if (!lightsoutCapability) {
    return (
      <div className="px-6 py-6">
        <SectionHeader title="Training" description="Lights-out training runs." />
        <EmptyState
          title="Lights-out unavailable"
          description="This device does not advertise the lightsout capability. Check backend mode registry / runtime capabilities."
        />
      </div>
    )
  }

  return (
    <div className="px-6 py-6">
      <SectionHeader
        title="Training"
        description="Configure and start lights-out training runs through the backend (no Condor MES)."
        action={
          <Button variant="outline" onClick={() => void refreshAll()} disabled={loading}>
            Refresh
          </Button>
        }
      />

      {error && (
        <div className="mb-4 rounded-[var(--radius-sm)] border border-[var(--status-bad-fg)]/30 bg-[var(--status-bad-bg)] px-3 py-2 text-sm text-[var(--status-bad-fg)]">
          {error}
        </div>
      )}

      <div className="mb-4 flex flex-wrap items-center gap-2">
        <StatusBadge
          label={runtime?.mode || 'unknown'}
          tone={isLightsoutMode ? 'good' : 'idle'}
        />
        <span className="text-xs text-[var(--text-muted)]">
          env {runtime?.environment || '—'}
        </span>
        {!isLightsoutMode && (
          <Button
            variant="outline"
            size="sm"
            onClick={() => void switchToLightsout()}
            disabled={switching}
          >
            {switching ? 'Switching…' : 'Switch to lightsout'}
          </Button>
        )}
      </div>

      <div className="grid grid-cols-12 gap-4">
        <div className="col-span-12 md:col-span-6">
          <GlassCard>
            <div className="flex flex-col gap-3">
              <div className="flex items-center gap-2">
                <GraduationCap className="h-4 w-4 text-[var(--accent)]" />
                <h3 className="text-lg font-semibold">Lights-Out Training</h3>
              </div>
              <p className="text-sm text-[var(--text-secondary)]">
                Starts via <code className="font-mono text-xs">POST /modes/lightsout/runs</code>{' '}
                → robot_start_adapter → <code className="font-mono text-xs">/bt_start_lightsout</code>.
                ExecuteScoop stays off unless you opt in.
              </p>
              {!isLightsoutMode ? (
                <EmptyState
                  title="Switch to lightsout first"
                  description="Training runs require active mode lightsout (HTTP 409 otherwise)."
                  action={
                    <Button onClick={() => void switchToLightsout()} disabled={switching}>
                      Switch to lightsout
                    </Button>
                  }
                />
              ) : (
                <Button onClick={() => setOpen(true)}>Start Lights-Out Training</Button>
              )}
            </div>
          </GlassCard>
        </div>

        {lastStart && (
          <div className="col-span-12 md:col-span-6">
            <GlassCard>
              <h3 className="mb-2 text-sm font-semibold">Last start</h3>
              <div className="space-y-1 font-mono text-xs text-[var(--text-secondary)]">
                <div>{lastStart.message}</div>
                {lastStart.request && (
                  <>
                    <div>powder: {lastStart.request.powder_name}</div>
                    <div>
                      {lastStart.request.target_weight_g} g · {lastStart.request.episodes}{' '}
                      episodes · scoop{' '}
                      {lastStart.request.enable_scoop ? 'on' : 'off'}
                    </div>
                    <div>batch: {lastStart.request.batch_id || '—'}</div>
                  </>
                )}
              </div>
            </GlassCard>
          </div>
        )}
      </div>

      {open && (
        <div className="fixed inset-0 z-50 flex items-center justify-center bg-[var(--overlay-backdrop)]">
          <div className="w-[92%] max-w-lg rounded-xl border border-[var(--border)] bg-[var(--surface)] p-5 shadow-xl">
            <div className="mb-4 flex items-center justify-between">
              <h2 className="text-lg font-semibold">Training Metadata</h2>
              <button
                className="text-[var(--text-muted)] hover:text-[var(--text-primary)]"
                onClick={() => setOpen(false)}
                aria-label="Close"
              >
                ✕
              </button>
            </div>

            <div className="grid grid-cols-1 gap-3">
              <label className="text-sm text-[var(--text-secondary)]">
                Powder name
                <input
                  value={powderName}
                  onChange={(e) => setPowderName(e.target.value)}
                  className="mt-1 w-full rounded border border-[var(--border)] bg-transparent px-2 py-1 text-sm text-[var(--text-primary)]"
                  placeholder="e.g. Alumina 5um"
                />
              </label>
              <label className="text-sm text-[var(--text-secondary)]">
                Cycle end limit
                <input
                  value={cycleEndLimit}
                  onChange={(e) => setCycleEndLimit(e.target.value)}
                  className="mt-1 w-full rounded border border-[var(--border)] bg-transparent px-2 py-1 text-sm text-[var(--text-primary)]"
                  placeholder="e.g. 120s or 150 cycles"
                />
              </label>
              <label className="text-sm text-[var(--text-secondary)]">
                Target weight (g)
                <input
                  value={targetWeight}
                  onChange={(e) => setTargetWeight(e.target.value)}
                  className="mt-1 w-full rounded border border-[var(--border)] bg-transparent px-2 py-1 text-sm text-[var(--text-primary)]"
                  placeholder="e.g. 125.0"
                />
              </label>
              <label className="text-sm text-[var(--text-secondary)]">
                Episodes
                <input
                  value={episodes}
                  onChange={(e) => setEpisodes(e.target.value)}
                  className="mt-1 w-full rounded border border-[var(--border)] bg-transparent px-2 py-1 text-sm text-[var(--text-primary)]"
                  placeholder="e.g. 10"
                />
              </label>
              <label className="text-sm text-[var(--text-secondary)]">
                Batch ID
                <input
                  value={batchId}
                  onChange={(e) => setBatchId(e.target.value)}
                  className="mt-1 w-full rounded border border-[var(--border)] bg-transparent px-2 py-1 text-sm text-[var(--text-primary)]"
                  placeholder="e.g. batch-2026-01-19"
                />
              </label>
              <label className="flex items-center gap-2 text-sm text-[var(--text-secondary)]">
                <input
                  type="checkbox"
                  checked={enableScoop}
                  onChange={(e) => setEnableScoop(e.target.checked)}
                  className="rounded border-[var(--border)]"
                />
                Enable ExecuteScoop (off by default — motion + pour only when unchecked)
              </label>
            </div>

            {startMsg && (
              <p className="mt-3 text-xs text-[var(--text-secondary)]">{startMsg}</p>
            )}

            <div className="mt-5 flex items-center justify-end gap-2">
              <Button
                variant="ghost"
                onClick={() => {
                  resetForm()
                  setOpen(false)
                }}
              >
                Cancel
              </Button>
              <Button onClick={() => void startLightsOut()} disabled={startDisabled}>
                {starting ? 'Starting…' : 'Start'}
              </Button>
            </div>
          </div>
        </div>
      )}
    </div>
  )
}
