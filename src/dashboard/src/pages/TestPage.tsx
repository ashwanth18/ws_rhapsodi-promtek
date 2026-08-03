import { useCallback, useEffect, useMemo, useState } from 'react'
import { FlaskConical } from 'lucide-react'
import { toast } from 'sonner'
import Button from '../components/ui/button'
import StatusBadge from '../components/ui/StatusBadge'
import { EmptyState, SectionHeader } from '../components/ui/SectionHeader'
import Select from '../components/ui/select'
import { useRuntimeConfig } from '../config/RuntimeConfig'

type RuntimeMode = {
  mode: string
  environment: string
  active_run?: {
    id: number
    status: string
    weightment_id: number | null
    event_id: string | null
    batch_id: string | null
  } | null
}

type CapabilityMode = {
  mode: string
  label?: string
  description?: string
  allowed_environments?: string[]
}

type Capabilities = {
  modes: CapabilityMode[]
  sim_allowed: boolean
  default_mode: string
}

type ActiveRobotRun = {
  run_id: number
  event_id: string | null
  batch_id: string | null
  weightment_id: number
  status: string | null
  ingredient_name: string | null
  target_weight_kg: number | null
  location_code: string | null
  started_at: string | null
  error_message?: string | null
}

type MockRunResponse = {
  ok: boolean
  event_id: string
  weightment_id: number
  run: {
    id: number
    status: string
    event_id: string | null
    target_weight_g: number | null
    weight_tolerance_g: number | null
    stock_location_code: string | null
    pickup_target_name: string | null
    weigh_target_name: string | null
    return_target_name: string | null
    error_message: string | null
  } | null
  contract?: {
    location_code: string
    pickup_target_name: string
    weigh_target_name: string
    return_target_name: string
    target_weight_g: number
    weight_tolerance_g: number
  }
}

function runTone(status: string | null | undefined): 'good' | 'warn' | 'bad' | 'idle' {
  if (status === 'succeeded') return 'good'
  if (status === 'failed' || status === 'mes_send_failed') return 'bad'
  if (status === 'starting' || status === 'running' || status === 'awaiting_processing') {
    return 'warn'
  }
  return 'idle'
}

export default function TestPage() {
  const { apiBase } = useRuntimeConfig()
  const [runtime, setRuntime] = useState<RuntimeMode | null>(null)
  const [capabilities, setCapabilities] = useState<Capabilities | null>(null)
  const [activeRun, setActiveRun] = useState<ActiveRobotRun | null>(null)
  const [lastMock, setLastMock] = useState<MockRunResponse | null>(null)
  const [loading, setLoading] = useState(true)
  const [switching, setSwitching] = useState(false)
  const [starting, setStarting] = useState(false)
  const [targetWeightG, setTargetWeightG] = useState('100')
  const [toleranceG, setToleranceG] = useState('')
  const [locationCode, setLocationCode] = useState('')
  const [selectedMode, setSelectedMode] = useState('mock-local')
  const [error, setError] = useState<string | null>(null)

  const mockCapability = useMemo(
    () => capabilities?.modes.find((m) => m.mode === 'mock-local') ?? null,
    [capabilities]
  )
  const isMockMode = runtime?.mode === 'mock-local'
  const hasActive =
    !!activeRun &&
    ['starting', 'running', 'awaiting_processing'].includes(activeRun.status ?? '')

  const loadRuntime = useCallback(async () => {
    const res = await fetch(`${apiBase}/runtime/mode`)
    if (!res.ok) throw new Error('Failed to load runtime mode')
    const json = (await res.json()) as RuntimeMode
    setRuntime(json)
    setSelectedMode(json.mode)
  }, [apiBase])

  const loadCapabilities = useCallback(async () => {
    const res = await fetch(`${apiBase}/runtime/capabilities`)
    if (!res.ok) throw new Error('Failed to load capabilities')
    setCapabilities((await res.json()) as Capabilities)
  }, [apiBase])

  const loadActiveRun = useCallback(async () => {
    const res = await fetch(`${apiBase}/robot_weightment_runs/active`)
    if (!res.ok) throw new Error('Failed to load active robot run')
    const json = await res.json()
    setActiveRun(json.active || null)
  }, [apiBase])

  const refreshAll = useCallback(async () => {
    setLoading(true)
    setError(null)
    try {
      await Promise.all([loadRuntime(), loadCapabilities(), loadActiveRun()])
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to load test page')
    } finally {
      setLoading(false)
    }
  }, [loadActiveRun, loadCapabilities, loadRuntime])

  useEffect(() => {
    void refreshAll()
  }, [refreshAll])

  useEffect(() => {
    if (!hasActive && !isMockMode) return
    const id = window.setInterval(() => {
      void loadActiveRun()
      void loadRuntime()
    }, 2000)
    return () => window.clearInterval(id)
  }, [hasActive, isMockMode, loadActiveRun, loadRuntime])

  const switchMode = async (mode: string) => {
    setSwitching(true)
    setError(null)
    try {
      const res = await fetch(`${apiBase}/runtime/mode`, {
        method: 'PUT',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          mode,
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
      setSelectedMode(json.mode)
      toast.success(`Mode set to ${json.mode}`)
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Mode switch failed'
      setError(message)
      toast.error(message)
    } finally {
      setSwitching(false)
    }
  }

  const startMockRun = async () => {
    const target = Number.parseFloat(targetWeightG)
    if (!Number.isFinite(target) || target <= 0) {
      toast.error('Target weight must be a positive number')
      return
    }
    let tolerance: number | undefined
    if (toleranceG.trim()) {
      tolerance = Number.parseFloat(toleranceG)
      if (!Number.isFinite(tolerance) || tolerance < 0) {
        toast.error('Tolerance must be a non-negative number')
        return
      }
    }
    setStarting(true)
    setError(null)
    try {
      const body: Record<string, unknown> = { target_weight_g: target }
      if (tolerance != null) body.tolerance_g = tolerance
      if (locationCode.trim()) body.location_code = locationCode.trim()
      const res = await fetch(`${apiBase}/modes/mock/runs`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(body),
      })
      const json = await res.json().catch(() => ({}))
      if (!res.ok) {
        const detail =
          typeof json.detail === 'string'
            ? json.detail
            : json.detail?.message || `Start failed (${res.status})`
        throw new Error(detail)
      }
      setLastMock(json as MockRunResponse)
      toast.success(`Mock run started (weightment ${json.weightment_id})`)
      await loadActiveRun()
      await loadRuntime()
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Failed to start mock run'
      setError(message)
      toast.error(message)
    } finally {
      setStarting(false)
    }
  }

  if (loading) {
    return (
      <div className="px-5 py-5 lg:px-6">
        <SectionHeader title="Test" description="Mock-local single-location runs." />
        <div className="py-12 text-center text-sm text-[var(--text-muted)]">Loading…</div>
      </div>
    )
  }

  if (!mockCapability) {
    return (
      <div className="px-5 py-5 lg:px-6">
        <SectionHeader title="Test" description="Mock-local single-location runs." />
        <EmptyState
          title="Mock-local unavailable"
          description="This device does not advertise the mock-local capability. Check backend mode registry / runtime capabilities."
        />
      </div>
    )
  }

  return (
    <div className="px-5 py-5 lg:px-6">
      <SectionHeader
        title="Test"
        description="Operator-triggered mock-local weighment — uses the production webhook start path with no Condor traffic."
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

      <div className="grid gap-4 lg:grid-cols-2">
        <section className="rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--surface-1)] p-4">
          <div className="mb-3 flex items-center gap-2">
            <FlaskConical className="h-4 w-4 text-[var(--accent)]" />
            <h3 className="font-display text-sm font-semibold">Runtime mode</h3>
          </div>
          <div className="mb-3 flex flex-wrap items-center gap-2">
            <StatusBadge
              label={runtime?.mode || 'unknown'}
              tone={isMockMode ? 'good' : 'idle'}
            />
            <span className="text-xs text-[var(--text-muted)]">
              env {runtime?.environment || '—'}
            </span>
          </div>
          <label className="mb-1 block text-xs font-medium text-[var(--text-muted)]">
            Mode
          </label>
          <div className="flex flex-wrap items-center gap-2">
            <Select
              value={selectedMode}
              onChange={(e) => setSelectedMode(e.target.value)}
              className="min-w-[10rem]"
              disabled={switching || hasActive}
            >
              {(capabilities?.modes || []).map((m) => (
                <option key={m.mode} value={m.mode}>
                  {m.label || m.mode}
                </option>
              ))}
            </Select>
            <Button
              onClick={() => void switchMode(selectedMode)}
              disabled={switching || hasActive || selectedMode === runtime?.mode}
            >
              {switching ? 'Switching…' : 'Apply mode'}
            </Button>
            {!isMockMode && (
              <Button
                variant="outline"
                onClick={() => void switchMode('mock-local')}
                disabled={switching || hasActive}
              >
                Switch to mock-local
              </Button>
            )}
          </div>
          {hasActive && (
            <p className="mt-2 text-xs text-[var(--status-warn-fg)]">
              Mode changes are blocked while a robot run is active.
            </p>
          )}
        </section>

        <section className="rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--surface-1)] p-4">
          <h3 className="mb-3 font-display text-sm font-semibold">Active run</h3>
          {activeRun ? (
            <div className="space-y-2 text-sm">
              <div className="flex flex-wrap items-center gap-2">
                <StatusBadge
                  label={activeRun.status || 'unknown'}
                  tone={runTone(activeRun.status)}
                  pulse={['starting', 'running'].includes(activeRun.status ?? '')}
                />
                <span className="font-mono text-xs text-[var(--text-secondary)]">
                  run {activeRun.run_id} · wt {activeRun.weightment_id}
                </span>
              </div>
              <div className="text-xs text-[var(--text-muted)]">
                event {activeRun.event_id ?? '—'}
                {activeRun.location_code ? ` · ${activeRun.location_code}` : ''}
                {activeRun.target_weight_kg != null
                  ? ` · ${(activeRun.target_weight_kg * 1000).toFixed(1)} g`
                  : ''}
              </div>
              {activeRun.error_message && (
                <div className="text-xs text-[var(--status-bad-fg)]">
                  {activeRun.error_message}
                </div>
              )}
            </div>
          ) : (
            <p className="text-sm text-[var(--text-muted)]">No active robot run.</p>
          )}
        </section>
      </div>

      <section className="mt-4 rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--surface-1)] p-4">
        <h3 className="mb-1 font-display text-sm font-semibold">Start mock run</h3>
        <p className="mb-4 text-xs text-[var(--text-muted)]">
          Creates a synthetic weightment (`mock-…` event id) and starts via{' '}
          <code className="font-mono">robot_start_adapter</code>. MES send is skipped on
          completion.
        </p>
        {!isMockMode ? (
          <EmptyState
            title="Switch to mock-local first"
            description="Mock runs require the active runtime mode to be mock-local (HTTP 409 otherwise)."
            action={
              <Button onClick={() => void switchMode('mock-local')} disabled={switching || hasActive}>
                Switch to mock-local
              </Button>
            }
          />
        ) : (
          <div className="grid gap-3 md:grid-cols-3">
            <div>
              <label className="mb-1 block text-xs font-medium text-[var(--text-muted)]">
                Target weight (g)
              </label>
              <input
                value={targetWeightG}
                onChange={(e) => setTargetWeightG(e.target.value)}
                className="w-full rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2 text-sm outline-none focus:border-[var(--accent)]"
                inputMode="decimal"
              />
            </div>
            <div>
              <label className="mb-1 block text-xs font-medium text-[var(--text-muted)]">
                Tolerance (g, optional)
              </label>
              <input
                value={toleranceG}
                onChange={(e) => setToleranceG(e.target.value)}
                placeholder="default from mapping / 2%"
                className="w-full rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2 text-sm outline-none focus:border-[var(--accent)]"
                inputMode="decimal"
              />
            </div>
            <div>
              <label className="mb-1 block text-xs font-medium text-[var(--text-muted)]">
                Location code (optional)
              </label>
              <input
                value={locationCode}
                onChange={(e) => setLocationCode(e.target.value)}
                placeholder="ROBOT_LOCATION_TARGETS_JSON key"
                className="w-full rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2 text-sm outline-none focus:border-[var(--accent)]"
              />
            </div>
            <div className="md:col-span-3">
              <Button onClick={() => void startMockRun()} disabled={starting || hasActive}>
                {starting ? 'Starting…' : hasActive ? 'Run in progress' : 'Start mock run'}
              </Button>
            </div>
          </div>
        )}
      </section>

      {lastMock && (
        <section className="mt-4 rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--surface-1)] p-4 text-sm">
          <h3 className="mb-2 font-display text-sm font-semibold">Last mock start</h3>
          <div className="space-y-1 font-mono text-xs text-[var(--text-secondary)]">
            <div>event_id: {lastMock.event_id}</div>
            <div>weightment_id: {lastMock.weightment_id}</div>
            <div>
              run: {lastMock.run?.id ?? '—'} ({lastMock.run?.status ?? '—'})
            </div>
            {lastMock.contract && (
              <div>
                targets: {lastMock.contract.pickup_target_name} →{' '}
                {lastMock.contract.weigh_target_name} →{' '}
                {lastMock.contract.return_target_name} · tol{' '}
                {lastMock.contract.weight_tolerance_g} g
              </div>
            )}
          </div>
        </section>
      )}
    </div>
  )
}
