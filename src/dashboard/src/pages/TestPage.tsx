import { useCallback, useEffect, useMemo, useState } from 'react'
import { FlaskConical } from 'lucide-react'
import { toast } from 'sonner'
import Button from '../components/ui/button'
import StatusBadge from '../components/ui/StatusBadge'
import { EmptyState, SectionHeader } from '../components/ui/SectionHeader'
import Select from '../components/ui/select'
import RunSetupSheet from '../features/runSetup/RunSetupSheet'
import { useRuntimeConfig } from '../config/RuntimeConfig'
import { useRuntimeMode } from '../hooks/useRuntimeMode'

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
  const { runtime, capabilities, loading, error, refresh } = useRuntimeMode(apiBase)
  const [activeRun, setActiveRun] = useState<ActiveRobotRun | null>(null)
  const [sheetOpen, setSheetOpen] = useState(false)
  const [switching, setSwitching] = useState(false)
  const [selectedMode, setSelectedMode] = useState('mock-local')

  const mockCapability = useMemo(
    () => capabilities?.modes.find((m) => m.mode === 'mock-local') ?? null,
    [capabilities]
  )
  const isMockMode = runtime?.mode === 'mock-local'
  const hasActive =
    !!activeRun &&
    ['starting', 'running', 'awaiting_processing'].includes(activeRun.status ?? '')

  const loadActiveRun = useCallback(async () => {
    const res = await fetch(`${apiBase}/robot_weightment_runs/active`)
    if (!res.ok) throw new Error('Failed to load active robot run')
    const json = await res.json()
    setActiveRun(json.active || null)
  }, [apiBase])

  const refreshAll = useCallback(async () => {
    await Promise.all([refresh(), loadActiveRun()])
  }, [loadActiveRun, refresh])

  useEffect(() => {
    void refreshAll()
  }, [refreshAll])

  useEffect(() => {
    if (runtime?.mode) setSelectedMode(runtime.mode)
  }, [runtime?.mode])

  useEffect(() => {
    if (!hasActive && !isMockMode) return
    const id = window.setInterval(() => {
      void loadActiveRun()
      void refresh()
    }, 2000)
    return () => window.clearInterval(id)
  }, [hasActive, isMockMode, loadActiveRun, refresh])

  const switchMode = async (mode: string) => {
    setSwitching(true)
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
      setSelectedMode(json.mode)
      toast.success(`Mode set to ${json.mode}`)
      await refreshAll()
    } catch (err) {
      toast.error(err instanceof Error ? err.message : 'Mode switch failed')
    } finally {
      setSwitching(false)
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
          description="This device does not advertise the mock-local capability."
        />
      </div>
    )
  }

  return (
    <div className="px-5 py-5 lg:px-6">
      <SectionHeader
        title="Test"
        description="Operator-triggered mock-local weighment — no Condor traffic."
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
          Uses powder catalog + operator labels via{' '}
          <code className="font-mono">POST /modes/mock/runs</code>.
        </p>
        {!isMockMode ? (
          <EmptyState
            title="Switch to mock-local first"
            description="Mock runs require active mode mock-local."
            action={
              <Button
                onClick={() => void switchMode('mock-local')}
                disabled={switching || hasActive}
              >
                Switch to mock-local
              </Button>
            }
          />
        ) : (
          <Button onClick={() => setSheetOpen(true)} disabled={hasActive}>
            {hasActive ? 'Run in progress' : 'Configure & start mock run'}
          </Button>
        )}
      </section>

      <RunSetupSheet
        mode="mock-local"
        apiBase={apiBase}
        open={sheetOpen}
        onClose={() => setSheetOpen(false)}
        onStarted={() => void refreshAll()}
      />
    </div>
  )
}
