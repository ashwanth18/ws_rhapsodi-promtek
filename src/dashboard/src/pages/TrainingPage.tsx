import { useState } from 'react'
import { GraduationCap } from 'lucide-react'
import { toast } from 'sonner'
import GlassCard from '../components/GlassCard'
import Button from '../components/ui/button'
import StatusBadge from '../components/ui/StatusBadge'
import { EmptyState, SectionHeader } from '../components/ui/SectionHeader'
import RunSetupSheet from '../features/runSetup/RunSetupSheet'
import { useRuntimeConfig } from '../config/RuntimeConfig'
import { useRuntimeMode } from '../hooks/useRuntimeMode'

export default function TrainingPage() {
  const { apiBase } = useRuntimeConfig()
  const { runtime, capabilities, loading, error, refresh } = useRuntimeMode(apiBase)
  const [sheetOpen, setSheetOpen] = useState(false)
  const [switching, setSwitching] = useState(false)

  const lightsoutCapability = capabilities?.modes.find((m) => m.mode === 'lightsout')
  const isLightsoutMode = runtime?.mode === 'lightsout'

  const switchToLightsout = async () => {
    setSwitching(true)
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
      toast.success('Mode set to lightsout')
      await refresh()
    } catch (err) {
      toast.error(err instanceof Error ? err.message : 'Mode switch failed')
    } finally {
      setSwitching(false)
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
          description="This device does not advertise the lightsout capability."
        />
      </div>
    )
  }

  return (
    <div className="px-6 py-6">
      <SectionHeader
        title="Training"
        description="Configure and start lights-out training runs through the backend."
        action={
          <Button variant="outline" onClick={() => void refresh()} disabled={loading}>
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

      <GlassCard>
        <div className="flex flex-col gap-3">
          <div className="flex items-center gap-2">
            <GraduationCap className="h-4 w-4 text-[var(--accent)]" />
            <h3 className="text-lg font-semibold">Lights-Out Training</h3>
          </div>
          <p className="text-sm text-[var(--text-secondary)]">
            Powder catalog, target strategy, episode limits, and operator labels — submitted via{' '}
            <code className="font-mono text-xs">POST /modes/lightsout/runs</code>.
          </p>
          {!isLightsoutMode ? (
            <EmptyState
              title="Switch to lightsout first"
              description="Training runs require active mode lightsout."
              action={
                <Button onClick={() => void switchToLightsout()} disabled={switching}>
                  Switch to lightsout
                </Button>
              }
            />
          ) : (
            <Button onClick={() => setSheetOpen(true)}>Configure & start run</Button>
          )}
        </div>
      </GlassCard>

      <RunSetupSheet
        mode="lightsout"
        apiBase={apiBase}
        open={sheetOpen}
        onClose={() => setSheetOpen(false)}
        onStarted={() => void refresh()}
      />
    </div>
  )
}
