import GlassCard from '../GlassCard'
import StatusBadge, { type StatusTone } from '../ui/StatusBadge'
import { formatNumber } from './operationsUtils'
import type { LightsoutSessionRequest } from '../../hooks/useLightsoutLive'

type Props = {
  episode: number | null
  episodesTotal: number | null
  episodesCompleted: number
  totalPouredG: number | null
  scoopedMassG: number | null
  pourOutcome: string | null
  stopReason: string | null
  runState: string
  runActive: boolean
  session: LightsoutSessionRequest | null
  startedAt: number | null
  toleranceFrac: number
}

function outcomeTone(outcome: string | null): StatusTone {
  if (outcome === 'achieved') return 'good'
  if (outcome === 'overshoot') return 'bad'
  if (outcome === 'timeout') return 'warn'
  return 'idle'
}

function ProgressBar({
  label,
  valueLabel,
  pct,
}: {
  label: string
  valueLabel: string
  pct: number
}) {
  return (
    <div>
      <div className="mb-1 flex items-center justify-between text-xs">
        <span className="text-[var(--text-muted)]">{label}</span>
        <span className="font-tabular text-[var(--text-secondary)]">{valueLabel}</span>
      </div>
      <div className="h-2 overflow-hidden rounded-full bg-[var(--surface-strong)]">
        <div
          className="h-full rounded-full bg-[var(--accent)] transition-all"
          style={{ width: `${Math.max(0, Math.min(100, pct))}%` }}
        />
      </div>
    </div>
  )
}

export default function SessionProgressCard({
  episode,
  episodesTotal,
  episodesCompleted,
  totalPouredG,
  scoopedMassG,
  pourOutcome,
  stopReason,
  runState,
  runActive,
  session,
  startedAt,
  toleranceFrac,
}: Props) {
  const total = episodesTotal ?? session?.episodes ?? null
  const current = episode ?? episodesCompleted
  const episodePct =
    typeof total === 'number' && total > 0 && typeof current === 'number'
      ? (current / total) * 100
      : 0

  const stopOn = session?.stop_on || 'episodes'
  const stopValue = typeof session?.stop_value === 'number' ? session.stop_value : null

  let stopPct = 0
  let stopValueLabel = '—'
  if (stopOn === 'total_weight_g' && stopValue != null && stopValue > 0) {
    const poured = totalPouredG ?? 0
    stopPct = (poured / stopValue) * 100
    stopValueLabel = `${formatNumber(poured, 1)} / ${formatNumber(stopValue, 1)} g`
  } else if (stopOn === 'duration_min' && stopValue != null && stopValue > 0 && startedAt) {
    const elapsedMin = (Date.now() / 1000 - startedAt) / 60
    stopPct = (elapsedMin / stopValue) * 100
    stopValueLabel = `${formatNumber(elapsedMin, 1)} / ${formatNumber(stopValue, 1)} min`
  }

  const stopping =
    Boolean(stopReason) && (runState === 'running' || runState === 'starting')

  const chips: string[] = []
  if (session?.target_mode) chips.push(`target: ${session.target_mode}`)
  if (typeof session?.min_scooped_g === 'number') {
    chips.push(`min scoop ${Math.round(session.min_scooped_g)} g`)
  }
  if (session?.target_min_g && session.target_min_g > 0) {
    chips.push(`clamp ≥ ${session.target_min_g} g`)
  }
  if (session?.target_max_g && session.target_max_g > 0) {
    chips.push(`clamp ≤ ${session.target_max_g} g`)
  }
  if (session?.container_target) chips.push(`scoop @ ${session.container_target}`)
  if (session?.pour_target) chips.push(`pour @ ${session.pour_target}`)
  if (session?.enable_scoop === false) chips.push('scoop off')
  chips.push(`tol ±${Math.round(toleranceFrac * 100)}%`)

  return (
    <GlassCard>
      <div className="mb-4 flex flex-wrap items-center justify-between gap-2">
        <div>
          <h3 className="font-display text-sm font-semibold uppercase tracking-wider text-[var(--text-faint)]">
            Session Progress
          </h3>
          <p className="mt-1 text-xs text-[var(--text-muted)]">
            Cumulative poured is training throughput (returns to the same vessel), not yield.
          </p>
        </div>
        <div className="flex flex-wrap gap-2">
          {stopping && (
            <StatusBadge label="Stopping" tone="warn" pulse />
          )}
          {stopReason && !stopping && (
            <StatusBadge label={`Stopped: ${stopReason}`} tone="neutral" />
          )}
          {runActive && !stopping && (
            <StatusBadge label="Training" tone="good" pulse />
          )}
        </div>
      </div>

      <div className="grid gap-4 lg:grid-cols-2">
        <ProgressBar
          label="Episode cap"
          valueLabel={
            typeof total === 'number'
              ? `${current ?? 0} / ${total}`
              : current != null
                ? String(current)
                : '—'
          }
          pct={episodePct}
        />
        {stopOn !== 'episodes' && (
          <ProgressBar
            label={`Stop on ${stopOn}`}
            valueLabel={stopValueLabel}
            pct={stopPct}
          />
        )}
        {stopOn === 'episodes' && (
          <div className="rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2">
            <div className="text-[10px] uppercase tracking-wider text-[var(--text-faint)]">
              Total poured (throughput)
            </div>
            <div className="mt-1 font-display text-lg font-semibold font-tabular">
              {typeof totalPouredG === 'number' ? `${formatNumber(totalPouredG, 1)} g` : '—'}
            </div>
          </div>
        )}
      </div>

      {chips.length > 0 && (
        <div className="mt-4 flex flex-wrap gap-2">
          {chips.map((chip) => (
            <span
              key={chip}
              className="rounded-full border border-[var(--border)] bg-[var(--surface-2)] px-2.5 py-1 text-[11px] text-[var(--text-secondary)]"
            >
              {chip}
            </span>
          ))}
        </div>
      )}

      <div className="mt-4 grid gap-3 sm:grid-cols-3">
        <div className="rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2">
          <div className="text-[10px] uppercase tracking-wider text-[var(--text-faint)]">
            Last scooped
          </div>
          <div className="mt-1 font-display text-sm font-semibold font-tabular">
            {typeof scoopedMassG === 'number' ? `${formatNumber(scoopedMassG, 1)} g` : '—'}
          </div>
        </div>
        <div className="rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2">
          <div className="text-[10px] uppercase tracking-wider text-[var(--text-faint)]">
            Pour outcome
          </div>
          <div className="mt-1">
            <StatusBadge
              label={pourOutcome || '—'}
              tone={outcomeTone(pourOutcome)}
            />
          </div>
        </div>
        <div className="rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2">
          <div className="text-[10px] uppercase tracking-wider text-[var(--text-faint)]">
            Episodes done
          </div>
          <div className="mt-1 font-display text-sm font-semibold font-tabular">
            {episodesCompleted}
          </div>
        </div>
      </div>
    </GlassCard>
  )
}
