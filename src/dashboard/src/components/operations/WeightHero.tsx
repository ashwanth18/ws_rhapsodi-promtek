import StatusBadge from '../ui/StatusBadge'
import GlassCard from '../GlassCard'
import { computeWeightBand, formatNumber } from './operationsUtils'

type Props = {
  weight: number | null
  targetWeightG: number | null
  targetToleranceG: number | null
  weightStale: boolean
  webhookActive: boolean
  /** Caption under the progress bar (default: Target band ±tol). */
  bandCaption?: string
}

export default function WeightHero({
  weight,
  targetWeightG,
  targetToleranceG,
  weightStale,
  webhookActive,
  bandCaption,
}: Props) {
  const band = computeWeightBand(weight, targetWeightG, targetToleranceG)
  const fillColor = band.inBand
    ? 'bg-[var(--status-good-fg)]'
    : typeof band.errorG === 'number' && band.errorG > 0
      ? 'bg-[var(--status-bad-fg)]'
      : 'bg-[var(--accent)]'

  return (
    <GlassCard className="h-full">
      <div className="flex items-start justify-between gap-3">
        <div>
          <div className="text-xs font-semibold uppercase tracking-[0.16em] text-[var(--text-faint)]">
            Live Weight
          </div>
          <div className="mt-2 font-display text-5xl font-bold font-tabular tracking-tight md:text-6xl">
            {typeof weight === 'number' ? Math.round(weight) : '—'}
            <span className="ml-2 text-2xl font-medium text-[var(--text-muted)]">g</span>
          </div>
        </div>
        <div className="flex flex-col items-end gap-2">
          <StatusBadge
            label={webhookActive ? 'Run active' : 'Idle'}
            tone={webhookActive ? 'good' : 'idle'}
            pulse={webhookActive}
          />
          <StatusBadge
            label={weightStale ? 'Scale offline' : 'Scale live'}
            tone={weightStale ? 'idle' : 'info'}
            pulse={!weightStale}
          />
        </div>
      </div>

      <div className="mt-6 grid gap-3 sm:grid-cols-3">
        <div className="rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2">
          <div className="text-[10px] uppercase tracking-wider text-[var(--text-faint)]">Target</div>
          <div className="mt-1 font-display text-lg font-semibold font-tabular">
            {typeof targetWeightG === 'number' ? Math.round(targetWeightG) : '—'} g
          </div>
        </div>
        <div className="rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2">
          <div className="text-[10px] uppercase tracking-wider text-[var(--text-faint)]">Error</div>
          <div
            className={`mt-1 font-display text-lg font-semibold font-tabular ${
              band.inBand
                ? 'text-[var(--status-good-fg)]'
                : typeof band.errorG === 'number' && Math.abs(band.errorG) > (targetToleranceG ?? 0)
                  ? 'text-[var(--status-bad-fg)]'
                  : 'text-[var(--text-primary)]'
            }`}
          >
            {typeof band.errorG === 'number'
              ? `${band.errorG >= 0 ? '+' : ''}${formatNumber(band.errorG, 1)} g`
              : '—'}
          </div>
        </div>
        <div className="rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2">
          <div className="text-[10px] uppercase tracking-wider text-[var(--text-faint)]">Tolerance</div>
          <div className="mt-1 font-display text-lg font-semibold font-tabular">
            {typeof targetToleranceG === 'number'
              ? `±${formatNumber(targetToleranceG, 1)} g`
              : '—'}
          </div>
        </div>
      </div>

      <div className="mt-5">
        <div className="relative h-3 overflow-hidden rounded-full bg-[var(--surface-strong)]">
          <div
            className={`absolute inset-y-0 left-0 rounded-full transition-all duration-300 ${fillColor}`}
            style={{ width: `${band.scalePct}%` }}
          />
          <div
            className="absolute inset-y-0 rounded bg-[var(--status-good-bg)] opacity-80"
            style={{
              left: `${band.toleranceBand.left}%`,
              width: `${band.toleranceBand.width}%`,
            }}
          />
          <div
            className="absolute inset-y-0 w-0.5 bg-[var(--text-primary)]"
            style={{ left: `${band.targetPct}%` }}
          />
        </div>
        <div className="mt-2 flex justify-between text-xs text-[var(--text-faint)]">
          <span>0 g</span>
          <span>
            {bandCaption ||
              (typeof targetToleranceG === 'number'
                ? `Target band ±${formatNumber(targetToleranceG, 1)} g`
                : 'Target band')}
          </span>
          <span>
            {typeof targetWeightG === 'number'
              ? `${Math.round(targetWeightG * 1.4)} g`
              : '—'}
          </span>
        </div>
      </div>
    </GlassCard>
  )
}
