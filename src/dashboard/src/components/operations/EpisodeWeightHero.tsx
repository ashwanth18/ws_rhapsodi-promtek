import StatusBadge from '../ui/StatusBadge'
import GlassCard from '../GlassCard'
import { computeWeightBand, formatNumber, type EpisodePhaseKind } from './operationsUtils'

type Props = {
  weight: number | null
  weightStale: boolean
  runActive: boolean
  phaseKind: EpisodePhaseKind
  /** Scoop-return model: scooped vs floor during scoop; net poured vs target during pour. */
  scoopedG: number | null
  minScoopedG: number | null
  netPouredG: number | null
  targetWeightG: number | null
  targetToleranceG: number | null
  targetPending?: boolean
  /** Absolute vessel reading shown as secondary. */
  showVesselReadout?: boolean
  bandCaption?: string
}

export default function EpisodeWeightHero({
  weight,
  weightStale,
  runActive,
  phaseKind,
  scoopedG,
  minScoopedG,
  netPouredG,
  targetWeightG,
  targetToleranceG,
  targetPending = false,
  showVesselReadout = true,
  bandCaption,
}: Props) {
  const scooping = phaseKind === 'scooping'
  const pouring = phaseKind === 'pouring'
  const weighing = phaseKind === 'weighing'

  let headline: string | number = '—'
  let headlineUnit = 'g'
  let headlineLabel = 'Live'
  let reason: string | null = null
  let barPct = 0
  let fillColor = 'bg-[var(--accent)]'
  let secondaryCards: { label: string; value: string }[] = []

  if (scooping || weighing) {
    // Scooped is always the headline during scoop/weigh. targetPending only
    // annotates that the pour target is sampled after scoop (non-fixed modes).
    headlineLabel = scooping ? 'Scooped' : 'Scooped (settled)'
    if (typeof scoopedG === 'number') {
      headline = Math.round(scoopedG)
      const floor = minScoopedG ?? 0
      const maxRange = Math.max(floor * 1.5, scoopedG * 1.2, 40)
      barPct = Math.max(0, Math.min(100, (scoopedG / maxRange) * 100))
      fillColor =
        floor > 0 && scoopedG >= floor
          ? 'bg-[var(--status-good-fg)]'
          : 'bg-[var(--accent)]'
      secondaryCards = [
        {
          label: 'Min scoop floor',
          value: typeof minScoopedG === 'number' ? `${Math.round(minScoopedG)} g` : '—',
        },
        {
          label: 'Status',
          value:
            typeof minScoopedG === 'number' && scoopedG >= minScoopedG
              ? 'Above floor'
              : 'Building',
        },
        {
          label: 'Pour target',
          value: targetPending
            ? 'After scoop'
            : typeof targetWeightG === 'number'
              ? `${Math.round(targetWeightG)} g`
              : '—',
        },
      ]
      if (targetPending) {
        reason = 'Pour target sampled after scoop'
      }
    } else {
      reason = 'Waiting for scoop baseline'
      secondaryCards = [
        {
          label: 'Min scoop floor',
          value: typeof minScoopedG === 'number' ? `${Math.round(minScoopedG)} g` : '—',
        },
        {
          label: 'Pour target',
          value: targetPending ? 'After scoop' : '—',
        },
        {
          label: 'Scale',
          value: 'Falls during scoop',
        },
      ]
    }
  } else if (pouring) {
    headlineLabel = 'Poured'
    if (typeof netPouredG === 'number') {
      headline = Math.round(netPouredG)
      const band = computeWeightBand(netPouredG, targetWeightG, targetToleranceG)
      barPct = band.scalePct
      fillColor = band.inBand
        ? 'bg-[var(--status-good-fg)]'
        : typeof band.errorG === 'number' && band.errorG > 0
          ? 'bg-[var(--status-bad-fg)]'
          : 'bg-[var(--accent)]'
      secondaryCards = [
        {
          label: 'Target',
          value: typeof targetWeightG === 'number' ? `${Math.round(targetWeightG)} g` : '—',
        },
        {
          label: 'Error',
          value:
            typeof band.errorG === 'number'
              ? `${band.errorG >= 0 ? '+' : ''}${formatNumber(band.errorG, 1)} g`
              : '—',
        },
        {
          label: 'Scooped',
          value: typeof scoopedG === 'number' ? `${Math.round(scoopedG)} g` : '—',
        },
      ]
    } else {
      reason = 'Waiting for pour baseline'
      secondaryCards = [
        {
          label: 'Target',
          value: typeof targetWeightG === 'number' ? `${Math.round(targetWeightG)} g` : '—',
        },
        {
          label: 'Scooped',
          value: typeof scoopedG === 'number' ? `${Math.round(scoopedG)} g` : '—',
        },
        {
          label: 'Tolerance',
          value:
            typeof targetToleranceG === 'number'
              ? `±${formatNumber(targetToleranceG, 1)} g`
              : '—',
        },
      ]
    }
  } else if (!runActive) {
    headlineLabel = 'Live weight'
    if (typeof weight === 'number') {
      headline = Math.round(weight)
    }
    reason = 'Idle'
    secondaryCards = [
      {
        label: 'Target',
        value: typeof targetWeightG === 'number' ? `${Math.round(targetWeightG)} g` : '—',
      },
      {
        label: 'Scooped',
        value: typeof scoopedG === 'number' ? `${Math.round(scoopedG)} g` : '—',
      },
      {
        label: 'Tolerance',
        value:
          typeof targetToleranceG === 'number'
            ? `±${formatNumber(targetToleranceG, 1)} g`
            : '—',
      },
    ]
  } else {
    headlineLabel = phaseKind === 'returning' ? 'Returning' : 'Live'
    if (typeof netPouredG === 'number') {
      headline = Math.round(netPouredG)
      headlineLabel = 'Last poured'
    } else if (typeof scoopedG === 'number') {
      headline = Math.round(scoopedG)
      headlineLabel = 'Scooped'
    } else if (typeof weight === 'number') {
      headline = Math.round(weight)
    }
    secondaryCards = [
      {
        label: 'Target',
        value: typeof targetWeightG === 'number' ? `${Math.round(targetWeightG)} g` : '—',
      },
      {
        label: 'Scooped',
        value: typeof scoopedG === 'number' ? `${Math.round(scoopedG)} g` : '—',
      },
      {
        label: 'Tolerance',
        value:
          typeof targetToleranceG === 'number'
            ? `±${formatNumber(targetToleranceG, 1)} g`
            : '—',
      },
    ]
  }

  const caption =
    bandCaption ||
    (pouring && typeof targetToleranceG === 'number' && typeof targetWeightG === 'number'
      ? `Target band ±${formatNumber(targetToleranceG, 1)} g`
      : scooping || weighing
        ? 'Scoop floor (not a tolerance band)'
        : '—')

  return (
    <GlassCard className="h-full">
      <div className="flex items-start justify-between gap-3">
        <div>
          <div className="text-xs font-semibold uppercase tracking-[0.16em] text-[var(--text-faint)]">
            {headlineLabel}
          </div>
          <div className="mt-2 font-display text-5xl font-bold font-tabular tracking-tight md:text-6xl">
            {headline}
            <span className="ml-2 text-2xl font-medium text-[var(--text-muted)]">
              {headlineUnit}
            </span>
          </div>
          {reason && (
            <div className="mt-1 text-xs text-[var(--text-muted)]">{reason}</div>
          )}
        </div>
        <div className="flex flex-col items-end gap-2">
          <StatusBadge
            label={runActive ? 'Run active' : 'Idle'}
            tone={runActive ? 'good' : 'idle'}
            pulse={runActive}
          />
          <StatusBadge
            label={weightStale ? 'Scale offline' : 'Scale live'}
            tone={weightStale ? 'idle' : 'info'}
            pulse={!weightStale}
          />
        </div>
      </div>

      {secondaryCards.length > 0 && (
        <div className="mt-6 grid gap-3 sm:grid-cols-3">
          {secondaryCards.map((card) => (
            <div
              key={card.label}
              className="rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2"
            >
              <div className="text-[10px] uppercase tracking-wider text-[var(--text-faint)]">
                {card.label}
              </div>
              <div className="mt-1 font-display text-lg font-semibold font-tabular">
                {card.value}
              </div>
            </div>
          ))}
        </div>
      )}

      {(scooping || weighing || pouring) && (
        <div className="mt-5">
          <div className="relative h-3 overflow-hidden rounded-full bg-[var(--surface-strong)]">
            <div
              className={`absolute inset-y-0 left-0 rounded-full transition-all duration-300 ${fillColor}`}
              style={{ width: `${barPct}%` }}
            />
            {pouring &&
              typeof targetWeightG === 'number' &&
              typeof targetToleranceG === 'number' &&
              (() => {
                const band = computeWeightBand(netPouredG, targetWeightG, targetToleranceG)
                return (
                  <>
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
                  </>
                )
              })()}
            {(scooping || weighing) &&
              typeof minScoopedG === 'number' &&
              (() => {
                const maxRange = Math.max(minScoopedG * 1.5, (scoopedG ?? 0) * 1.2, 40)
                const floorPct = Math.max(
                  0,
                  Math.min(100, (minScoopedG / maxRange) * 100)
                )
                return (
                  <div
                    className="absolute inset-y-0 w-0.5 bg-[var(--text-primary)]"
                    style={{ left: `${floorPct}%` }}
                  />
                )
              })()}
          </div>
          <div className="mt-2 flex justify-between text-xs text-[var(--text-faint)]">
            <span>0 g</span>
            <span>{caption}</span>
            <span>
              {pouring && typeof targetWeightG === 'number'
                ? `${Math.round(targetWeightG * 1.4)} g`
                : scooping || weighing
                  ? typeof minScoopedG === 'number'
                    ? `${Math.round(minScoopedG * 1.5)} g`
                    : '—'
                  : '—'}
            </span>
          </div>
        </div>
      )}

      {showVesselReadout && typeof weight === 'number' && (
        <div className="mt-4 text-xs text-[var(--text-muted)]">
          Vessel <span className="font-tabular text-[var(--text-secondary)]">{Math.round(weight)} g</span>
          <span className="ml-2 text-[var(--text-faint)]">(absolute scale)</span>
        </div>
      )}
    </GlassCard>
  )
}
