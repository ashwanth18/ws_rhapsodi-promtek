import {
  ArrowLeftRight,
  Droplets,
  Package,
  Scale,
  Truck,
} from 'lucide-react'
import GlassCard from '../GlassCard'
import StatusBadge from '../ui/StatusBadge'
import { TIMELINE_PHASES, formatPhaseLabel } from './operationsUtils'

const PHASE_ICONS = [
  <Truck key="t1" className="h-4 w-4" />,
  <Package key="t2" className="h-4 w-4" />,
  <Scale key="t3" className="h-4 w-4" />,
  <Droplets key="t4" className="h-4 w-4" />,
  <ArrowLeftRight key="t5" className="h-4 w-4" />,
]

type Props = {
  phaseIndex: number
  showLive: boolean
  pourPhaseLabel: string
  vibrationIntensity: number | null
  inclineAngleDeg: number | null
  pourTelemetryVisible: boolean
}

export default function PhasePanel({
  phaseIndex,
  showLive,
  pourPhaseLabel,
  vibrationIntensity,
  inclineAngleDeg,
  pourTelemetryVisible,
}: Props) {
  const currentLabel =
    phaseIndex < 0
      ? 'Idle'
      : phaseIndex >= TIMELINE_PHASES.length
        ? 'Complete'
        : TIMELINE_PHASES[phaseIndex]

  const progress =
    phaseIndex < 0
      ? 0
      : Math.max(0, Math.min(100, ((phaseIndex + 1) / TIMELINE_PHASES.length) * 100))

  return (
    <GlassCard>
      <div className="mb-4 flex items-center justify-between">
        <h3 className="font-display text-sm font-semibold uppercase tracking-wider text-[var(--text-faint)]">
          Robot Phase
        </h3>
        <StatusBadge
          label={showLive ? 'Live' : 'Last run'}
          tone={showLive ? 'good' : 'neutral'}
          pulse={showLive}
        />
      </div>

      <div className="mb-4 flex items-center gap-2">
        <span className="text-sm text-[var(--text-muted)]">Current</span>
        <span className="font-display text-base font-semibold text-[var(--text-primary)]">
          {currentLabel}
        </span>
      </div>

      <div className="grid grid-cols-5 gap-2">
        {TIMELINE_PHASES.map((phase, i) => {
          const completed = phaseIndex > i
          const active = phaseIndex === i
          return (
            <div
              key={phase}
              className={`rounded-[var(--radius-sm)] border px-2 py-3 text-center transition-colors ${
                completed
                  ? 'border-[var(--status-good-fg)]/30 bg-[var(--status-good-bg)]'
                  : active
                    ? 'border-[var(--status-warn-fg)]/40 bg-[var(--status-warn-bg)]'
                    : 'border-[var(--border)] bg-[var(--surface-2)]'
              }`}
            >
              <div className="mx-auto mb-1 flex justify-center text-[var(--text-secondary)]">
                {PHASE_ICONS[i]}
              </div>
              <div className="text-[10px] font-medium leading-tight text-[var(--text-muted)]">
                {phase}
              </div>
            </div>
          )
        })}
      </div>

      <div className="mt-4 h-1.5 overflow-hidden rounded-full bg-[var(--surface-strong)]">
        <div
          className="h-full rounded-full bg-[var(--accent)] transition-all duration-500"
          style={{ width: `${progress}%` }}
        />
      </div>

      <div className="mt-4 grid gap-3 sm:grid-cols-3">
        <div className="rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2">
          <div className="text-[10px] uppercase tracking-wider text-[var(--text-faint)]">Pour phase</div>
          <div className="mt-1 text-sm font-medium">
            {pourTelemetryVisible ? formatPhaseLabel(pourPhaseLabel) : '—'}
          </div>
        </div>
        <div className="rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2">
          <div className="text-[10px] uppercase tracking-wider text-[var(--text-faint)]">Vibration</div>
          <div className="mt-1 font-display text-sm font-semibold font-tabular">
            {pourTelemetryVisible && typeof vibrationIntensity === 'number'
              ? vibrationIntensity.toFixed(2)
              : '—'}
          </div>
        </div>
        <div className="rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2">
          <div className="text-[10px] uppercase tracking-wider text-[var(--text-faint)]">Incline</div>
          <div className="mt-1 font-display text-sm font-semibold font-tabular">
            {pourTelemetryVisible && typeof inclineAngleDeg === 'number'
              ? `${inclineAngleDeg.toFixed(1)}°`
              : '—'}
          </div>
        </div>
      </div>
    </GlassCard>
  )
}
