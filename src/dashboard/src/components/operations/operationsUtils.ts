export const TIMELINE_PHASES = [
  'Move To Scoop',
  'Scooping',
  'Move To Weigh',
  'Pouring',
  'Return To Scoop',
]

/** Lights-out BT phases: scoop → weigh scoop → pour → return (no transport_*). */
export const LIGHTSOUT_TIMELINE_PHASES = ['Scoop', 'Weigh Scoop', 'Pour', 'Return']

export function formatNumber(value: number | null | undefined, digits = 2): string {
  return typeof value === 'number' ? value.toFixed(digits) : '—'
}

export function statusLabel(state: string | null): string {
  if (!state) return 'Idle'
  if (state === 'awaiting_processing') return 'Awaiting Processing'
  return state.replace(/_/g, ' ').replace(/\b\w/g, (c) => c.toUpperCase())
}

export function formatPhaseLabel(phase: string | null | undefined): string {
  if (!phase) return '—'
  return phase
    .replace(/_/g, ' ')
    .replace(/\b\w/g, (c) => c.toUpperCase())
}

export function timelineIndexFromEvents(phases: string[], isDone: boolean): number {
  if (phases.length === 0) return isDone ? TIMELINE_PHASES.length : -1
  if (isDone) return TIMELINE_PHASES.length

  const normalized = phases.map((p) => p.toLowerCase())
  const last = normalized[normalized.length - 1] || ''
  const scoopEndIndex = normalized.indexOf('scoop_end')
  const pourStartIndex = normalized.indexOf('pour_start')
  const pourEndIndex = normalized.indexOf('pour_end')

  if (last === 'scoop_start' || last === 'scoop_end') return 1
  if (last === 'pour_start' || last === 'pour_end') return 3
  if (last === 'transport_start' || last === 'transport_end') {
    if (pourEndIndex >= 0) return 4
    if (scoopEndIndex >= 0 || pourStartIndex >= 0) return 2
    if (normalized.indexOf('scoop_start') >= 0) return 1
    return 0
  }
  return -1
}

/**
 * Lights-out timeline index:
 *   scoop_start → 0 (Scoop)
 *   scoop_end   → 1 (Weigh Scoop)
 *   pour_start  → 2 (Pour)
 *   pour_end    → 3 (Return)
 *   episode_end / isDone → complete (length)
 */
export function lightsoutTimelineIndex(
  phases: string[],
  isDone: boolean,
  episodeEnded = false
): number {
  if (isDone || episodeEnded) return LIGHTSOUT_TIMELINE_PHASES.length
  if (phases.length === 0) return -1

  const last = (phases[phases.length - 1] || '').toLowerCase()
  if (last === 'scoop_start') return 0
  if (last === 'scoop_end') return 1
  if (last === 'pour_start') return 2
  if (last === 'pour_end') return 3
  if (last === 'episode_end') return LIGHTSOUT_TIMELINE_PHASES.length
  return -1
}

export type EpisodePhaseKind = 'idle' | 'scooping' | 'weighing' | 'pouring' | 'returning' | 'done'

export function episodePhaseKind(
  livePhase: string | null | undefined,
  phaseIndex: number,
  timelineLen: number
): EpisodePhaseKind {
  const phase = (livePhase || '').toLowerCase()
  if (phaseIndex >= timelineLen && timelineLen > 0) return 'done'
  if (phase === 'scoop_start') return 'scooping'
  if (phase === 'scoop_end') return 'weighing'
  if (phase === 'pour_start') return 'pouring'
  if (phase === 'pour_end') return 'returning'
  if (phaseIndex < 0) return 'idle'
  return 'idle'
}

export type WeightBand = {
  scalePct: number
  targetPct: number
  toleranceBand: { left: number; width: number }
  errorG: number | null
  inBand: boolean
}

export function computeWeightBand(
  weight: number | null,
  targetWeightG: number | null,
  targetToleranceG: number | null
): WeightBand {
  if (typeof weight !== 'number' || typeof targetWeightG !== 'number') {
    return { scalePct: 0, targetPct: 0, toleranceBand: { left: 0, width: 0 }, errorG: null, inBand: false }
  }
  const maxRange = Math.max(targetWeightG * 1.4, 200)
  const errorG = weight - targetWeightG
  const inBand =
    typeof targetToleranceG === 'number'
      ? Math.abs(errorG) <= targetToleranceG
      : false
  const low = Math.max(0, targetWeightG - (targetToleranceG ?? 0))
  const high = Math.min(maxRange, targetWeightG + (targetToleranceG ?? 0))
  return {
    scalePct: Math.max(0, Math.min(100, (weight / maxRange) * 100)),
    targetPct: Math.max(0, Math.min(100, (targetWeightG / maxRange) * 100)),
    toleranceBand: {
      left: Math.max(0, Math.min(100, (low / maxRange) * 100)),
      width: Math.max(0, Math.min(100, ((high - low) / maxRange) * 100)),
    },
    errorG,
    inBand,
  }
}
