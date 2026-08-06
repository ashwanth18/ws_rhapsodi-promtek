import { LIGHTSOUT_TIMELINE_PHASES, TIMELINE_PHASES } from './operationsUtils'

export type WeightModel = 'fill' | 'scoopReturn'
export type OperationsFamily = 'mes' | 'mock' | 'lightsout'

export type OperationsProfile = {
  mode: string
  family: OperationsFamily
  weightModel: WeightModel
  phaseTopic: string
  activeTopic: string | null
  timelinePhases: string[]
  showMesStatus: boolean
  showBatchQueue: boolean
  showLightsoutPanels: boolean
  /** Fraction of target used as tolerance when set (lightsout = 0.02). */
  toleranceFrac: number | null
}

const DEFAULT_WEBHOOK_PHASE_TOPIC =
  (import.meta as any).env.VITE_WEBHOOK_PHASE_TOPIC || '/webhook_run/phase'
const LIGHTSOUT_PHASE_TOPIC =
  (import.meta as any).env.VITE_LIGHTSOUT_PHASE_TOPIC || '/lightsout_training/phase'
const WEBHOOK_ACTIVE_TOPIC =
  (import.meta as any).env.VITE_WEBHOOK_ACTIVE_TOPIC || '/webhook_run/active'
const LIGHTSOUT_ACTIVE_TOPIC =
  (import.meta as any).env.VITE_LIGHTSOUT_ACTIVE_TOPIC || '/lightsout_training/active'

export function operationsProfile(mode: string | undefined): OperationsProfile {
  const normalized = (mode || '').toLowerCase()

  if (normalized === 'lightsout') {
    return {
      mode: normalized,
      family: 'lightsout',
      weightModel: 'scoopReturn',
      phaseTopic: LIGHTSOUT_PHASE_TOPIC,
      activeTopic: LIGHTSOUT_ACTIVE_TOPIC,
      timelinePhases: LIGHTSOUT_TIMELINE_PHASES,
      showMesStatus: false,
      showBatchQueue: false,
      showLightsoutPanels: true,
      toleranceFrac: 0.02,
    }
  }

  if (normalized === 'mock-local') {
    return {
      mode: normalized,
      family: 'mock',
      weightModel: 'fill',
      phaseTopic: DEFAULT_WEBHOOK_PHASE_TOPIC,
      activeTopic: WEBHOOK_ACTIVE_TOPIC,
      timelinePhases: TIMELINE_PHASES,
      showMesStatus: false,
      showBatchQueue: true,
      showLightsoutPanels: false,
      toleranceFrac: null,
    }
  }

  // mes-condor, mes-generic, unknown → MES family defaults
  return {
    mode: normalized || '—',
    family: 'mes',
    weightModel: 'fill',
    phaseTopic: DEFAULT_WEBHOOK_PHASE_TOPIC,
    activeTopic: WEBHOOK_ACTIVE_TOPIC,
    timelinePhases: TIMELINE_PHASES,
    showMesStatus: true,
    showBatchQueue: true,
    showLightsoutPanels: false,
    toleranceFrac: null,
  }
}
