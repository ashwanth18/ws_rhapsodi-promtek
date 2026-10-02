import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import {
  bindPourOriginKey,
  getPourOriginG,
  latchPourOriginG,
} from './pourOriginStore'

export type PhaseBaselines = {
  episodeBaselineG: number | null
  postScoopG: number | null
  pourBaselineG: number | null
  /** First pour_start weight of this weightment — cumulative poured origin. */
  weightmentPourOriginG: number | null
  scoopedLiveG: number | null
  scoopedFinalG: number | null
  netPouredG: number | null
  /** Vessel − first pour baseline (survives rescoop segment re-tares). */
  cumulativePouredG: number | null
  /**
   * Bind a new weightment/run key. Clears origins only when the key changes.
   * Pass null to hard-clear (mode switch).
   */
  reset: (weightmentKey?: string | null) => void
}

/**
 * Latch scale readings at BT phase markers, mirroring CaptureBaseline /
 * MeasureScoopedMass / pour_server tare on the client.
 *
 * Do not consume a phase marker until a finite weight sample is available;
 * otherwise scoop_start can be dropped when weight is briefly null.
 *
 * Rescoop note: each pour_start re-tares pour_server (and pourBaselineG) so
 * segment net goes to 0. weightmentPourOriginG is latched only once per
 * weightment key (module store) so the MES hero keeps cumulative poured.
 */
export function usePhaseBaselines(
  weight: number | null,
  livePhase: string | null,
  weightmentKey: string | null = null
): PhaseBaselines {
  const [episodeBaselineG, setEpisodeBaselineG] = useState<number | null>(null)
  const [postScoopG, setPostScoopG] = useState<number | null>(null)
  const [pourBaselineG, setPourBaselineG] = useState<number | null>(null)
  const [weightmentPourOriginG, setWeightmentPourOriginG] = useState<number | null>(
    () => (weightmentKey ? getPourOriginG() : null)
  )
  const lastPhaseRef = useRef<string | null>(null)
  const weightRef = useRef<number | null>(weight)
  const episodeBaselineRef = useRef<number | null>(null)
  const postScoopRef = useRef<number | null>(null)

  weightRef.current = weight
  episodeBaselineRef.current = episodeBaselineG
  postScoopRef.current = postScoopG

  // Keep React state aligned with the durable store when key binds / remounts.
  useEffect(() => {
    if (!weightmentKey) return
    bindPourOriginKey(weightmentKey)
    const stored = getPourOriginG()
    setWeightmentPourOriginG(stored)
  }, [weightmentKey])

  const reset = useCallback((nextKey?: string | null) => {
    setEpisodeBaselineG(null)
    setPostScoopG(null)
    setPourBaselineG(null)
    episodeBaselineRef.current = null
    postScoopRef.current = null
    lastPhaseRef.current = null

    if (nextKey == null) {
      // Remount / mode-family effect. Do not drop the weightment origin —
      // the next rescoop pour_start would latch the vessel and the hero
      // would show 0.
      setWeightmentPourOriginG(getPourOriginG())
      return
    }
    const changed = bindPourOriginKey(nextKey)
    if (changed) {
      setWeightmentPourOriginG(null)
    } else {
      setWeightmentPourOriginG(getPourOriginG())
    }
  }, [])

  useEffect(() => {
    if (!livePhase) return
    const phase = livePhase.toLowerCase()
    if (phase === lastPhaseRef.current) return

    const w = weightRef.current
    if (typeof w !== 'number' || !Number.isFinite(w)) {
      // Retry when weight arrives; do not consume the phase yet.
      return
    }

    lastPhaseRef.current = phase

    if (phase === 'scoop_start') {
      setEpisodeBaselineG(w)
      episodeBaselineRef.current = w
      setPostScoopG(null)
      postScoopRef.current = null
      setPourBaselineG(null)
      // Keep weightmentPourOriginG across rescoop scoops.
    } else if (phase === 'scoop_end') {
      setPostScoopG(w)
      postScoopRef.current = w
      if (episodeBaselineRef.current == null) {
        setEpisodeBaselineG(w)
        episodeBaselineRef.current = w
      }
    } else if (phase === 'pour_start') {
      setPourBaselineG(w)
      // Bind first so a known weightment key is attached, then latch.
      // latch never raises an existing origin, so a rescoop pour_start at
      // ~400 g cannot zero the hero.
      if (weightmentKey) {
        bindPourOriginKey(weightmentKey)
      }
      const origin = latchPourOriginG(w)
      setWeightmentPourOriginG(origin)
      if (postScoopRef.current == null) {
        setPostScoopG(w)
        postScoopRef.current = w
      }
    }
  }, [livePhase, weight, weightmentKey])

  const scoopedLiveG = useMemo(() => {
    if (typeof weight !== 'number' || episodeBaselineG == null) return null
    return Math.max(0, episodeBaselineG - weight)
  }, [weight, episodeBaselineG])

  const scoopedFinalG = useMemo(() => {
    if (episodeBaselineG == null || postScoopG == null) return null
    return Math.max(0, episodeBaselineG - postScoopG)
  }, [episodeBaselineG, postScoopG])

  const netPouredG = useMemo(() => {
    if (typeof weight !== 'number' || pourBaselineG == null) return null
    return Math.max(0, weight - pourBaselineG)
  }, [weight, pourBaselineG])

  const cumulativePouredG = useMemo(() => {
    if (typeof weight !== 'number' || weightmentPourOriginG == null) return null
    return Math.max(0, weight - weightmentPourOriginG)
  }, [weight, weightmentPourOriginG])

  return {
    episodeBaselineG,
    postScoopG,
    pourBaselineG,
    weightmentPourOriginG,
    scoopedLiveG,
    scoopedFinalG,
    netPouredG,
    cumulativePouredG,
    reset,
  }
}
