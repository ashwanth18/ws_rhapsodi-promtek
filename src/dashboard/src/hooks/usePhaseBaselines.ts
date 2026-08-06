import { useEffect, useMemo, useRef, useState } from 'react'

export type PhaseBaselines = {
  episodeBaselineG: number | null
  postScoopG: number | null
  pourBaselineG: number | null
  scoopedLiveG: number | null
  scoopedFinalG: number | null
  netPouredG: number | null
  reset: () => void
}

/**
 * Latch scale readings at BT phase markers, mirroring CaptureBaseline /
 * MeasureScoopedMass / pour_server tare on the client.
 *
 * Do not consume a phase marker until a finite weight sample is available;
 * otherwise scoop_start can be dropped when weight is briefly null.
 */
export function usePhaseBaselines(
  weight: number | null,
  livePhase: string | null
): PhaseBaselines {
  const [episodeBaselineG, setEpisodeBaselineG] = useState<number | null>(null)
  const [postScoopG, setPostScoopG] = useState<number | null>(null)
  const [pourBaselineG, setPourBaselineG] = useState<number | null>(null)
  const lastPhaseRef = useRef<string | null>(null)
  const weightRef = useRef<number | null>(weight)
  const episodeBaselineRef = useRef<number | null>(null)
  const postScoopRef = useRef<number | null>(null)

  weightRef.current = weight
  episodeBaselineRef.current = episodeBaselineG
  postScoopRef.current = postScoopG

  const reset = () => {
    setEpisodeBaselineG(null)
    setPostScoopG(null)
    setPourBaselineG(null)
    episodeBaselineRef.current = null
    postScoopRef.current = null
    lastPhaseRef.current = null
  }

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
    } else if (phase === 'scoop_end') {
      setPostScoopG(w)
      postScoopRef.current = w
      if (episodeBaselineRef.current == null) {
        setEpisodeBaselineG(w)
        episodeBaselineRef.current = w
      }
    } else if (phase === 'pour_start') {
      setPourBaselineG(w)
      if (postScoopRef.current == null) {
        setPostScoopG(w)
        postScoopRef.current = w
      }
    }
  }, [livePhase, weight])

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

  return {
    episodeBaselineG,
    postScoopG,
    pourBaselineG,
    scoopedLiveG,
    scoopedFinalG,
    netPouredG,
    reset,
  }
}
