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
 */
export function usePhaseBaselines(
  weight: number | null,
  livePhase: string | null
): PhaseBaselines {
  const [episodeBaselineG, setEpisodeBaselineG] = useState<number | null>(null)
  const [postScoopG, setPostScoopG] = useState<number | null>(null)
  const [pourBaselineG, setPourBaselineG] = useState<number | null>(null)
  const lastPhaseRef = useRef<string | null>(null)

  const reset = () => {
    setEpisodeBaselineG(null)
    setPostScoopG(null)
    setPourBaselineG(null)
    lastPhaseRef.current = null
  }

  useEffect(() => {
    if (!livePhase) return
    const phase = livePhase.toLowerCase()
    if (phase === lastPhaseRef.current) return
    lastPhaseRef.current = phase

    if (typeof weight !== 'number' || !Number.isFinite(weight)) return

    if (phase === 'scoop_start') {
      setEpisodeBaselineG(weight)
      setPostScoopG(null)
      setPourBaselineG(null)
    } else if (phase === 'scoop_end') {
      setPostScoopG(weight)
      if (episodeBaselineG == null) setEpisodeBaselineG(weight)
    } else if (phase === 'pour_start') {
      setPourBaselineG(weight)
      if (postScoopG == null) setPostScoopG(weight)
    }
  }, [livePhase, weight, episodeBaselineG, postScoopG])

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
