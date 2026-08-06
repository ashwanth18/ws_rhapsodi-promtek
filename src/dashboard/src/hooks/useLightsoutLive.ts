import { useEffect, useMemo, useRef, useState } from 'react'
import { useRos } from '../ros/RosContext'
import { ROSLIB } from '../ros/roslib'

const PREFIX =
  (import.meta as any).env.VITE_LIGHTSOUT_TOPIC_PREFIX || '/lightsout_training'

export type LightsoutSessionRequest = {
  powder_id?: string
  powder_name?: string
  container_target?: string
  pour_target?: string
  target_weight_g?: number
  episodes?: number
  batch_id?: string
  enable_scoop?: boolean
  lot_code?: string
  operator?: string
  notes?: string
  stop_on?: string
  stop_value?: number
  target_mode?: string
  target_fractions?: number[]
  min_scooped_g?: number
  target_min_g?: number
  target_max_g?: number
  layout_id?: string
  layout_hash?: string
  [key: string]: unknown
}

export type LightsoutSessionApi = {
  active: boolean
  started_at: number | null
  session: LightsoutSessionRequest | null
  episodes: { completed: number }
  tolerance_frac: number
}

export type LightsoutLive = {
  topicActive: boolean
  episode: number | null
  episodesTotal: number | null
  episodeEnd: number | null
  targetWeightG: number | null
  scoopedMassG: number | null
  totalPouredG: number | null
  pourOutcome: string | null
  stopReason: string | null
  runId: string | null
  batchId: string | null
  powderId: string | null
  ingredientId: string | null
  lotCode: string | null
  operator: string | null
  notes: string | null
  sessionApi: LightsoutSessionApi | null
  /** True when latched target should not be shown (pre-pour, non-fixed). */
  targetPending: boolean
  /** Combined active: topic + run_state running. */
  runActive: boolean
}

const IDLE_SESSION: LightsoutSessionApi = {
  active: false,
  started_at: null,
  session: null,
  episodes: { completed: 0 },
  tolerance_frac: 0.02,
}

export function useLightsoutLive(
  enabled: boolean,
  apiBase: string,
  runState: string,
  livePhase: string | null
): LightsoutLive {
  const ros = useRos()
  const [topicActive, setTopicActive] = useState(false)
  const [episode, setEpisode] = useState<number | null>(null)
  const [episodesTotal, setEpisodesTotal] = useState<number | null>(null)
  const [episodeEnd, setEpisodeEnd] = useState<number | null>(null)
  const [targetWeightG, setTargetWeightG] = useState<number | null>(null)
  const [scoopedMassG, setScoopedMassG] = useState<number | null>(null)
  const [totalPouredG, setTotalPouredG] = useState<number | null>(null)
  const [pourOutcome, setPourOutcome] = useState<string | null>(null)
  const [stopReason, setStopReason] = useState<string | null>(null)
  const [runId, setRunId] = useState<string | null>(null)
  const [batchId, setBatchId] = useState<string | null>(null)
  const [powderId, setPowderId] = useState<string | null>(null)
  const [ingredientId, setIngredientId] = useState<string | null>(null)
  const [lotCode, setLotCode] = useState<string | null>(null)
  const [operator, setOperator] = useState<string | null>(null)
  const [notes, setNotes] = useState<string | null>(null)
  const [sessionApi, setSessionApi] = useState<LightsoutSessionApi | null>(null)

  const targetEpisodeRef = useRef<number | null>(null)
  const lastTargetRef = useRef<number | null>(null)
  const episodeRef = useRef<number | null>(null)

  useEffect(() => {
    episodeRef.current = episode
  }, [episode])

  useEffect(() => {
    if (!enabled || !ros) return

    const subs: { unsubscribe: () => void }[] = []
    const sub = <T>(name: string, messageType: string, onMsg: (msg: T) => void) => {
      const topic = new ROSLIB.Topic({ ros, name, messageType })
      topic.subscribe(onMsg as (msg: unknown) => void)
      subs.push(topic)
    }

    sub<{ data: boolean }>(`${PREFIX}/active`, 'std_msgs/Bool', (msg) => {
      setTopicActive(Boolean(msg.data))
    })
    sub<{ data: number }>(`${PREFIX}/episode`, 'std_msgs/Int32', (msg) => {
      if (typeof msg.data === 'number') setEpisode(msg.data)
    })
    sub<{ data: number }>(`${PREFIX}/episodes_total`, 'std_msgs/Int32', (msg) => {
      if (typeof msg.data === 'number') setEpisodesTotal(msg.data)
    })
    sub<{ data: number }>(`${PREFIX}/episode_end`, 'std_msgs/Int32', (msg) => {
      if (typeof msg.data === 'number') setEpisodeEnd(msg.data)
    })
    sub<{ data: number }>(`${PREFIX}/target_weight_g`, 'std_msgs/Float64', (msg) => {
      if (typeof msg.data === 'number' && Number.isFinite(msg.data)) {
        setTargetWeightG(msg.data)
        if (lastTargetRef.current !== msg.data) {
          lastTargetRef.current = msg.data
          targetEpisodeRef.current = episodeRef.current
        }
      }
    })
    sub<{ data: number }>(`${PREFIX}/scooped_mass_g`, 'std_msgs/Float32', (msg) => {
      if (typeof msg.data === 'number' && Number.isFinite(msg.data)) {
        setScoopedMassG(msg.data)
      }
    })
    sub<{ data: number }>(`${PREFIX}/total_poured_g`, 'std_msgs/Float32', (msg) => {
      if (typeof msg.data === 'number' && Number.isFinite(msg.data)) {
        setTotalPouredG(msg.data)
      }
    })
    sub<{ data: string }>(`${PREFIX}/pour_outcome`, 'std_msgs/String', (msg) => {
      setPourOutcome(msg.data || null)
    })
    sub<{ data: string }>(`${PREFIX}/stop_reason`, 'std_msgs/String', (msg) => {
      setStopReason(msg.data || null)
    })
    sub<{ data: string }>(`${PREFIX}/run_id`, 'std_msgs/String', (msg) => {
      setRunId(msg.data || null)
    })
    sub<{ data: string }>(`${PREFIX}/batch_id`, 'std_msgs/String', (msg) => {
      setBatchId(msg.data || null)
    })
    sub<{ data: string }>(`${PREFIX}/powder_id`, 'std_msgs/String', (msg) => {
      setPowderId(msg.data || null)
    })
    sub<{ data: string }>(`${PREFIX}/ingredient_id`, 'std_msgs/String', (msg) => {
      setIngredientId(msg.data || null)
    })
    sub<{ data: string }>(`${PREFIX}/lot_code`, 'std_msgs/String', (msg) => {
      setLotCode(msg.data || null)
    })
    sub<{ data: string }>(`${PREFIX}/operator`, 'std_msgs/String', (msg) => {
      setOperator(msg.data || null)
    })
    sub<{ data: string }>(`${PREFIX}/notes`, 'std_msgs/String', (msg) => {
      setNotes(msg.data || null)
    })

    return () => subs.forEach((t) => t.unsubscribe())
    // episode intentionally omitted from deps — used only inside target callback
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [enabled, ros])

  useEffect(() => {
    if (!enabled) {
      setSessionApi(null)
      return
    }
    let cancelled = false
    async function poll() {
      try {
        const res = await fetch(`${apiBase}/modes/lightsout/session`, {
          cache: 'no-store',
        })
        if (!res.ok) return
        const json = (await res.json()) as LightsoutSessionApi
        if (!cancelled) setSessionApi(json)
      } catch {
        if (!cancelled) setSessionApi(IDLE_SESSION)
      }
    }
    poll()
    const id = setInterval(poll, 5000)
    return () => {
      cancelled = true
      clearInterval(id)
    }
  }, [enabled, apiBase])

  // When a new episode starts, clear stale pour outcome / stop until republished.
  useEffect(() => {
    if (episode == null) return
    if (targetEpisodeRef.current != null && targetEpisodeRef.current !== episode) {
      // Target from previous episode until SampleTargetWeight republishes.
      // Keep targetWeightG but mark pending via targetPending below.
    }
  }, [episode])

  const targetMode = sessionApi?.session?.target_mode || 'fixed'
  const phase = (livePhase || '').toLowerCase()
  const afterTargetSampled =
    phase === 'pour_start' || phase === 'pour_end' || phase === 'episode_end'

  const targetPending = useMemo(() => {
    if (targetMode === 'fixed') {
      // Session fixed target is authoritative from the start.
      return false
    }
    // Non-fixed: suppress until SampleTargetWeight has run (pour_start+).
    if (!afterTargetSampled) {
      if (
        episode != null &&
        targetEpisodeRef.current != null &&
        targetEpisodeRef.current !== episode
      ) {
        return true
      }
      if (phase === 'scoop_start' || phase === 'scoop_end' || !phase) {
        return true
      }
    }
    return false
  }, [targetMode, afterTargetSampled, phase, episode])

  const runActive =
    enabled && topicActive && (runState === 'running' || runState === 'starting')

  const effectiveTarget =
    targetMode === 'fixed' && sessionApi?.session?.target_weight_g != null
      ? Number(sessionApi.session.target_weight_g)
      : targetWeightG

  return {
    topicActive,
    episode,
    episodesTotal,
    episodeEnd,
    targetWeightG: targetPending ? null : effectiveTarget,
    scoopedMassG,
    totalPouredG,
    pourOutcome,
    stopReason,
    runId,
    batchId: batchId || sessionApi?.session?.batch_id || null,
    powderId: powderId || sessionApi?.session?.powder_id || null,
    ingredientId,
    lotCode: lotCode || sessionApi?.session?.lot_code || null,
    operator: operator || sessionApi?.session?.operator || null,
    notes: notes || sessionApi?.session?.notes || null,
    sessionApi,
    targetPending,
    runActive,
  }
}
