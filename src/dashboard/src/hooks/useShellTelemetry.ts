import { useEffect, useState } from 'react'
import { useRos } from '../ros/RosContext'
import { ROSLIB } from '../ros/roslib'
import { useRuntimeConfig } from '../config/RuntimeConfig'

const RUN_STATE_TOPIC =
  (import.meta as any).env.VITE_RUN_STATE_TOPIC || '/orchestrator/run_state'
const WEIGHT_TOPIC = (import.meta as any).env.VITE_WEIGHT_TOPIC || '/weight'
const JOINT_STATES_TOPIC =
  (import.meta as any).env.VITE_JOINT_STATES_TOPIC || '/joint_states'

/** MoveIt-style freshness for arm bridge (/joint_states). */
const ARM_STALE_MS = 1500
const WEIGHT_STALE_MS = 1500
/** Warn when clocks disagree by more than this (seconds). */
export const CLOCK_SKEW_WARN_S = 0.5

type RosTime = { sec?: number; nanosec?: number; secs?: number; nsecs?: number }

function rosStampToUnixSeconds(stamp: RosTime | undefined): number | null {
  if (!stamp) return null
  const sec = stamp.sec ?? stamp.secs
  const nsec = stamp.nanosec ?? stamp.nsecs ?? 0
  if (typeof sec !== 'number' || !Number.isFinite(sec)) return null
  return sec + (typeof nsec === 'number' ? nsec / 1e9 : 0)
}

function formatClock(unixSec: number | null): string {
  if (unixSec == null || !Number.isFinite(unixSec)) return '—'
  try {
    return new Date(unixSec * 1000).toISOString().replace('T', ' ').replace('Z', 'Z')
  } catch {
    return '—'
  }
}

function skewLabel(deltaSec: number | null): string {
  if (deltaSec == null || !Number.isFinite(deltaSec)) return '—'
  const sign = deltaSec >= 0 ? '+' : ''
  return `${sign}${deltaSec.toFixed(3)}s`
}

export function useShellTelemetry() {
  const ros = useRos()
  const { apiBase } = useRuntimeConfig()
  const [runState, setRunState] = useState('idle')
  const [weightStale, setWeightStale] = useState(true)
  const [armStale, setArmStale] = useState(true)
  const [lastWeightTs, setLastWeightTs] = useState<number | null>(null)
  const [lastArmTs, setLastArmTs] = useState<number | null>(null)
  const [browserUnix, setBrowserUnix] = useState(() => Date.now() / 1000)
  const [piUnix, setPiUnix] = useState<number | null>(null)
  const [niryoUnix, setNiryoUnix] = useState<number | null>(null)

  useEffect(() => {
    if (!ros) return
    const runStateTopic = new ROSLIB.Topic({
      ros,
      name: RUN_STATE_TOPIC,
      messageType: 'std_msgs/String',
    })
    runStateTopic.subscribe((msg: { data: string }) => {
      setRunState((msg.data || 'idle').toLowerCase())
    })
    const weightTopic = new ROSLIB.Topic({
      ros,
      name: WEIGHT_TOPIC,
      messageType: 'std_msgs/Float64',
    })
    weightTopic.subscribe((msg: { data: number }) => {
      if (typeof msg.data === 'number' && Number.isFinite(msg.data)) {
        setLastWeightTs(Date.now())
        setWeightStale(false)
      }
    })
    const jointTopic = new ROSLIB.Topic({
      ros,
      name: JOINT_STATES_TOPIC,
      messageType: 'sensor_msgs/JointState',
    })
    jointTopic.subscribe((msg: { header?: { stamp?: RosTime } }) => {
      setLastArmTs(Date.now())
      setArmStale(false)
      const stampUnix = rosStampToUnixSeconds(msg?.header?.stamp)
      if (stampUnix != null) setNiryoUnix(stampUnix)
    })
    return () => {
      runStateTopic.unsubscribe()
      weightTopic.unsubscribe()
      jointTopic.unsubscribe()
    }
  }, [ros])

  useEffect(() => {
    const id = window.setInterval(() => {
      const now = Date.now()
      setBrowserUnix(now / 1000)
      setWeightStale(!lastWeightTs || now - lastWeightTs > WEIGHT_STALE_MS)
      setArmStale(!lastArmTs || now - lastArmTs > ARM_STALE_MS)
    }, 500)
    return () => window.clearInterval(id)
  }, [lastWeightTs, lastArmTs])

  useEffect(() => {
    let cancelled = false
    const pingPiClock = async () => {
      try {
        const res = await fetch(`${apiBase}/host_info`, { cache: 'no-store' })
        if (!res.ok) throw new Error('host_info failed')
        const data = await res.json()
        if (cancelled) return
        const unix =
          typeof data?.utc_unix === 'number'
            ? data.utc_unix
            : data?.utc_iso
              ? Date.parse(data.utc_iso) / 1000
              : null
        if (unix != null && Number.isFinite(unix)) setPiUnix(unix)
      } catch {
        if (!cancelled) setPiUnix(null)
      }
    }
    void pingPiClock()
    const id = window.setInterval(pingPiClock, 2000)
    return () => {
      cancelled = true
      window.clearInterval(id)
    }
  }, [apiBase])

  const piSkew = piUnix != null ? piUnix - browserUnix : null
  const niryoSkew = niryoUnix != null ? niryoUnix - browserUnix : null
  const niryoVsPiSkew =
    niryoUnix != null && piUnix != null ? niryoUnix - piUnix : null

  const clockSkewWarn =
    (piSkew != null && Math.abs(piSkew) > CLOCK_SKEW_WARN_S) ||
    (niryoSkew != null && Math.abs(niryoSkew) > CLOCK_SKEW_WARN_S) ||
    (niryoVsPiSkew != null && Math.abs(niryoVsPiSkew) > CLOCK_SKEW_WARN_S)

  return {
    runState,
    weightStale,
    armStale,
    browserUnix,
    piUnix,
    niryoUnix,
    piSkew,
    niryoSkew,
    niryoVsPiSkew,
    clockSkewWarn,
    formatClock,
    skewLabel,
  }
}
