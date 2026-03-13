// Main lights-out training dashboard
// Heavily commented to explain each concept and decision

import { useEffect, useMemo, useState } from 'react'
import { useNavigate } from 'react-router-dom'
import { useRos } from './ros/RosContext'
import { ROSLIB } from './ros/roslib'
import axios from 'axios'
import KpiCard from './components/KpiCard'
import GlassCard from './components/GlassCard'
import PhaseTimeline from './components/PhaseTimeline'
import Button from './components/ui/button'
import { useRuntimeConfig } from './config/RuntimeConfig'
// Removed Chart.js; using D3 for visualizations

// Type definitions for joint state rows
interface JointRow {
  name: string
  position?: number
  velocity?: number
  effort?: number
}

// Central configuration for services; replace hostnames/IPs as needed
const WEIGHT_TOPIC: string = (import.meta as any).env.VITE_WEIGHT_TOPIC || '/weight'
const PHASE_TOPIC: string = (import.meta as any).env.VITE_PHASE_TOPIC || '/lightsout_training/phase'
const EPISODE_TOPIC: string = (import.meta as any).env.VITE_EPISODE_TOPIC || '/lightsout_training/episode'
const EPISODES_TOTAL_TOPIC: string =
  (import.meta as any).env.VITE_EPISODES_TOTAL_TOPIC || '/lightsout_training/episodes_total'
const EPISODE_END_TOPIC: string =
  (import.meta as any).env.VITE_EPISODE_END_TOPIC || '/lightsout_training/episode_end'
const LIGHTSOUT_ACTIVE_TOPIC: string =
  (import.meta as any).env.VITE_LIGHTSOUT_ACTIVE_TOPIC || '/lightsout_training/active'
const POUR_STATUS_TOPIC: string = (import.meta as any).env.VITE_POUR_STATUS_TOPIC || '/pour_status'

function App() {
  const navigate = useNavigate()
  const ros = useRos()
  const { apiBase } = useRuntimeConfig()
  // ------------------------------ State --------------------------------------
  // Latest ROS-reported status line for the real-time panel
  const [realTimeStatus, setRealTimeStatus] = useState<string>('Waiting for robot status...')

  // Metrics from FastAPI (averages, success rate, alerts)
  const [metrics, setMetrics] = useState<any>({})

  // Raw historical rows for tables and charts
  const [historicalData, setHistoricalData] = useState<any[]>([])
  // Parsed joint states for display (latest message only)
  const [jointStates, setJointStates] = useState<JointRow[]>([])
  // Track last-seen timestamps to detect staleness
  const [lastBatchStatusTs, setLastBatchStatusTs] = useState<number | null>(null)
  const [lastJointStatesTs, setLastJointStatesTs] = useState<number | null>(null)
  const [nowMs, setNowMs] = useState<number>(Date.now())
  // Weighing scale live reading and sparkline history
  const [weight, setWeight] = useState<number | null>(null)
  const [weightHistory, setWeightHistory] = useState<{ x: string; y: number }[]>([])
  const [lastWeightTs, setLastWeightTs] = useState<number | null>(null)
  // Episode status (remaining episodes)
  const [currentEpisode, setCurrentEpisode] = useState<number | null>(null)
  const [totalEpisodes, setTotalEpisodes] = useState<number | null>(null)
  const [lastEpisodeTs, setLastEpisodeTs] = useState<number | null>(null)
  const [lastEpisodeEndTs, setLastEpisodeEndTs] = useState<number | null>(null)
  // Dynamic tolerance from PourToTarget feedback (grams)
  const [activeBandThresholdG, setActiveBandThresholdG] = useState<number | null>(null)
  const [lastPourFeedbackTs, setLastPourFeedbackTs] = useState<number | null>(null)
  const [activeTargetG, setActiveTargetG] = useState<number | null>(null)
  const [activePhase, setActivePhase] = useState<string | null>(null)
  // Joint position history for real-time plotting
  const [selectedJoint, setSelectedJoint] = useState<string | null>(null)
  const [jointHistory, setJointHistory] = useState<Record<string, { x: number; y: number }[]>>({})
  // Simple phase timeline (Scooping → Transporting → Pouring → Settling)
  const phases = useMemo(() => ['Scooping', 'Transporting', 'Pouring'], [])
  const [phaseIndex, setPhaseIndex] = useState<number>(-1) // -1 = Idle, 0..n-1 active/completed
  const startTsRef = useState<number>(Date.now())[0]
  const [elapsed, setElapsed] = useState<number>(0)
  useEffect(() => {
    const t = setInterval(() => setElapsed(((Date.now() - startTsRef) / 1000)), 500)
    return () => clearInterval(t)
  }, [startTsRef])
  // Last finished episode snapshot (for dashboard preview)
  type LastCycle = {
    batch_id?: string
    episode_index?: number
    start_time?: string
    end_time?: string
    target_weight_g?: number
    final_weight_g?: number
    net_weight_g?: number
    avg_flow_rate_g_s?: number
    total_episode_time_s?: number
    overshoot_g?: number
  }
  const [lastCycle, setLastCycle] = useState<LastCycle | null>(null)
  const [cycleStartMs, setCycleStartMs] = useState<number | null>(null)

  // Periodic ticker to re-evaluate staleness in the UI
  useEffect(() => {
    const id = setInterval(() => setNowMs(Date.now()), 1000)
    return () => clearInterval(id)
  }, [])

  // Staleness thresholds (tune as needed)
  const BATCH_STALE_MS = 10000 // 10s for low-rate status
  const JOINT_STALE_MS = 2000 // 2s for high-rate joint states
  const WEIGHT_STALE_MS = 1500 // 1.5s for scale readings
  const batchStale = !lastBatchStatusTs || nowMs - lastBatchStatusTs > BATCH_STALE_MS
  const jointStale = !lastJointStatesTs || nowMs - lastJointStatesTs > JOINT_STALE_MS
  const weightStale = !lastWeightTs || nowMs - lastWeightTs > WEIGHT_STALE_MS
  const queueStale = !lastEpisodeTs || nowMs - lastEpisodeTs > 5000
  const pourActive = !!lastPourFeedbackTs && (nowMs - lastPourFeedbackTs) < 2000

  // ------------------------------ ROS Bridge ---------------------------------
  useEffect(() => {
    const r = ros
    if (!r) return

    r.on('connection', () => {
      // Connected
    })
    r.on('error', () => {
      // Connection error
    })
    r.on('close', () => {
      // Disconnected
    })

    const phaseTopic = new ROSLIB.Topic({
      ros: r,
      name: PHASE_TOPIC,
      messageType: 'std_msgs/String',
    })
    phaseTopic.subscribe((msg: { data: string }) => {
      const phase = (msg.data || '').toLowerCase()
      setRealTimeStatus(phase || 'Waiting for robot status...')
      setLastBatchStatusTs(Date.now())
      if (phase.includes('scoop')) {
        setPhaseIndex(0)
        setCycleStartMs(Date.now())
      } else if (phase.includes('transport')) {
        setPhaseIndex(1)
      } else if (phase.includes('pour')) {
        setPhaseIndex(2)
      }
    })

    // Subscribe to /joint_states to visualize robot joints
    const jointStatesTopic = new ROSLIB.Topic({
      ros: r,
      name: '/joint_states',
      messageType: 'sensor_msgs/JointState',
    })
    jointStatesTopic.subscribe((msg: { name?: string[]; position?: number[]; velocity?: number[]; effort?: number[] }) => {
      const names = Array.isArray(msg.name) ? msg.name : []
      const positions = Array.isArray(msg.position) ? msg.position : []
      const velocities = Array.isArray(msg.velocity) ? msg.velocity : []
      const efforts = Array.isArray(msg.effort) ? msg.effort : []
      const rows: JointRow[] = names.map((n, i) => ({
        name: n,
        position: positions[i],
        velocity: velocities[i],
        effort: efforts[i],
      }))
      setJointStates(rows)
      setLastJointStatesTs(Date.now())

      // Update per-joint history (ring buffer)
      const nowSec = (Date.now() - startTsRef) / 1000
      setJointHistory((prev) => {
        const next = { ...prev }
        names.forEach((n, i) => {
          const p = positions[i]
          if (typeof p !== 'number' || !Number.isFinite(p)) return
          const arr = (next[n] ?? [])
          const updated = [...arr, { x: nowSec, y: p }]
          // keep last 300 samples
          next[n] = updated.slice(Math.max(0, updated.length - 300))
        })
        return next
      })

      // Default selection to the first joint seen
      if (!selectedJoint && names.length > 0) setSelectedJoint(names[0]!)
    })

    // Weighing scale subscriber
    const weightTopic = new ROSLIB.Topic({
      ros: r,
      name: WEIGHT_TOPIC,
      messageType: 'std_msgs/Float64',
    })
    weightTopic.subscribe((msg: { data: number }) => {
      const val = typeof msg.data === 'number' ? msg.data : NaN
      if (!Number.isNaN(val)) {
        setWeight(val)
        setLastWeightTs(Date.now())
        setWeightHistory((prev) => {
          const next = [...prev, { x: new Date().toISOString(), y: val }]
          // cap to last 120 points (~2 minutes at 1s rate)
          return next.slice(Math.max(0, next.length - 120))
        })
      }
    })

    const episodeTopic = new ROSLIB.Topic({
      ros: r,
      name: EPISODE_TOPIC,
      messageType: 'std_msgs/Int32',
    })
    episodeTopic.subscribe((msg: { data: number }) => {
      if (typeof msg.data === 'number') {
        setCurrentEpisode(msg.data)
        setLastEpisodeTs(Date.now())
      }
    })

    const episodesTotalTopic = new ROSLIB.Topic({
      ros: r,
      name: EPISODES_TOTAL_TOPIC,
      messageType: 'std_msgs/Int32',
    })
    episodesTotalTopic.subscribe((msg: { data: number }) => {
      if (typeof msg.data === 'number') {
        setTotalEpisodes(msg.data)
      }
    })

    const episodeEndTopic = new ROSLIB.Topic({
      ros: r,
      name: EPISODE_END_TOPIC,
      messageType: 'std_msgs/Int32',
    })
    episodeEndTopic.subscribe(() => {
      setPhaseIndex(phases.length)
      setLastEpisodeEndTs(Date.now())
    })

    const lightsoutActive = new ROSLIB.Topic({
      ros: r,
      name: LIGHTSOUT_ACTIVE_TOPIC,
      messageType: 'std_msgs/Bool',
    })
    lightsoutActive.subscribe((msg: { data: boolean }) => {
      if (!msg.data) {
        setPhaseIndex(-1)
        setCurrentEpisode(null)
        setTotalEpisodes(null)
      }
    })

    // Subscribe to PourStatus (UI-oriented topic)
    const pourStatus = new ROSLIB.Topic({
      ros: r,
      name: POUR_STATUS_TOPIC,
      messageType: 'robot_common_msgs/msg/PourStatus',
    })
    pourStatus.subscribe((msg: any) => {
      // Debug: confirm we receive PourStatus messages
      try { console.debug('pour_status:', msg) } catch {}
      if (msg && msg.active === true) {
        const band = Number(msg.band_threshold_g)
        const tgt = Number(msg.target_g)
        if (Number.isFinite(band)) setActiveBandThresholdG(band)
        if (Number.isFinite(tgt)) setActiveTargetG(tgt)
        if (typeof msg.phase === 'string') setActivePhase(msg.phase.toLowerCase())
        setLastPourFeedbackTs(Date.now())
      } else {
        setActiveBandThresholdG(null)
        setActiveTargetG(null)
        setActivePhase(null)
        setLastPourFeedbackTs(null)
      }
    })

    // Cleanup on unmount to avoid lingering subscriptions
    return () => {
      phaseTopic.unsubscribe()
      jointStatesTopic.unsubscribe()
      weightTopic.unsubscribe()
      episodeTopic.unsubscribe()
      episodesTotalTopic.unsubscribe()
      episodeEndTopic.unsubscribe()
      lightsoutActive.unsubscribe()
      pourStatus.unsubscribe()
    }
  }, [ros, phases.length])

  // ------------------------------ Data Fetch ---------------------------------
  useEffect(() => {
    let cancelled = false

    async function fetchData() {
      try {
        const [metricsRes, histRes] = await Promise.all([
          axios.get(`${apiBase}/metrics`),
          axios.get(`${apiBase}/lightsout_processed?limit=50`),
        ])
        if (!cancelled) {
          setMetrics(metricsRes.data.metrics || {})
          setHistoricalData(histRes.data.rows || [])
        }
      } catch {
        // Surface error in production
      }
    }

    // Initial fetch + polling every 30 seconds
    fetchData()
    const id = setInterval(fetchData, 30000)
    return () => {
      cancelled = true
      clearInterval(id)
    }
  }, [apiBase])

  const lastEpisodeRemaining =
    totalEpisodes != null && currentEpisode != null
      ? Math.max(totalEpisodes - currentEpisode, 0)
      : null

  const fetchLastCycle = async () => {
    try {
      const res = await axios.get(`${apiBase}/lightsout_processed?limit=1`)
      const row = res.data?.rows?.[0]
      if (row) {
        setLastCycle({
          batch_id: row.batch_id,
          episode_index: row.episode_index,
          start_time: row.start_time_ns ? new Date(row.start_time_ns / 1e6).toLocaleString() : undefined,
          end_time: row.end_time_ns ? new Date(row.end_time_ns / 1e6).toLocaleString() : undefined,
          target_weight_g: row.target_weight_g,
          final_weight_g: row.final_weight_g,
          net_weight_g: row.net_weight_g,
          avg_flow_rate_g_s: row.avg_flow_rate_g_s,
          total_episode_time_s: row.total_episode_time_s,
          overshoot_g: row.overshoot_g,
        })
      }
    } catch {
      // ignore fetch errors
    }
  }

  useEffect(() => {
    fetchLastCycle()
  }, [apiBase])

  useEffect(() => {
    if (!lastEpisodeEndTs) return
    const id = setTimeout(() => {
      fetchLastCycle()
    }, 1500)
    return () => clearTimeout(id)
  }, [lastEpisodeEndTs])

  // ------------------------------ UI -----------------------------------------
  return (
    <div className="px-6 py-6">
      {/* Header */}
      <div className="flex items-end justify-between gap-2 mb-4">
        <div className="flex flex-col gap-1">
          <h1 className="text-2xl font-bold tracking-tight" style={{ fontFamily: 'Space Grotesk' }}>Lights-Out Training Dashboard</h1>
          <p className="text-white/70">Monitor live episode progress, weighing, and recent training results.</p>
        </div>
        <div className="flex items-center gap-2">
          <Button onClick={() => navigate('/training')}>Configure Training Run</Button>
        </div>
      </div>

      {/* Alerts */}
      {metrics?.high_deviation_alert ? (
        <div className="mb-4 rounded-md border border-amber-400/30 bg-amber-400/10 px-4 py-2 text-amber-400">High deviation alert: investigate calibration or process drift</div>
      ) : null}

      <div className="grid grid-cols-12 gap-4">
        {/* KPI row */}
        <div className="col-span-12 md:col-span-3">
          <KpiCard label="Avg Episode Time" value={metrics?.avg_cycle_time != null ? `${(metrics.avg_cycle_time as number).toFixed(2)} s` : '—'} />
        </div>
        <div className="col-span-12 md:col-span-3">
          <KpiCard label="Success Rate" value={metrics?.success_rate != null ? `${(metrics.success_rate as number).toFixed(1)} %` : '—'} />
        </div>
        <div className="col-span-12 md:col-span-3">
          <KpiCard label="Avg Weight Deviation" value={metrics?.avg_weight_deviation != null ? `${(metrics.avg_weight_deviation as number).toFixed(3)} kg` : '—'} />
        </div>
        <div className="col-span-12 md:col-span-3">
          <KpiCard label="Training Status" value={batchStale ? 'Stale' : 'Live'} help={realTimeStatus} />
        </div>

        {/* Trends chart */}
        {/* <div className="col-span-12 md:col-span-8">
          <GlassCard>
            <div className="flex flex-col gap-2">
              <h3 className="text-lg font-semibold">Weight Deviation Trend</h3>
              <div className="border-t border-white/10" />
              <EChartsLine data={deviationSeries.map((d, i) => ({ x: i, y: d.y }))} height={260} />
            </div>
          </GlassCard>
        </div> */}
        {/* Comparison row: ECharts vs Plotly with same data */}
        {/* <div className="col-span-12 md:col-span-6">
          <GlassCard>
            <div className="flex flex-col gap-2">
              <h4 className="text-sm font-semibold">ECharts (area)</h4>
              <div className="border-t border-white/10" />
              <EChartsLine data={deviationSeries.map((d, i) => ({ x: i, y: d.y }))} height={220} />
            </div>
          </GlassCard>
        </div>
        <div className="col-span-12 md:col-span-6">
          <GlassCard>
            <div className="flex flex-col gap-2">
              <h4 className="text-sm font-semibold">Plotly (area)</h4>
              <div className="border-t border-white/10" />
              <PlotlyLine data={deviationSeries.map((d, i) => ({ x: i, y: d.y }))} height={220} />
            </div>
          </GlassCard>
        </div> */}

        {/* Overshoot scatter (last 100) */}
        {/* <div className="col-span-12">
          <GlassCard>
            <div className="flex flex-col gap-2">
              <h3 className="text-lg font-semibold">Overshoot (%) — last 100 batches</h3>
              <div className="border-t border-white/10" />
              <EChartsScatter data={overshoot} xLabel="Batch #" yLabel="Overshoot %" height={240} />
            </div>
          </GlassCard>
        </div> */}
        {/* Robot Status */}
        <div className="col-span-12">
          <GlassCard>
            <h3 className="text-lg font-semibold mb-2">Phase Timeline</h3>
            <PhaseTimeline phases={phases} index={phaseIndex} />
          </GlassCard>
        </div>
        {/* Queue Remaining Widget */}
        <div className="col-span-12 md:col-span-3">
          <GlassCard>
            <div className="flex items-center justify-between">
              <h3 className="text-lg font-semibold">Episodes Remaining</h3>
              <span className={`text-xs px-2 py-0.5 rounded-md ${queueStale ? 'bg-rose-400/15 text-rose-400' : 'bg-emerald-400/15 text-emerald-400'}`}>
                {queueStale ? 'Stale' : 'Live'}
              </span>
            </div>
            <div className="mt-2 text-4xl font-bold tracking-tight" style={{ fontFamily: 'Space Grotesk' }}>
              {lastEpisodeRemaining != null ? lastEpisodeRemaining : '—'}
            </div>
            <div className="text-xs text-white/60 mt-1">
              Episode {currentEpisode ?? '—'} of {totalEpisodes ?? '—'}
            </div>
          </GlassCard>
        </div>
        {/* Weighing scale widget (horizontal bar) */}
        <div className="col-span-12 md:col-span-4">
          <GlassCard>
            <div className={`flex flex-col gap-2 ${!pourActive ? 'opacity-50' : ''} transition-opacity`}>
              <div className="flex items-center justify-between">
                <h3 className="text-lg font-semibold">Weighing Scale</h3>
                <div className="flex items-center gap-2">
                  <span className={`text-xs px-2 py-0.5 rounded-md ${pourActive ? 'bg-emerald-400/15 text-emerald-400' : 'bg-white/10 text-white/60'}`}>
                    {pourActive ? 'Active' : 'Inactive'}
                  </span>
                  <span className={`text-xs px-2 py-0.5 rounded-md ${weightStale ? 'bg-white/10 text-white/60' : 'bg-sky-400/15 text-sky-300'}`}>
                    {weightStale ? 'Scale Disconnected' : 'Scale Connected'}
                  </span>
                </div>
              </div>
              <div className="flex items-center justify-between text-sm text-white/70">
                <span>Target</span>
                <span>
                  {(() => {
                    const target = activeTargetG ?? ((historicalData?.[0]?.target_weight_g as number) || 5000)
                    return `${Math.round(target)} g`
                  })()}
                </span>
              </div>
              <div className="w-full h-4 bg-white/10 rounded">
                {(() => {
                  const target = activeTargetG ?? ((historicalData?.[0]?.target_weight_g as number) || 5000)
                  // Always-visible base band ±10 g (yellow)
                  const baseTolG = 10
                  // Final tolerance band (green) shown only during TRICKLE
                  const tolG = (pourActive && activePhase === 'trickle' && activeBandThresholdG != null) ? activeBandThresholdG : 0
                  const maxRange = Math.max(target * 2, Math.max(2 * tolG, 200))
                  const val = typeof weight === 'number' ? Math.max(0, Math.min(maxRange, weight)) : 0
                  const pct = (val / maxRange) * 100
                  const baseLowBand = Math.max(0, (target - baseTolG) / maxRange * 100)
                  const baseHighBand = Math.min(100, (target + baseTolG) / maxRange * 100)
                  const lowBand = tolG > 0 ? Math.max(0, (target - tolG) / maxRange * 100) : 0
                  const highBand = tolG > 0 ? Math.min(100, (target + tolG) / maxRange * 100) : 100
                  const targetPct = Math.max(0, Math.min(100, (target / maxRange) * 100))
                  // Color fill from red -> yellow -> green as we approach target
                  const err = Math.abs(target - val)
                  const closeness = Math.max(0, Math.min(1, 1 - err / Math.max(target, 1)))
                  const hue = Math.round(120 * closeness) // 0=red, 120=green
                  const fillColor = `hsla(${hue}, 80%, 45%, 0.6)`
                  return (
                    <div className="relative w-full h-full">
                      {/* always-visible base tolerance band (±10 g) */}
                      <div className="absolute top-0 bottom-0" style={{ left: `${baseLowBand}%`, right: `${100 - baseHighBand}%`, borderRadius: 4, background: 'rgba(234,179,8,0.25)', zIndex: 2 }} />
                      {/* base band edge markers */}
                      <div className="absolute top-0 bottom-0" style={{ left: `${baseLowBand}%`, width: '2px', transform: 'translateX(-1px)', background: 'rgba(234,179,8,0.9)', zIndex: 3, borderRadius: 1 }} />
                      <div className="absolute top-0 bottom-0" style={{ left: `${baseHighBand}%`, width: '2px', transform: 'translateX(-1px)', background: 'rgba(234,179,8,0.9)', zIndex: 3, borderRadius: 1 }} />
                      {/* final tolerance band (TRICKLE only) */}
                      {tolG > 0 && (
                        <>
                          <div className="absolute top-0 bottom-0 bg-emerald-400/25" style={{ left: `${lowBand}%`, right: `${100 - highBand}%`, borderRadius: 4, zIndex: 2 }} />
                          {/* trickle band edge markers */}
                          <div className="absolute top-0 bottom-0" style={{ left: `${lowBand}%`, width: '2px', transform: 'translateX(-1px)', background: 'rgba(16,185,129,0.95)', zIndex: 3, borderRadius: 1 }} />
                          <div className="absolute top-0 bottom-0" style={{ left: `${highBand}%`, width: '2px', transform: 'translateX(-1px)', background: 'rgba(16,185,129,0.95)', zIndex: 3, borderRadius: 1 }} />
                        </>
                      )}
                      {/* fill */}
                      <div className="absolute top-0 bottom-0" style={{ width: `${pct}%`, borderRadius: 4, background: fillColor, transition: 'width 0.2s linear, background-color 0.2s linear', zIndex: 1 }} />
                      {/* center target line */}
                      <div className="absolute top-0 bottom-0" style={{ left: `${targetPct}%`, width: '2px', transform: 'translateX(-1px)', background: '#10b981', zIndex: 2, borderRadius: 1 }} />
                    </div>
                  )
                })()}
              </div>
              <div className="flex items-center justify-between text-xs text-white/60">
                <span>
                  {typeof weight === 'number' ? `${Math.round(weight)} g` : '—'} (±10 g{pourActive && activePhase === 'trickle' && activeBandThresholdG != null ? `, final ±${Math.round(activeBandThresholdG)} g` : ''})
                </span>
                <span className={`${weightStale ? 'text-rose-400' : 'text-emerald-400'}`}>{weightStale ? 'Stale' : WEIGHT_TOPIC}</span>
              </div>
            </div>
          </GlassCard>
        </div>

        {/* Last finished episode snapshot */}
        <div className="col-span-12 md:col-span-8">
          <GlassCard>
            <div className="flex flex-col gap-2">
              <h3 className="text-lg font-semibold">Last Finished Episode</h3>
              <div className="border-t border-white/10" />
              {lastCycle ? (
                <div className="grid grid-cols-2 md:grid-cols-3 gap-y-2 text-sm">
                  <div><span className="text-white/60">Batch</span><div>{lastCycle.batch_id}</div></div>
                  <div><span className="text-white/60">Episode</span><div>{lastCycle.episode_index ?? '—'}</div></div>
                  <div><span className="text-white/60">Start</span><div>{lastCycle.start_time ?? '—'}</div></div>
                  <div><span className="text-white/60">End</span><div>{lastCycle.end_time ?? '—'}</div></div>
                  <div className="text-right"><span className="text-white/60">Target</span><div>{lastCycle.target_weight_g != null ? lastCycle.target_weight_g.toFixed(2) : '—'} g</div></div>
                  <div className="text-right"><span className="text-white/60">Final</span><div>{lastCycle.final_weight_g != null ? lastCycle.final_weight_g.toFixed(2) : '—'} g</div></div>
                  <div className="text-right"><span className="text-white/60">Net</span><div>{lastCycle.net_weight_g != null ? lastCycle.net_weight_g.toFixed(2) : '—'} g</div></div>
                  <div className="text-right"><span className="text-white/60">Avg Flow</span><div>{lastCycle.avg_flow_rate_g_s != null ? lastCycle.avg_flow_rate_g_s.toFixed(2) : '—'} g/s</div></div>
                  <div className="text-right"><span className="text-white/60">Duration</span><div>{lastCycle.total_episode_time_s != null ? lastCycle.total_episode_time_s.toFixed(2) : '—'} s</div></div>
                  <div className="text-right"><span className="text-white/60">Overshoot</span><div>{lastCycle.overshoot_g != null ? lastCycle.overshoot_g.toFixed(2) : '—'} g</div></div>
                </div>
              ) : (
                <span className="text-sm text-white/70">No finished episode yet</span>
              )}
            </div>
          </GlassCard>
        </div>



        {/* Real-time Joint Position plot */}
        {/* <div className="col-span-12 md:col-span-12">
          <GlassCard>
            <div className="flex flex-col gap-2">
              <div className="flex items-center justify-between gap-2">
                <h3 className="text-lg font-semibold">Joint Position (real-time)</h3>
                <select
                  className="bg-[#13203499] border border-white/20 rounded-md px-2 py-1 text-sm"
                  value={selectedJoint ?? ''}
                  onChange={(e) => setSelectedJoint(e.target.value)}
                >
                  {(Object.keys(jointHistory) as string[]).map((name) => (
                    <option key={name} value={name}>{name}</option>
                  ))}
                </select>
              </div>
              <div className="border-t border-white/10" />
              {selectedJoint && jointHistory[selectedJoint] && jointHistory[selectedJoint].length > 0 ? (
                <div className="grid grid-cols-1 md:grid-cols-2 gap-4 w-full">
                  <div>
                    <div className="text-sm font-semibold mb-2">ECharts</div>
                    <EChartsLine data={jointHistory[selectedJoint]!} height={220} xMax={elapsed} />
                  </div>
                  <div>
                    <div className="text-sm font-semibold mb-2">Plotly</div>
                    <PlotlyLine data={jointHistory[selectedJoint]!} height={220} xMax={elapsed} />
                  </div>
                </div>
              ) : (
                <span className="text-sm text-white/70">Waiting for joint states...</span>
              )}
            </div>
          </GlassCard>
        </div> */}

        {/* Joint states table */}
        <div className="col-span-12">
          <GlassCard>
            <div className="flex flex-col gap-2">
              <h3 className="text-lg font-semibold">Joint States (latest)</h3>
              <span className={`text-xs ${jointStale ? 'text-rose-400' : 'text-emerald-400'}`}>
                {jointStale ? 'Stale: no updates in >2s' : 'Healthy: receiving updates'}
              </span>
              <div className="border-t border-white/10" />
              <table className="w-full text-sm">
                <thead className="text-left text-white/70">
                  <tr>
                    <th>Joint</th>
                    <th className="text-right">Position (rad)</th>
                    <th className="text-right">Velocity (rad/s)</th>
                    <th className="text-right">Effort</th>
                  </tr>
                </thead>
                <tbody>
                  {jointStates.map((row) => (
                    <tr key={row.name} className="border-t border-white/10">
                      <td>{row.name}</td>
                      <td className="text-right">{typeof row.position === 'number' ? row.position.toFixed(4) : '—'}</td>
                      <td className="text-right">{typeof row.velocity === 'number' ? row.velocity.toFixed(4) : '—'}</td>
                      <td className="text-right">{typeof row.effort === 'number' ? row.effort.toFixed(3) : '—'}</td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          </GlassCard>
        </div>
      </div>
    </div>
  )
}

export default App


