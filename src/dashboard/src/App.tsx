// Professional ROS-enabled dashboard for batch processing
// Heavily commented to explain each concept and decision

import { useEffect, useMemo, useState } from 'react'
import { useRos } from './ros/RosContext'
import { ROSLIB } from './ros/roslib'
import axios from 'axios'
import KpiCard from './components/KpiCard'
import EChartsLine from './components/EChartsLine'
import PlotlyLine from './components/PlotlyLine'
import GlassCard from './components/GlassCard'
import PhaseTimeline from './components/PhaseTimeline'
import EChartsScatter from './components/EChartsScatter'
import Button from './components/ui/button'
import Select from './components/ui/select'
// Removed Chart.js; using D3 for visualizations

// Type definitions for joint state rows
interface JointRow {
  name: string
  position?: number
  velocity?: number
  effort?: number
}

// Central configuration for services; replace hostnames/IPs as needed
const ROSBRIDGE_URL: string = (import.meta as any).env.VITE_ROSBRIDGE_URL || 'ws://localhost:9090'
const API_BASE: string = (import.meta as any).env.VITE_API_BASE || 'http://localhost:8000'
const WEIGHT_TOPIC: string = (import.meta as any).env.VITE_WEIGHT_TOPIC || '/weight'
const BT_QUEUE_TOPIC: string = (import.meta as any).env.VITE_BT_QUEUE_TOPIC || '/bt_queue_remaining'
const POUR_STATUS_TOPIC: string = (import.meta as any).env.VITE_POUR_STATUS_TOPIC || '/pour_status'

function App() {
  const ros = useRos()
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
  // Queue status (remaining ingredients)
  const [queueRemaining, setQueueRemaining] = useState<number | null>(null)
  const [lastQueueTs, setLastQueueTs] = useState<number | null>(null)
  // Dynamic tolerance from PourToTarget feedback (grams)
  const [activeBandThresholdG, setActiveBandThresholdG] = useState<number | null>(null)
  const [lastPourFeedbackTs, setLastPourFeedbackTs] = useState<number | null>(null)
  const [activeTargetG, setActiveTargetG] = useState<number | null>(null)
  const [activePhase, setActivePhase] = useState<string | null>(null)
  // Joint position history for real-time plotting
  const [selectedJoint, setSelectedJoint] = useState<string | null>(null)
  const [jointHistory, setJointHistory] = useState<Record<string, { x: number; y: number }[]>>({})
  // Simple phase timeline (Scooping → Transporting → Pouring → Settling)
  const phases = useMemo(() => ['Scooping', 'Transporting', 'Pouring', 'Settling'], [])
  const [phaseIndex, setPhaseIndex] = useState<number>(-1) // -1 = Idle, 0..n-1 active/completed
  // Demo mode: simulate ROS topics locally
  const [demoMode, setDemoMode] = useState<boolean>(false)
  const startTsRef = useState<number>(Date.now())[0]
  const [elapsed, setElapsed] = useState<number>(0)
  useEffect(() => {
    const t = setInterval(() => setElapsed(((Date.now() - startTsRef) / 1000)), 500)
    return () => clearInterval(t)
  }, [startTsRef])
  // Demo scatter data for overshoot percentage
  const [overshoot, setOvershoot] = useState<{ x: number; y: number }[]>([])
  // Last finished cycle snapshot (for dashboard preview)
  type LastCycle = {
    batch_id?: string
    start_time?: string
    end_time?: string
    target_weight?: number
    actual_weight?: number
    success?: boolean
    cycle_time?: number
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
  const queueStale = !lastQueueTs || nowMs - lastQueueTs > 5000
  const pourActive = !!lastPourFeedbackTs && (nowMs - lastPourFeedbackTs) < 2000

  // ------------------------------ ROS Bridge ---------------------------------
  useEffect(() => {
    if (demoMode) return
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

    // Subscribe to the String topic that the ROS2 node publishes
    const statusTopic = new ROSLIB.Topic({
      ros: r,
      name: '/batch_status',
      messageType: 'std_msgs/String',
    })
    statusTopic.subscribe((msg: { data: string }) => {
      setRealTimeStatus(msg.data)
      setLastBatchStatusTs(Date.now())
      const t = msg.data.toLowerCase()
      if (t.includes('scoop')) { setPhaseIndex(0); setCycleStartMs(Date.now()) }
      else if (t.includes('transport')) setPhaseIndex(1)
      else if (t.includes('pour')) setPhaseIndex(2)
      else if (t.includes('settle')) setPhaseIndex(3)
      else if (t.includes('success') || t.includes('complete')) {
        setPhaseIndex(phases.length) // all done
        // fetch latest completed row from backend
        fetch(`${API_BASE}/historical_data?days=1`).then(r => r.json()).then(json => {
          const row = (json.raw_data && json.raw_data[0]) || null
          if (row) {
            setLastCycle({
              batch_id: row.batch_id,
              start_time: row.start_time,
              end_time: row.end_time,
              target_weight: row.target_weight,
              actual_weight: row.actual_weight,
              success: row.success === 1,
              cycle_time: row.cycle_time,
            })
          }
        }).catch(() => {})
      }
      else if (t.includes('fail') || t.includes('error')) setPhaseIndex(phases.length) // terminal
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

    // Queue remaining subscriber
    const queueTopic = new ROSLIB.Topic({
      ros: r,
      name: BT_QUEUE_TOPIC,
      messageType: 'std_msgs/Int32',
    })
    queueTopic.subscribe((msg: { data: number }) => {
      if (typeof msg.data === 'number') {
        setQueueRemaining(msg.data)
        setLastQueueTs(Date.now())
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
      statusTopic.unsubscribe()
      jointStatesTopic.unsubscribe()
      weightTopic.unsubscribe()
      queueTopic.unsubscribe()
      pourStatus.unsubscribe()
    }
  }, [demoMode, ros])

  // Demo mode simulation
  useEffect(() => {
    if (!demoMode) return
    let raf: any
    let phaseTimer: any
    const t0 = Date.now()
    // ensure a joint is selected
    if (!selectedJoint) setSelectedJoint('joint1')
    // phases loop
    const seq = [...phases, 'Success']
    let pi = -1
    const advance = () => {
      pi = (pi + 1) % seq.length
      if (seq[pi] === 'Success') {
        setPhaseIndex(phases.length)
        // synthesize last cycle snapshot from current simulated state
        const end = new Date()
        const start = cycleStartMs ? new Date(cycleStartMs) : new Date(end.getTime() - 6000)
        const target = 1.0 + Math.random() * 0.2
        const actual = (weight ?? target) as number
        setLastCycle({
          batch_id: `demo-${Math.floor(Math.random()*1000)}`,
          start_time: start.toISOString(),
          end_time: end.toISOString(),
          target_weight: +target.toFixed(3),
          actual_weight: +actual.toFixed(3),
          success: Math.abs(actual - target) / target < 0.05,
          cycle_time: ((end.getTime() - start.getTime())/1000)|0,
        })
      } else {
        setPhaseIndex(phases.indexOf(seq[pi]))
        if (seq[pi] === 'Scooping') setCycleStartMs(Date.now())
      }
      setRealTimeStatus(seq[pi] === 'Success' ? 'Batch Success' : `${seq[pi]}...`)
      phaseTimer = setTimeout(advance, 2000)
    }
    advance()

    const tick = () => {
      const tsec = (Date.now() - t0) / 1000
      // joint sine wave
      const name = selectedJoint || 'joint1'
      const pos = Math.sin(tsec) * 0.5
      setJointHistory((prev) => {
        const next = { ...prev }
        const arr = next[name] ?? []
        const updated = [...arr, { x: tsec, y: pos }]
        next[name] = updated.slice(Math.max(0, updated.length - 300))
        return next
      })
      if (!selectedJoint) setSelectedJoint(name)
      // weight around target 1.0kg
      const val = 1.0 + 0.05 * Math.sin(tsec * 0.7) + (Math.random() - 0.5) * 0.01
      setWeight(val)
      setLastWeightTs(Date.now())
      // overshoot: add random point every ~0.5s up to 100
      setOvershoot((prev) => {
        const next = [...prev, { x: prev.length + 1, y: Math.max(-5, Math.min(15, (Math.random() - 0.5) * 12)) }]
        return next.slice(Math.max(0, next.length - 100))
      })
      setWeightHistory((prev) => {
        const iso = new Date().toISOString()
        const next = [...prev, { x: iso, y: val }]
        return next.slice(Math.max(0, next.length - 120))
      })
      raf = requestAnimationFrame(tick)
    }
    raf = requestAnimationFrame(tick)
    return () => {
      cancelAnimationFrame(raf)
      clearTimeout(phaseTimer)
    }
  }, [demoMode])

  // ------------------------------ Data Fetch ---------------------------------
  useEffect(() => {
    let cancelled = false

    async function fetchData() {
      try {
        const [metricsRes, histRes] = await Promise.all([
          axios.get(`${API_BASE}/metrics`),
          axios.get(`${API_BASE}/historical_data`),
        ])
        if (!cancelled) {
          setMetrics(metricsRes.data.metrics || {})
          setHistoricalData(histRes.data.raw_data || [])
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
  }, [])

  // ------------------------------ Charts (D3) ---------------------------------
  const deviationSeries = useMemo(() => {
    // Build series for D3 from historical data; if empty, provide mock data so UI is demonstrable
    if (historicalData.length === 0) {
      const now = new Date()
      const mock: { x: string; y: number }[] = []
      for (let i = 29; i >= 0; i -= 1) {
        const d = new Date(now)
        d.setDate(now.getDate() - i)
        // Smooth wave + small noise to resemble deviation (kg)
        const wave = 0.15 + 0.35 * Math.abs(Math.sin((i / 10) * Math.PI))
        const noise = (Math.random() - 0.5) * 0.05
        const val = Math.max(0, +(wave + noise).toFixed(3))
        mock.push({ x: d.toISOString().slice(0, 10), y: val })
      }
      return mock
    }
    return historicalData.map((row: any) => ({
      x: (row.start_time as string) ?? '',
      y: Math.abs(((row.actual_weight ?? 0) as number) - ((row.target_weight ?? 0) as number)),
    })) as { x: string; y: number }[]
  }, [historicalData])

  // ------------------------------ UI -----------------------------------------
  return (
    <div className="px-6 py-6">
      {/* Header */}
      <div className="flex items-end justify-between gap-2 mb-4">
        <div className="flex flex-col gap-1">
          <h1 className="text-2xl font-bold tracking-tight" style={{ fontFamily: 'Space Grotesk' }}>Robot Batch Processing Dashboard</h1>
          <p className="text-white/70">Real-time operations and historical analytics</p>
        </div>
        <div className="flex items-center gap-2">
          <span className={`text-xs ${demoMode ? 'text-emerald-400' : 'text-white/60'}`}>{demoMode ? 'Demo Mode' : 'Live Mode'}</span>
          <Button onClick={() => setDemoMode((v) => !v)}>{demoMode ? 'Disable Demo' : 'Enable Demo'}</Button>
        </div>
      </div>

      {/* Alerts */}
      {metrics?.high_deviation_alert ? (
        <div className="mb-4 rounded-md border border-amber-400/30 bg-amber-400/10 px-4 py-2 text-amber-400">High deviation alert: investigate calibration or process drift</div>
      ) : null}

      <div className="grid grid-cols-12 gap-4">
        {/* KPI row */}
        <div className="col-span-12 md:col-span-3">
          <KpiCard label="Avg Cycle Time" value={metrics?.avg_cycle_time != null ? `${(metrics.avg_cycle_time as number).toFixed(2)} s` : '—'} />
        </div>
        <div className="col-span-12 md:col-span-3">
          <KpiCard label="Success Rate" value={metrics?.success_rate != null ? `${(metrics.success_rate as number).toFixed(1)} %` : '—'} />
        </div>
        <div className="col-span-12 md:col-span-3">
          <KpiCard label="Avg Weight Deviation" value={metrics?.avg_weight_deviation != null ? `${(metrics.avg_weight_deviation as number).toFixed(3)} kg` : '—'} />
        </div>
        <div className="col-span-12 md:col-span-3">
          <KpiCard label="Status" value={batchStale ? 'Stale' : 'Live'} help={realTimeStatus} />
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
              <h3 className="text-lg font-semibold">Ingredients Remaining</h3>
              <span className={`text-xs px-2 py-0.5 rounded-md ${queueStale ? 'bg-rose-400/15 text-rose-400' : 'bg-emerald-400/15 text-emerald-400'}`}>
                {queueStale ? 'Stale' : 'Live'}
              </span>
            </div>
            <div className="mt-2 text-4xl font-bold tracking-tight" style={{ fontFamily: 'Space Grotesk' }}>
              {queueRemaining != null ? queueRemaining : '—'}
            </div>
            <div className="text-xs text-white/60 mt-1">Topic: {BT_QUEUE_TOPIC}</div>
          </GlassCard>
        </div>
        {/* Weighing scale widget (horizontal bar) */}
        <div className="col-span-12 md:col-span-4">
          <GlassCard>
            <div className={`flex flex-col gap-2 ${!pourActive ? 'opacity-50' : ''} transition-opacity`}>
              <div className="flex items-center justify-between">
                <h3 className="text-lg font-semibold">Weighing Scale</h3>
                <span className={`text-xs px-2 py-0.5 rounded-md ${pourActive ? 'bg-emerald-400/15 text-emerald-400' : 'bg-white/10 text-white/60'}`}>
                  {pourActive ? 'Active' : 'Inactive'}
                </span>
              </div>
              <div className="flex items-center justify-between text-sm text-white/70">
                <span>Target</span>
                <span>
                  {(() => {
                    const target = activeTargetG ?? ((historicalData?.[0]?.target_weight as number) || 5000)
                    return `${Math.round(target)} g`
                  })()}
                </span>
              </div>
              <div className="w-full h-4 bg-white/10 rounded">
                {(() => {
                  const target = activeTargetG ?? ((historicalData?.[0]?.target_weight as number) || 5000)
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

        {/* Last finished cycle snapshot */}
        <div className="col-span-12 md:col-span-8">
          <GlassCard>
            <div className="flex flex-col gap-2">
              <h3 className="text-lg font-semibold">Last Finished Cycle</h3>
              <div className="border-t border-white/10" />
              {lastCycle ? (
                <div className="grid grid-cols-2 md:grid-cols-3 gap-y-2 text-sm">
                  <div><span className="text-white/60">Batch</span><div>{lastCycle.batch_id}</div></div>
                  <div><span className="text-white/60">Start</span><div>{lastCycle.start_time}</div></div>
                  <div><span className="text-white/60">End</span><div>{lastCycle.end_time}</div></div>
                  <div className="text-right"><span className="text-white/60">Target</span><div>{lastCycle.target_weight?.toFixed(3)} kg</div></div>
                  <div className="text-right"><span className="text-white/60">Actual</span><div>{lastCycle.actual_weight?.toFixed(3)} kg</div></div>
                  <div className="text-right"><span className="text-white/60">Cycle</span><div>{lastCycle.cycle_time?.toFixed(2)} s</div></div>
                  <div className="col-span-2 md:col-span-3">
                    <span className={`text-xs px-2 py-0.5 rounded-md ${lastCycle.success ? 'bg-emerald-400/15 text-emerald-400' : 'bg-rose-400/15 text-rose-400'}`}>
                      {lastCycle.success ? 'Success' : 'Failed'}
                    </span>
                  </div>
                </div>
              ) : (
                <span className="text-sm text-white/70">No finished cycle yet</span>
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


