import { useEffect, useMemo, useState } from 'react'
import { useNavigate } from 'react-router-dom'
import DateTimeText from './components/DateTimeText'
import { useRos } from './ros/RosContext'
import { ROSLIB } from './ros/roslib'
import KpiCard from './components/KpiCard'
import GlassCard from './components/GlassCard'
import PhaseTimeline from './components/PhaseTimeline'
import Button from './components/ui/button'
import { useRuntimeConfig } from './config/RuntimeConfig'
type Metadata = {
  run_id?: string
  weightment_id?: string
  batch_id?: string
  ingredient_id?: string
  location_id?: string
  location_code?: string
  target_weight_g?: number | string
}

type RobotRunRow = {
  id: number
  weightment_id: number
  event_id: string | null
  batch_id: string | null
  ingredient_id: string | null
  stock_location_id: number | null
  stock_location_code: string | null
  ingredient_name: string | null
  target_weight_g: number | null
  trace_run_id: string | null
  status: string | null
  error_message: string | null
  requested_at: string | null
  started_at: string | null
  finished_at: string | null
  start_utc: string | null
  end_utc: string | null
  actual_weight_kg: number | null
  processed_id: number | null
  mcap_path: string | null
  parquet_path: string | null
  mes_weighment_sent: boolean
  mes_batch_end_sent: boolean
}

type DetailRow = {
  weightment_id: number
  target_weight_kg: number | null
  actual_weight_kg: number | null
  completed: boolean
  stock_item_id: string | null
  ingredient_name: string | null
  location_id: number | null
  location_code: string | null
  start_time: string | null
  end_time: string | null
  energy_kwh: number | null
  robot_status: string | null
  robot_trace_run_id: string | null
  robot_processed_id: number | null
  robot_live_completion_weight_kg: number | null
  robot_processed_final_weight_kg: number | null
  robot_processed_start_time: string | null
  robot_processed_end_time: string | null
  robot_processed_overshoot_g: number | null
  robot_processed_scoop_duration_s: number | null
  robot_processed_pour_duration_s: number | null
  robot_processed_settle_time_s: number | null
  robot_phase_events: Array<{ t_ns: number; phase: string }> | null
  robot_mcap_path: string | null
  robot_parquet_path: string | null
  robot_mes_weighment_sent: boolean
  robot_mes_batch_end_sent: boolean
}

type PourStatusMsg = {
  target_g?: number
  band_threshold_g?: number
  remaining_to_band_g?: number
  active?: boolean
  phase?: string
}

const WEIGHT_TOPIC: string =
  (import.meta as any).env.VITE_WEIGHT_TOPIC || '/weight'
const WEBHOOK_PHASE_TOPIC: string =
  (import.meta as any).env.VITE_WEBHOOK_PHASE_TOPIC || '/webhook_run/phase'
const WEBHOOK_ACTIVE_TOPIC: string =
  (import.meta as any).env.VITE_WEBHOOK_ACTIVE_TOPIC || '/webhook_run/active'
const WEBHOOK_METADATA_TOPIC: string =
  (import.meta as any).env.VITE_WEBHOOK_METADATA_TOPIC || '/webhook_run/metadata'
const RUN_STATE_TOPIC: string =
  (import.meta as any).env.VITE_RUN_STATE_TOPIC || '/orchestrator/run_state'
const POUR_STATUS_TOPIC: string =
  (import.meta as any).env.VITE_POUR_STATUS_TOPIC || '/pour_status'
const VIBRATION_TOPIC: string =
  (import.meta as any).env.VITE_VIBRATION_TOPIC || '/vibration/intensity'
const INCLINE_TOPIC: string =
  (import.meta as any).env.VITE_INCLINE_TOPIC || '/incline_control'

const TIMELINE_PHASES = [
  'Move To Scoop',
  'Scooping',
  'Move To Weigh',
  'Pouring',
  'Return To Scoop',
]

function formatNumber(value: number | null | undefined, digits = 2): string {
  return typeof value === 'number' ? value.toFixed(digits) : '—'
}

function parseTargetWeight(metadata: Metadata | null): number | null {
  const value = metadata?.target_weight_g
  if (typeof value === 'number') return value
  if (typeof value === 'string') {
    const parsed = Number.parseFloat(value)
    return Number.isFinite(parsed) ? parsed : null
  }
  return null
}

function timelineIndexFromEvents(
  phases: string[],
  isDone: boolean
): number {
  if (phases.length === 0) {
    return isDone ? TIMELINE_PHASES.length : -1
  }

  if (isDone) {
    return TIMELINE_PHASES.length
  }

  const normalized = phases.map((phase) => phase.toLowerCase())
  const last = normalized[normalized.length - 1] || ''
  const scoopStartIndex = normalized.indexOf('scoop_start')
  const scoopEndIndex = normalized.indexOf('scoop_end')
  const pourStartIndex = normalized.indexOf('pour_start')
  const pourEndIndex = normalized.indexOf('pour_end')

  if (last === 'scoop_start' || last === 'scoop_end') {
    return 1
  }

  if (last === 'pour_start' || last === 'pour_end') {
    return 3
  }

  if (last === 'transport_start' || last === 'transport_end') {
    if (pourEndIndex >= 0) {
      return 4
    }
    if (scoopEndIndex >= 0 || pourStartIndex >= 0) {
      return 2
    }
    if (scoopStartIndex >= 0) {
      return 1
    }
    return 0
  }

  return -1
}

function processedPhaseIndex(detail: DetailRow | null): number {
  const events = Array.isArray(detail?.robot_phase_events)
    ? detail?.robot_phase_events ?? []
    : []
  return timelineIndexFromEvents(
    events.map((event) => event.phase || ''),
    Boolean(detail?.robot_mes_weighment_sent || detail?.robot_status === 'succeeded')
  )
}

function phaseSequence(detail: DetailRow | null): string {
  const events = Array.isArray(detail?.robot_phase_events)
    ? detail?.robot_phase_events ?? []
    : []
  if (events.length === 0) return '—'
  let transportLeg = 0
  return events
    .map((event) => {
      const phase = (event.phase || '').toLowerCase()
      if (phase === 'transport_start') {
        transportLeg += 1
        if (transportLeg === 1) return 'Start move to scoop'
        if (transportLeg === 2) return 'Start move to weigh'
        return 'Start return to scoop'
      }
      if (phase === 'transport_end') {
        if (transportLeg === 1) return 'Reached scoop position'
        if (transportLeg === 2) return 'Reached weigh position'
        return 'Returned to scoop'
      }
      if (phase === 'scoop_start') return 'Start scooping'
      if (phase === 'scoop_end') return 'Scooping complete'
      if (phase === 'pour_start') return 'Start pouring'
      if (phase === 'pour_end') return 'Pouring complete'
      return event.phase
    })
    .join(' -> ')
}

function statusLabel(state: string | null): string {
  if (!state) return 'Idle'
  if (state === 'awaiting_processing') return 'Awaiting Processing'
  return state.replace(/_/g, ' ')
}

function formatPhaseLabel(phase: string | null | undefined): string {
  if (!phase) return '—'
  return phase
    .replace(/_/g, ' ')
    .replace(/\b\w/g, (c) => c.toUpperCase())
}

function App() {
  const navigate = useNavigate()
  const ros = useRos()
  const { apiBase } = useRuntimeConfig()

  const [runState, setRunState] = useState<string>('idle')
  const [livePhase, setLivePhase] = useState<string | null>(null)
  const [livePhaseEvents, setLivePhaseEvents] = useState<string[]>([])
  const [webhookActive, setWebhookActive] = useState(false)
  const [metadata, setMetadata] = useState<Metadata | null>(null)
  const [weight, setWeight] = useState<number | null>(null)
  const [lastWeightTs, setLastWeightTs] = useState<number | null>(null)
  const [pourStatus, setPourStatus] = useState<PourStatusMsg | null>(null)
  const [vibrationIntensity, setVibrationIntensity] = useState<number | null>(null)
  const [inclineAngleDeg, setInclineAngleDeg] = useState<number | null>(null)
  const [latestRun, setLatestRun] = useState<RobotRunRow | null>(null)
  const [latestDetail, setLatestDetail] = useState<DetailRow | null>(null)
  const [loading, setLoading] = useState(false)

  useEffect(() => {
    const r = ros
    if (!r) return

    const phaseTopic = new ROSLIB.Topic({
      ros: r,
      name: WEBHOOK_PHASE_TOPIC,
      messageType: 'std_msgs/String',
    })
    phaseTopic.subscribe((msg: { data: string }) => {
      const phase = (msg.data || '').toLowerCase()
      setLivePhase(phase)
      setLivePhaseEvents((prev) => [...prev, phase])
      setWebhookActive(true)
    })

    const activeTopic = new ROSLIB.Topic({
      ros: r,
      name: WEBHOOK_ACTIVE_TOPIC,
      messageType: 'std_msgs/Bool',
    })
    activeTopic.subscribe((msg: { data: boolean }) => {
      const isActive = Boolean(msg.data)
      setWebhookActive(isActive)
    })

    const metadataTopic = new ROSLIB.Topic({
      ros: r,
      name: WEBHOOK_METADATA_TOPIC,
      messageType: 'std_msgs/String',
    })
    metadataTopic.subscribe((msg: { data: string }) => {
      try {
        const parsed = JSON.parse(msg.data || '{}')
        setMetadata(parsed)
        setLivePhaseEvents([])
        setLivePhase(null)
      } catch {
        setMetadata(null)
      }
    })

    const weightTopic = new ROSLIB.Topic({
      ros: r,
      name: WEIGHT_TOPIC,
      messageType: 'std_msgs/Float64',
    })
    weightTopic.subscribe((msg: { data: number }) => {
      if (typeof msg.data === 'number' && Number.isFinite(msg.data)) {
        setWeight(msg.data)
        setLastWeightTs(Date.now())
      }
    })

    const pourStatusTopic = new ROSLIB.Topic({
      ros: r,
      name: POUR_STATUS_TOPIC,
      messageType: 'robot_common_msgs/PourStatus',
    })
    pourStatusTopic.subscribe((msg: PourStatusMsg) => {
      setPourStatus(msg)
    })

    const vibrationTopic = new ROSLIB.Topic({
      ros: r,
      name: VIBRATION_TOPIC,
      messageType: 'std_msgs/Float64',
    })
    vibrationTopic.subscribe((msg: { data: number }) => {
      if (typeof msg.data === 'number' && Number.isFinite(msg.data)) {
        setVibrationIntensity(msg.data)
      }
    })

    const inclineTopic = new ROSLIB.Topic({
      ros: r,
      name: INCLINE_TOPIC,
      messageType: 'std_msgs/Float64',
    })
    inclineTopic.subscribe((msg: { data: number }) => {
      if (typeof msg.data === 'number' && Number.isFinite(msg.data)) {
        setInclineAngleDeg(msg.data)
      }
    })

    const runStateTopic = new ROSLIB.Topic({
      ros: r,
      name: RUN_STATE_TOPIC,
      messageType: 'std_msgs/String',
    })
    runStateTopic.subscribe((msg: { data: string }) => {
      setRunState((msg.data || 'idle').toLowerCase())
    })

    return () => {
      phaseTopic.unsubscribe()
      activeTopic.unsubscribe()
      metadataTopic.unsubscribe()
      weightTopic.unsubscribe()
      pourStatusTopic.unsubscribe()
      vibrationTopic.unsubscribe()
      inclineTopic.unsubscribe()
      runStateTopic.unsubscribe()
    }
  }, [ros])

  useEffect(() => {
    let cancelled = false

    async function fetchLatestRun() {
      try {
        setLoading(true)
        const runRes = await fetch(`${apiBase}/robot_weightment_runs?limit=1`)
        const runJson = await runRes.json()
        const run = (runJson.rows?.[0] || null) as RobotRunRow | null
        if (cancelled) return
        setLatestRun(run)
        if (!run?.event_id) {
          setLatestDetail(null)
          return
        }
        const detailRes = await fetch(
          `${apiBase}/webhook_weightments/${encodeURIComponent(run.event_id)}`
        )
        const detailJson = await detailRes.json()
        const row =
          (detailJson.rows || []).find(
            (item: DetailRow) => item.weightment_id === run.weightment_id
          ) || null
        if (!cancelled) {
          setLatestDetail(row)
        }
      } catch {
        if (!cancelled) {
          setLatestRun(null)
          setLatestDetail(null)
        }
      } finally {
        if (!cancelled) {
          setLoading(false)
        }
      }
    }

    fetchLatestRun()
    const id = setInterval(fetchLatestRun, 5000)
    return () => {
      cancelled = true
      clearInterval(id)
    }
  }, [apiBase])

  const targetWeightG = useMemo(() => {
    return (
      parseTargetWeight(metadata) ??
      latestRun?.target_weight_g ??
      (typeof latestDetail?.target_weight_kg === 'number'
        ? latestDetail.target_weight_kg * 1000
        : null)
    )
  }, [metadata, latestRun, latestDetail])

  const phaseIndex = useMemo(() => {
    const showLiveTimeline =
      webhookActive ||
      runState === 'starting' ||
      runState === 'running' ||
      livePhaseEvents.length > 0
    if (showLiveTimeline) {
      return timelineIndexFromEvents(livePhaseEvents, runState === 'succeeded')
    }
    return processedPhaseIndex(latestDetail)
  }, [latestDetail, livePhaseEvents, runState, webhookActive])

  const showLiveTimeline =
    webhookActive ||
    runState === 'starting' ||
    runState === 'running' ||
    livePhaseEvents.length > 0

  const weightStale =
    !lastWeightTs || Date.now() - lastWeightTs > 1500

  const pourPhaseLabel = useMemo(() => {
    if (pourStatus?.active) return formatPhaseLabel(pourStatus.phase)
    if (livePhase === 'pour_start') return 'Starting'
    if (livePhase === 'pour_end') return 'Complete'
    return '—'
  }, [livePhase, pourStatus])

  const pourTelemetryVisible = showLiveTimeline && (
    Boolean(pourStatus?.active) ||
    livePhase === 'pour_start' ||
    livePhase === 'pour_end' ||
    (typeof vibrationIntensity === 'number' && vibrationIntensity > 0) ||
    (typeof inclineAngleDeg === 'number' && inclineAngleDeg > 0)
  )

  const scalePct = useMemo(() => {
    if (typeof weight !== 'number' || typeof targetWeightG !== 'number') return 0
    const maxRange = Math.max(targetWeightG * 1.4, 200)
    return Math.max(0, Math.min(100, (weight / maxRange) * 100))
  }, [targetWeightG, weight])

  const targetPct = useMemo(() => {
    if (typeof targetWeightG !== 'number') return 0
    const maxRange = Math.max(targetWeightG * 1.4, 200)
    return Math.max(0, Math.min(100, (targetWeightG / maxRange) * 100))
  }, [targetWeightG])

  return (
    <div className="px-6 py-6">
      <div className="mb-4 flex items-end justify-between gap-2">
        <div className="flex flex-col gap-1">
          <h1
            className="text-2xl font-bold tracking-tight"
            style={{ fontFamily: 'Space Grotesk' }}
          >
            Batch Execution Dashboard
          </h1>
          {/* <p className="text-[var(--text-secondary)]">
            Monitor live batch execution, robot phase, weight, and the latest
            processed result.
          </p> */}
        </div>
        <div className="flex items-center gap-2">
          <Button onClick={() => navigate('/batches')}>
            View Batches
          </Button>
          <Button variant="ghost" onClick={() => navigate('/logs')}>
            View Run History
          </Button>
        </div>
      </div>

      <div className="grid grid-cols-12 gap-4">
        {/* <div className="col-span-12 md:col-span-3">
          <KpiCard
            label="Robot State"
            value={statusLabel(webhookActive ? runState : latestRun?.status || runState)}
            help={webhookActive ? 'Live from ROS' : 'Latest known batch run'}
          />
        </div>
        <div className="col-span-12 md:col-span-3">
          <KpiCard
            label="Weightment"
            value={
              metadata?.weightment_id ||
              (latestRun?.weightment_id != null
                ? String(latestRun.weightment_id)
                : '—')
            }
          />
        </div>
        <div className="col-span-12 md:col-span-3">
          <KpiCard label="Batch" value={metadata?.batch_id || latestRun?.batch_id || '—'} />
        </div>
        <div className="col-span-12 md:col-span-3">
          <KpiCard
            label="MES Status"
            value={
              latestRun?.mes_weighment_sent
                ? 'Sent'
                : latestRun?.status === 'awaiting_processing'
                  ? 'Waiting on Trace'
                  : 'Pending'
            }
          />
        </div> */}

        <div className="col-span-12">
          <GlassCard>
            <div className="mb-2 flex items-center justify-between">
              <h3 className="text-lg font-semibold">Robot Timeline</h3>
              <span
                className={`rounded-md px-2 py-0.5 text-xs ${
                  showLiveTimeline
                    ? 'bg-[var(--status-good-bg)] text-[var(--status-good-fg)]'
                    : 'bg-[var(--status-idle-bg)] text-[var(--status-idle-fg)]'
                }`}
              >
                {showLiveTimeline ? 'Active batch run' : 'Latest run'}
              </span>
            </div>
            <PhaseTimeline phases={TIMELINE_PHASES} index={phaseIndex} />
            <div className="mt-4 grid gap-3 md:grid-cols-3">
              <div className="rounded-lg border border-[var(--border)] bg-[var(--surface-muted)] px-3 py-2">
                <div className="text-xs uppercase tracking-wide text-[var(--text-faint)]">
                  Pour Phase
                </div>
                <div className="mt-1 text-sm font-medium text-[var(--text-primary)]">
                  {pourTelemetryVisible ? pourPhaseLabel : '—'}
                </div>
              </div>
              <div className="rounded-lg border border-[var(--border)] bg-[var(--surface-muted)] px-3 py-2">
                <div className="text-xs uppercase tracking-wide text-[var(--text-faint)]">
                  Vibration Command
                </div>
                <div className="mt-1 text-sm font-medium text-[var(--text-primary)]">
                  {pourTelemetryVisible && typeof vibrationIntensity === 'number'
                    ? vibrationIntensity.toFixed(2)
                    : '—'}
                </div>
              </div>
              <div className="rounded-lg border border-[var(--border)] bg-[var(--surface-muted)] px-3 py-2">
                <div className="text-xs uppercase tracking-wide text-[var(--text-faint)]">
                  Incline Command
                </div>
                <div className="mt-1 text-sm font-medium text-[var(--text-primary)]">
                  {pourTelemetryVisible && typeof inclineAngleDeg === 'number'
                    ? `${inclineAngleDeg.toFixed(1)}°`
                    : '—'}
                </div>
              </div>
            </div>
            {/* <div className="mt-3 text-sm text-[var(--text-muted)]">
              {showLiveTimeline
                ? livePhaseEvents.join(' -> ') || livePhase || 'Waiting for execution phase markers...'
                : phaseSequence(latestDetail)}
            </div> */}
          </GlassCard>
        </div>

        <div className="col-span-12 md:col-span-5">
          <GlassCard>
            <div className="flex flex-col gap-3">
              <div className="flex items-center justify-between">
                <h3 className="text-lg font-semibold">Weighing Scale</h3>
                <div className="flex items-center gap-2">
                  <span
                    className={`rounded-md px-2 py-0.5 text-xs ${
                      webhookActive
                        ? 'bg-[var(--status-good-bg)] text-[var(--status-good-fg)]'
                        : 'bg-[var(--status-idle-bg)] text-[var(--status-idle-fg)]'
                    }`}
                  >
                    {webhookActive ? 'Active' : 'Idle'}
                  </span>
                  <span
                    className={`rounded-md px-2 py-0.5 text-xs ${
                      weightStale
                        ? 'bg-[var(--status-idle-bg)] text-[var(--status-idle-fg)]'
                        : 'bg-[var(--status-info-bg)] text-[var(--status-info-fg)]'
                    }`}
                  >
                    {weightStale ? 'Scale Disconnected' : 'Scale Connected'}
                  </span>
                </div>
              </div>
              <div className="flex items-center justify-between text-sm text-[var(--text-secondary)]">
                <span>Target</span>
                <span>{typeof targetWeightG === 'number' ? `${Math.round(targetWeightG)} g` : '—'}</span>
              </div>
              <div className="h-4 w-full rounded bg-[var(--surface-strong)]">
                <div className="relative h-full w-full">
                  <div
                    className="absolute top-0 bottom-0 rounded bg-[var(--status-good-bg)]"
                    style={{ width: `${scalePct}%` }}
                  />
                  <div
                    className="absolute top-0 bottom-0 w-[2px] rounded bg-[var(--status-good-fg)]"
                    style={{ left: `${targetPct}%`, transform: 'translateX(-1px)' }}
                  />
                </div>
              </div>
              <div className="flex items-center justify-between text-xs text-[var(--text-muted)]">
                <span>{typeof weight === 'number' ? `${Math.round(weight)} g` : '—'}</span>
                <span>{WEIGHT_TOPIC}</span>
              </div>
            </div>
          </GlassCard>
        </div>

        <div className="col-span-12 md:col-span-7">
          <GlassCard>
            <div className="flex flex-col gap-2">
              <div className="flex items-center justify-between">
                <h3 className="text-lg font-semibold">Current / Latest Run</h3>
                <span className="text-xs text-[var(--text-faint)]">
                  {loading ? 'Refreshing…' : 'Auto refresh 5s'}
                </span>
              </div>
              {latestRun ? (
                <div className="grid grid-cols-2 gap-y-2 text-sm md:grid-cols-3">
                  <div>
                    <span className="text-[var(--text-muted)]">Trace</span>
                    <div>{latestRun.trace_run_id || '—'}</div>
                  </div>
                  <div>
                    <span className="text-[var(--text-muted)]">Weightment</span>
                    <div>{latestRun.weightment_id}</div>
                  </div>
                  <div>
                    <span className="text-[var(--text-muted)]">Batch</span>
                    <div>{latestRun.batch_id || metadata?.batch_id || '—'}</div>
                  </div>
                  <div>
                    <span className="text-[var(--text-muted)]">Ingredient</span>
                    <div>{latestRun.ingredient_name || latestRun.ingredient_id || '—'}</div>
                  </div>
                  <div>
                    <span className="text-[var(--text-muted)]">Location</span>
                    <div>{latestRun.stock_location_code || '—'}</div>
                  </div>
                  <div>
                    <span className="text-[var(--text-muted)]">Status</span>
                    <div>{statusLabel(latestRun.status)}</div>
                  </div>
                  <div>
                    <span className="text-[var(--text-muted)]">Requested</span>
                    <DateTimeText value={latestRun.requested_at} />
                  </div>
                  <div>
                    <span className="text-[var(--text-muted)]">Started</span>
                    <DateTimeText value={latestRun.started_at} />
                  </div>
                  <div>
                    <span className="text-[var(--text-muted)]">Finished</span>
                    <DateTimeText value={latestRun.finished_at} />
                  </div>
                </div>
              ) : (
                <span className="text-sm text-[var(--text-secondary)]">
                  No batch robot run found yet.
                </span>
              )}
            </div>
          </GlassCard>
        </div>

      </div>
    </div>
  )
}

export default App


