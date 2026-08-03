import { useEffect, useMemo, useState } from 'react'
import { useNavigate } from 'react-router-dom'
import MetricCard from './components/ui/MetricCard'
import { SectionHeader } from './components/ui/SectionHeader'
import BatchQueueStrip from './components/operations/BatchQueueStrip'
import LiveWeightChart from './components/operations/LiveWeightChart'
import PhasePanel from './components/operations/PhasePanel'
import RunContextCard from './components/operations/RunContextCard'
import WeightHero from './components/operations/WeightHero'
import {
  formatPhaseLabel,
  statusLabel,
  timelineIndexFromEvents,
} from './components/operations/operationsUtils'
import Button from './components/ui/button'
import { useRuntimeConfig } from './config/RuntimeConfig'
import { useRuntimeMode } from './hooks/useRuntimeMode'
import { useRos } from './ros/RosContext'
import { ROSLIB } from './ros/roslib'

type Metadata = {
  run_id?: string
  weightment_id?: string
  batch_id?: string
  ingredient_id?: string
  target_weight_g?: number | string
}

type RobotRunRow = {
  id: number
  weightment_id: number
  event_id: string | null
  batch_id: string | null
  ingredient_id: string | null
  ingredient_name: string | null
  stock_location_code: string | null
  target_weight_g: number | null
  trace_run_id: string | null
  status: string | null
  requested_at: string | null
  started_at: string | null
  finished_at: string | null
  mes_weighment_sent: boolean
  mes_batch_end_sent: boolean
}

type DetailRow = {
  weightment_id: number
  target_weight_kg: number | null
  completed: boolean
  stock_item_id: string | null
  ingredient_name: string | null
  robot_status: string | null
  robot_mes_weighment_sent: boolean
  robot_phase_events: Array<{ t_ns: number; phase: string }> | null
}

type PourStatusMsg = {
  active?: boolean
  phase?: string
}

const WEIGHT_TOPIC = (import.meta as any).env.VITE_WEIGHT_TOPIC || '/weight'
const DEFAULT_WEBHOOK_PHASE_TOPIC =
  (import.meta as any).env.VITE_WEBHOOK_PHASE_TOPIC || '/webhook_run/phase'
const LIGHTSOUT_PHASE_TOPIC =
  (import.meta as any).env.VITE_LIGHTSOUT_PHASE_TOPIC || '/lightsout_training/phase'
const WEBHOOK_ACTIVE_TOPIC =
  (import.meta as any).env.VITE_WEBHOOK_ACTIVE_TOPIC || '/webhook_run/active'
const WEBHOOK_METADATA_TOPIC =
  (import.meta as any).env.VITE_WEBHOOK_METADATA_TOPIC || '/webhook_run/metadata'
const RUN_STATE_TOPIC =
  (import.meta as any).env.VITE_RUN_STATE_TOPIC || '/orchestrator/run_state'
const POUR_STATUS_TOPIC =
  (import.meta as any).env.VITE_POUR_STATUS_TOPIC || '/pour_status'
const VIBRATION_TOPIC =
  (import.meta as any).env.VITE_VIBRATION_TOPIC || '/vibration/intensity'
const INCLINE_TOPIC =
  (import.meta as any).env.VITE_INCLINE_TOPIC || '/incline_control'

function parseTargetWeight(metadata: Metadata | null): number | null {
  const value = metadata?.target_weight_g
  if (typeof value === 'number') return value
  if (typeof value === 'string') {
    const parsed = Number.parseFloat(value)
    return Number.isFinite(parsed) ? parsed : null
  }
  return null
}

function processedPhaseIndex(detail: DetailRow | null): number {
  const events = detail?.robot_phase_events ?? []
  return timelineIndexFromEvents(
    events.map((e) => e.phase || ''),
    Boolean(detail?.robot_mes_weighment_sent || detail?.robot_status === 'succeeded')
  )
}

function App() {
  const navigate = useNavigate()
  const ros = useRos()
  const { apiBase } = useRuntimeConfig()
  const { runtime, mesSinkDisabled } = useRuntimeMode(apiBase)
  const activeMode = runtime?.mode || '—'

  const phaseTopic =
    runtime?.mode === 'lightsout' ? LIGHTSOUT_PHASE_TOPIC : DEFAULT_WEBHOOK_PHASE_TOPIC

  const [runState, setRunState] = useState('idle')
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
  const [queueItems, setQueueItems] = useState<DetailRow[]>([])
  const [loading, setLoading] = useState(false)

  useEffect(() => {
    const r = ros
    if (!r) return

    const subs = [
      new ROSLIB.Topic({ ros: r, name: phaseTopic, messageType: 'std_msgs/String' }),
      new ROSLIB.Topic({ ros: r, name: WEBHOOK_ACTIVE_TOPIC, messageType: 'std_msgs/Bool' }),
      new ROSLIB.Topic({ ros: r, name: WEBHOOK_METADATA_TOPIC, messageType: 'std_msgs/String' }),
      new ROSLIB.Topic({ ros: r, name: WEIGHT_TOPIC, messageType: 'std_msgs/Float64' }),
      new ROSLIB.Topic({ ros: r, name: POUR_STATUS_TOPIC, messageType: 'robot_common_msgs/PourStatus' }),
      new ROSLIB.Topic({ ros: r, name: VIBRATION_TOPIC, messageType: 'std_msgs/Float64' }),
      new ROSLIB.Topic({ ros: r, name: INCLINE_TOPIC, messageType: 'std_msgs/Float64' }),
      new ROSLIB.Topic({ ros: r, name: RUN_STATE_TOPIC, messageType: 'std_msgs/String' }),
    ]

    subs[0].subscribe((msg: { data: string }) => {
      const phase = (msg.data || '').toLowerCase()
      setLivePhase(phase)
      setLivePhaseEvents((prev) => [...prev, phase])
      setWebhookActive(true)
    })
    subs[1].subscribe((msg: { data: boolean }) => setWebhookActive(Boolean(msg.data)))
    subs[2].subscribe((msg: { data: string }) => {
      try {
        setMetadata(JSON.parse(msg.data || '{}'))
        setLivePhaseEvents([])
        setLivePhase(null)
      } catch {
        setMetadata(null)
      }
    })
    subs[3].subscribe((msg: { data: number }) => {
      if (typeof msg.data === 'number' && Number.isFinite(msg.data)) {
        setWeight(msg.data)
        setLastWeightTs(Date.now())
      }
    })
    subs[4].subscribe((msg: PourStatusMsg) => setPourStatus(msg))
    subs[5].subscribe((msg: { data: number }) => {
      if (typeof msg.data === 'number') setVibrationIntensity(msg.data)
    })
    subs[6].subscribe((msg: { data: number }) => {
      if (typeof msg.data === 'number') setInclineAngleDeg(msg.data)
    })
    subs[7].subscribe((msg: { data: string }) => setRunState((msg.data || 'idle').toLowerCase()))

    return () => subs.forEach((t) => t.unsubscribe())
  }, [ros, phaseTopic])

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
          setQueueItems([])
          return
        }
        const detailRes = await fetch(
          `${apiBase}/webhook_weightments/${encodeURIComponent(run.event_id)}`
        )
        const detailJson = await detailRes.json()
        const rows = (detailJson.rows || []) as DetailRow[]
        const row = rows.find((item) => item.weightment_id === run.weightment_id) || null
        if (!cancelled) {
          setLatestDetail(row)
          setQueueItems(rows)
        }
      } catch {
        if (!cancelled) {
          setLatestRun(null)
          setLatestDetail(null)
          setQueueItems([])
        }
      } finally {
        if (!cancelled) setLoading(false)
      }
    }
    fetchLatestRun()
    const id = setInterval(fetchLatestRun, 5000)
    return () => {
      cancelled = true
      clearInterval(id)
    }
  }, [apiBase])

  const targetWeightG = useMemo(
    () =>
      parseTargetWeight(metadata) ??
      latestRun?.target_weight_g ??
      (typeof latestDetail?.target_weight_kg === 'number'
        ? latestDetail.target_weight_kg * 1000
        : null),
    [metadata, latestRun, latestDetail]
  )

  const targetToleranceG = useMemo(
    () => (typeof targetWeightG === 'number' ? targetWeightG * 0.02 : null),
    [targetWeightG]
  )

  const showLiveTimeline =
    webhookActive ||
    runState === 'starting' ||
    runState === 'running' ||
    livePhaseEvents.length > 0

  const phaseIndex = useMemo(() => {
    if (showLiveTimeline) {
      return timelineIndexFromEvents(livePhaseEvents, runState === 'succeeded')
    }
    return processedPhaseIndex(latestDetail)
  }, [latestDetail, livePhaseEvents, runState, showLiveTimeline])

  const weightStale = !lastWeightTs || Date.now() - lastWeightTs > 1500

  const pourPhaseLabel = useMemo(() => {
    if (pourStatus?.active) return formatPhaseLabel(pourStatus.phase)
    if (livePhase === 'pour_start') return 'Starting'
    if (livePhase === 'pour_end') return 'Complete'
    return '—'
  }, [livePhase, pourStatus])

  const pourTelemetryVisible =
    showLiveTimeline &&
    (Boolean(pourStatus?.active) ||
      livePhase === 'pour_start' ||
      livePhase === 'pour_end' ||
      (typeof vibrationIntensity === 'number' && vibrationIntensity > 0) ||
      (typeof inclineAngleDeg === 'number' && inclineAngleDeg > 0))

  const batchProgress =
    queueItems.length > 0
      ? `${queueItems.filter((i) => i.completed).length}/${queueItems.length}`
      : '—'

  return (
    <div className="px-5 py-5 lg:px-6">
      <SectionHeader
        title={`Operations — ${activeMode}`}
        description="Live robot execution, weight telemetry, and batch progress."
        action={
          <div className="flex gap-2">
            <Button variant="outline" onClick={() => navigate('/batches')}>
              Batches
            </Button>
            <Button variant="ghost" onClick={() => navigate('/logs')}>
              History
            </Button>
          </div>
        }
      />

      <div className="mb-4 grid grid-cols-2 gap-3 lg:grid-cols-4">
        <MetricCard
          label="Robot State"
          value={statusLabel(webhookActive ? runState : latestRun?.status || runState)}
        />
        <MetricCard
          label="Weightment"
          value={
            metadata?.weightment_id ||
            (latestRun?.weightment_id != null ? String(latestRun.weightment_id) : '—')
          }
        />
        <MetricCard
          label="Batch Progress"
          value={mesSinkDisabled ? 'N/A' : batchProgress}
          unit={!mesSinkDisabled && queueItems.length > 0 ? 'done' : undefined}
        />
        <MetricCard
          label="MES"
          value={
            mesSinkDisabled
              ? 'N/A'
              : latestRun?.mes_weighment_sent
                ? 'Sent'
                : latestRun?.status === 'awaiting_processing'
                  ? 'Processing'
                  : 'Pending'
          }
        />
      </div>

      <div className="grid grid-cols-1 gap-4 xl:grid-cols-12">
        <div className="xl:col-span-5">
          <WeightHero
            weight={weight}
            targetWeightG={targetWeightG}
            targetToleranceG={targetToleranceG}
            weightStale={weightStale}
            webhookActive={webhookActive}
          />
        </div>
        <div className="xl:col-span-7">
          <LiveWeightChart
            weight={weight}
            targetWeightG={targetWeightG}
            targetToleranceG={targetToleranceG}
          />
        </div>
        <div className="xl:col-span-7">
          <PhasePanel
            phaseIndex={phaseIndex}
            showLive={showLiveTimeline}
            pourPhaseLabel={pourPhaseLabel}
            vibrationIntensity={vibrationIntensity}
            inclineAngleDeg={inclineAngleDeg}
            pourTelemetryVisible={pourTelemetryVisible}
          />
        </div>
        <div className="xl:col-span-5">
          <RunContextCard
            run={latestRun}
            loading={loading}
            metadataBatchId={metadata?.batch_id}
            mesSinkDisabled={mesSinkDisabled}
          />
        </div>
        <div
          className={`col-span-1 xl:col-span-12 ${
            mesSinkDisabled ? 'opacity-50' : ''
          }`}
        >
          <BatchQueueStrip
            items={queueItems}
            activeWeightmentId={latestRun?.weightment_id}
          />
        </div>
      </div>
    </div>
  )
}

export default App
