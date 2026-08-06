import { useEffect, useMemo, useState } from 'react'
import { useNavigate } from 'react-router-dom'
import MetricCard from './components/ui/MetricCard'
import { SectionHeader } from './components/ui/SectionHeader'
import BatchQueueStrip from './components/operations/BatchQueueStrip'
import EpisodeStrip from './components/operations/EpisodeStrip'
import EpisodeWeightHero from './components/operations/EpisodeWeightHero'
import LiveWeightChart from './components/operations/LiveWeightChart'
import PhasePanel from './components/operations/PhasePanel'
import RunContextCard from './components/operations/RunContextCard'
import SessionProgressCard from './components/operations/SessionProgressCard'
import WeightHero from './components/operations/WeightHero'
import { operationsProfile } from './components/operations/operationsProfile'
import {
  episodePhaseKind,
  formatNumber,
  formatPhaseLabel,
  lightsoutTimelineIndex,
  statusLabel,
  timelineIndexFromEvents,
} from './components/operations/operationsUtils'
import Button from './components/ui/button'
import StatusBadge from './components/ui/StatusBadge'
import DateTimeText from './components/DateTimeText'
import { useRuntimeConfig } from './config/RuntimeConfig'
import { useLightsoutLive } from './hooks/useLightsoutLive'
import { usePhaseBaselines } from './hooks/usePhaseBaselines'
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
  ingredient_id: string | null
  robot_status: string | null
  robot_mes_weighment_sent: boolean
  robot_phase_events: Array<{ t_ns: number; phase: string }> | null
}

type PourStatusMsg = {
  active?: boolean
  phase?: string
}

const WEIGHT_TOPIC = (import.meta as any).env.VITE_WEIGHT_TOPIC || '/weight'
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
  const { runtime } = useRuntimeMode(apiBase)
  const profile = useMemo(() => operationsProfile(runtime?.mode), [runtime?.mode])
  const activeMode = runtime?.mode || '—'
  const isLightsout = profile.family === 'lightsout'

  const [runState, setRunState] = useState('idle')
  const [livePhase, setLivePhase] = useState<string | null>(null)
  const [livePhaseEvents, setLivePhaseEvents] = useState<string[]>([])
  const [topicActive, setTopicActive] = useState(false)
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

  const lightsout = useLightsoutLive(isLightsout, apiBase, runState, livePhase)
  const baselines = usePhaseBaselines(weight, livePhase)

  useEffect(() => {
    const r = ros
    if (!r) return

    const phaseTopic = new ROSLIB.Topic({
      ros: r,
      name: profile.phaseTopic,
      messageType: 'std_msgs/String',
    })
    const weightTopic = new ROSLIB.Topic({
      ros: r,
      name: WEIGHT_TOPIC,
      messageType: 'std_msgs/Float64',
    })
    const pourTopic = new ROSLIB.Topic({
      ros: r,
      name: POUR_STATUS_TOPIC,
      messageType: 'robot_common_msgs/PourStatus',
    })
    const vibeTopic = new ROSLIB.Topic({
      ros: r,
      name: VIBRATION_TOPIC,
      messageType: 'std_msgs/Float64',
    })
    const inclineTopic = new ROSLIB.Topic({
      ros: r,
      name: INCLINE_TOPIC,
      messageType: 'std_msgs/Float64',
    })
    const runStateTopic = new ROSLIB.Topic({
      ros: r,
      name: RUN_STATE_TOPIC,
      messageType: 'std_msgs/String',
    })

    const subs: { unsubscribe: () => void }[] = [
      phaseTopic,
      weightTopic,
      pourTopic,
      vibeTopic,
      inclineTopic,
      runStateTopic,
    ]

    phaseTopic.subscribe((msg: { data: string }) => {
      const phase = (msg.data || '').toLowerCase()
      setLivePhase(phase)
      setLivePhaseEvents((prev) => [...prev, phase])
      setTopicActive(true)
    })
    weightTopic.subscribe((msg: { data: number }) => {
      if (typeof msg.data === 'number' && Number.isFinite(msg.data)) {
        setWeight(msg.data)
        setLastWeightTs(Date.now())
      }
    })
    pourTopic.subscribe((msg: PourStatusMsg) => setPourStatus(msg))
    vibeTopic.subscribe((msg: { data: number }) => {
      if (typeof msg.data === 'number') setVibrationIntensity(msg.data)
    })
    inclineTopic.subscribe((msg: { data: number }) => {
      if (typeof msg.data === 'number') setInclineAngleDeg(msg.data)
    })
    runStateTopic.subscribe((msg: { data: string }) =>
      setRunState((msg.data || 'idle').toLowerCase())
    )

    if (profile.activeTopic && !isLightsout) {
      const activeTopic = new ROSLIB.Topic({
        ros: r,
        name: profile.activeTopic,
        messageType: 'std_msgs/Bool',
      })
      activeTopic.subscribe((msg: { data: boolean }) => setTopicActive(Boolean(msg.data)))
      subs.push(activeTopic)

      const metaTopic = new ROSLIB.Topic({
        ros: r,
        name: WEBHOOK_METADATA_TOPIC,
        messageType: 'std_msgs/String',
      })
      metaTopic.subscribe((msg: { data: string }) => {
        try {
          setMetadata(JSON.parse(msg.data || '{}'))
          setLivePhaseEvents([])
          setLivePhase(null)
          baselines.reset()
        } catch {
          setMetadata(null)
        }
      })
      subs.push(metaTopic)
    }

    return () => subs.forEach((t) => t.unsubscribe())
    // baselines.reset is stable enough; avoid re-sub on every render
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [ros, profile.phaseTopic, profile.activeTopic, isLightsout])

  // Reset phase stream when mode family changes.
  useEffect(() => {
    setLivePhase(null)
    setLivePhaseEvents([])
    setTopicActive(false)
    setMetadata(null)
    baselines.reset()
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [profile.family])

  useEffect(() => {
    if (isLightsout) {
      setLatestRun(null)
      setLatestDetail(null)
      setQueueItems([])
      setLoading(false)
      return
    }
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
        const row =
          rows.find((item) => item.weightment_id === run.weightment_id) || null
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
  }, [apiBase, isLightsout])

  const runActive = isLightsout
    ? lightsout.runActive
    : topicActive || runState === 'running' || runState === 'starting'

  const mesTargetWeightG = useMemo(
    () =>
      parseTargetWeight(metadata) ??
      latestRun?.target_weight_g ??
      (typeof latestDetail?.target_weight_kg === 'number'
        ? latestDetail.target_weight_kg * 1000
        : null),
    [metadata, latestRun, latestDetail]
  )

  const targetWeightG = isLightsout ? lightsout.targetWeightG : mesTargetWeightG

  const targetToleranceG = useMemo(() => {
    if (typeof targetWeightG !== 'number') return null
    if (profile.toleranceFrac != null) return Math.max(0.1, targetWeightG * profile.toleranceFrac)
    return targetWeightG * 0.02
  }, [targetWeightG, profile.toleranceFrac])

  const showLiveTimeline =
    runActive ||
    runState === 'starting' ||
    runState === 'running' ||
    livePhaseEvents.length > 0

  const phaseIndex = useMemo(() => {
    if (isLightsout) {
      const episodeEnded =
        lightsout.episodeEnd != null &&
        lightsout.episode != null &&
        lightsout.episodeEnd >= lightsout.episode
      return lightsoutTimelineIndex(
        livePhaseEvents,
        runState === 'succeeded',
        episodeEnded && !runActive
      )
    }
    if (showLiveTimeline) {
      return timelineIndexFromEvents(livePhaseEvents, runState === 'succeeded')
    }
    return processedPhaseIndex(latestDetail)
  }, [
    isLightsout,
    lightsout.episodeEnd,
    lightsout.episode,
    livePhaseEvents,
    runState,
    runActive,
    showLiveTimeline,
    latestDetail,
  ])

  const phaseKind = episodePhaseKind(
    livePhase,
    phaseIndex,
    profile.timelinePhases.length
  )

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

  const scoopedDisplay =
    phaseKind === 'scooping'
      ? baselines.scoopedLiveG
      : lightsout.scoopedMassG ?? baselines.scoopedFinalG ?? baselines.scoopedLiveG

  // Chart reference lines on absolute scale for scoop-return.
  const chartRefWeight = useMemo(() => {
    if (!isLightsout) return targetWeightG
    if (phaseKind === 'pouring' && baselines.pourBaselineG != null && targetWeightG != null) {
      return baselines.pourBaselineG + targetWeightG
    }
    if (
      (phaseKind === 'scooping' || phaseKind === 'weighing') &&
      baselines.episodeBaselineG != null &&
      lightsout.sessionApi?.session?.min_scooped_g != null
    ) {
      return (
        baselines.episodeBaselineG - Number(lightsout.sessionApi.session.min_scooped_g)
      )
    }
    return null
  }, [
    isLightsout,
    phaseKind,
    baselines.pourBaselineG,
    baselines.episodeBaselineG,
    targetWeightG,
    lightsout.sessionApi,
  ])

  const chartRefTol = isLightsout
    ? phaseKind === 'pouring'
      ? targetToleranceG
      : null
    : targetToleranceG

  const mesContextFields = latestRun
    ? [
        { label: 'Trace', value: latestRun.trace_run_id || '—' },
        { label: 'Weightment', value: latestRun.weightment_id },
        {
          label: 'Batch',
          value: latestRun.batch_id || metadata?.batch_id || '—',
        },
        {
          label: 'Ingredient',
          value: latestRun.ingredient_name || latestRun.ingredient_id || '—',
        },
        { label: 'Location', value: latestRun.stock_location_code || '—' },
        {
          label: 'Requested',
          value: <DateTimeText value={latestRun.requested_at} />,
        },
        {
          label: 'Started',
          value: <DateTimeText value={latestRun.started_at} />,
        },
        {
          label: 'Finished',
          value: <DateTimeText value={latestRun.finished_at} />,
        },
      ]
    : []

  const loSession = lightsout.sessionApi?.session
  const loContextFields =
    lightsout.runId || loSession || lightsout.batchId
      ? [
          { label: 'Run ID', value: lightsout.runId || '—' },
          { label: 'Session / batch', value: lightsout.batchId || '—' },
          {
            label: 'Powder',
            value:
              lightsout.ingredientId ||
              loSession?.powder_name ||
              lightsout.powderId ||
              '—',
          },
          { label: 'Lot', value: lightsout.lotCode || '—' },
          { label: 'Operator', value: lightsout.operator || '—' },
          {
            label: 'Layout',
            value: loSession?.layout_id
              ? `${loSession.layout_id}${
                  loSession.layout_hash
                    ? ` · ${String(loSession.layout_hash).slice(0, 8)}`
                    : ''
                }`
              : runtime?.layout_id || '—',
          },
          {
            label: 'Target mode',
            value: loSession?.target_mode || '—',
          },
          {
            label: 'Started',
            value:
              lightsout.sessionApi?.started_at != null ? (
                <DateTimeText
                  value={new Date(lightsout.sessionApi.started_at * 1000).toISOString()}
                />
              ) : (
                '—'
              ),
          },
        ]
      : []

  return (
    <div className="px-5 py-5 lg:px-6">
      <SectionHeader
        title={`Operations — ${activeMode}`}
        description={
          isLightsout
            ? 'Live lights-out training: scoop-and-return on a single vessel.'
            : 'Live robot execution, weight telemetry, and batch progress.'
        }
        action={
          <div className="flex gap-2">
            {!isLightsout && (
              <Button variant="outline" onClick={() => navigate('/batches')}>
                Batches
              </Button>
            )}
            {isLightsout && (
              <Button variant="outline" onClick={() => navigate('/training')}>
                Training
              </Button>
            )}
            <Button variant="ghost" onClick={() => navigate('/logs')}>
              History
            </Button>
          </div>
        }
      />

      <div className="mb-4 grid grid-cols-2 gap-3 lg:grid-cols-4">
        <MetricCard
          label="Robot State"
          value={statusLabel(
            runActive ? runState : isLightsout ? runState : latestRun?.status || runState
          )}
        />
        {isLightsout ? (
          <>
            <MetricCard
              label="Episode"
              value={
                lightsout.episodesTotal != null
                  ? `${lightsout.episode ?? 0}/${lightsout.episodesTotal}`
                  : lightsout.episode != null
                    ? String(lightsout.episode)
                    : '—'
              }
            />
            <MetricCard
              label="Episode Target"
              value={
                lightsout.targetPending
                  ? 'pending'
                  : typeof targetWeightG === 'number'
                    ? Math.round(targetWeightG)
                    : '—'
              }
              unit={
                lightsout.targetPending || typeof targetWeightG !== 'number'
                  ? undefined
                  : 'g'
              }
            />
            <MetricCard
              label="Total Poured"
              value={
                typeof lightsout.totalPouredG === 'number'
                  ? formatNumber(lightsout.totalPouredG, 1)
                  : '—'
              }
              unit={typeof lightsout.totalPouredG === 'number' ? 'g' : undefined}
            />
          </>
        ) : (
          <>
            <MetricCard
              label="Weightment"
              value={
                metadata?.weightment_id ||
                (latestRun?.weightment_id != null
                  ? String(latestRun.weightment_id)
                  : '—')
              }
            />
            <MetricCard
              label="Batch Progress"
              value={profile.showBatchQueue ? batchProgress : 'N/A'}
              unit={
                profile.showBatchQueue && queueItems.length > 0 ? 'done' : undefined
              }
            />
            <MetricCard
              label="MES"
              value={
                !profile.showMesStatus
                  ? 'N/A'
                  : latestRun?.mes_weighment_sent
                    ? 'Sent'
                    : latestRun?.status === 'awaiting_processing'
                      ? 'Processing'
                      : 'Pending'
              }
            />
          </>
        )}
      </div>

      <div className="grid grid-cols-1 gap-4 xl:grid-cols-12">
        <div className="xl:col-span-5">
          {isLightsout ? (
            <EpisodeWeightHero
              weight={weight}
              weightStale={weightStale}
              runActive={runActive}
              phaseKind={phaseKind}
              scoopedG={scoopedDisplay}
              minScoopedG={
                typeof loSession?.min_scooped_g === 'number'
                  ? loSession.min_scooped_g
                  : null
              }
              netPouredG={baselines.netPouredG}
              targetWeightG={targetWeightG}
              targetToleranceG={targetToleranceG}
              targetPending={lightsout.targetPending}
              showVesselReadout
            />
          ) : (
            <WeightHero
              weight={
                // Prefer net poured when we have a pour baseline (correct by construction).
                baselines.pourBaselineG != null && baselines.netPouredG != null
                  ? baselines.netPouredG
                  : weight
              }
              targetWeightG={targetWeightG}
              targetToleranceG={targetToleranceG}
              weightStale={weightStale}
              webhookActive={runActive}
              bandCaption={
                typeof targetToleranceG === 'number'
                  ? `Target band ±${formatNumber(targetToleranceG, 1)} g`
                  : undefined
              }
            />
          )}
        </div>
        <div className="xl:col-span-7">
          <LiveWeightChart
            weight={weight}
            targetWeightG={isLightsout ? null : targetWeightG}
            targetToleranceG={isLightsout ? null : targetToleranceG}
            referenceWeightG={chartRefWeight}
            referenceToleranceG={chartRefTol}
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
            timelinePhases={profile.timelinePhases}
          />
        </div>
        <div className="xl:col-span-5">
          {isLightsout ? (
            <RunContextCard
              title="Session Context"
              loading={false}
              empty="No lights-out session active."
              badges={
                <>
                  <StatusBadge
                    label={statusLabel(runState)}
                    tone={
                      runState === 'succeeded'
                        ? 'good'
                        : runState === 'running' || runState === 'starting'
                          ? 'warn'
                          : runState === 'failed'
                            ? 'bad'
                            : 'neutral'
                    }
                    pulse={runState === 'running' || runState === 'starting'}
                  />
                  <StatusBadge label="MES N/A" tone="neutral" />
                </>
              }
              fields={loContextFields}
            />
          ) : (
            <RunContextCard
              title="Run Context"
              loading={loading}
              empty="No robot run recorded yet."
              badges={
                latestRun ? (
                  <>
                    <StatusBadge
                      label={statusLabel(latestRun.status)}
                      tone={
                        latestRun.status === 'succeeded'
                          ? 'good'
                          : latestRun.status === 'running' ||
                              latestRun.status === 'starting'
                            ? 'warn'
                            : latestRun.status === 'failed'
                              ? 'bad'
                              : 'neutral'
                      }
                      pulse={
                        latestRun.status === 'running' ||
                        latestRun.status === 'starting'
                      }
                    />
                    <StatusBadge
                      label={
                        !profile.showMesStatus
                          ? 'MES N/A'
                          : latestRun.mes_weighment_sent
                            ? 'MES sent'
                            : 'MES pending'
                      }
                      tone={
                        !profile.showMesStatus
                          ? 'neutral'
                          : latestRun.mes_weighment_sent
                            ? 'good'
                            : 'idle'
                      }
                    />
                  </>
                ) : undefined
              }
              fields={mesContextFields}
            />
          )}
        </div>

        {isLightsout ? (
          <>
            <div className="col-span-1 xl:col-span-12">
              <SessionProgressCard
                episode={lightsout.episode}
                episodesTotal={lightsout.episodesTotal}
                episodesCompleted={lightsout.sessionApi?.episodes.completed ?? 0}
                totalPouredG={lightsout.totalPouredG}
                scoopedMassG={lightsout.scoopedMassG}
                pourOutcome={lightsout.pourOutcome}
                stopReason={lightsout.stopReason}
                runState={runState}
                runActive={runActive}
                session={loSession ?? null}
                startedAt={lightsout.sessionApi?.started_at ?? null}
                toleranceFrac={lightsout.sessionApi?.tolerance_frac ?? 0.02}
              />
            </div>
            <div className="col-span-1 xl:col-span-12">
              <EpisodeStrip
                apiBase={apiBase}
                batchId={lightsout.batchId}
                activeEpisode={lightsout.episode}
              />
            </div>
          </>
        ) : (
          profile.showBatchQueue && (
            <div className="col-span-1 xl:col-span-12">
              <BatchQueueStrip
                items={queueItems}
                activeWeightmentId={latestRun?.weightment_id}
              />
            </div>
          )
        )}
      </div>
    </div>
  )
}

export default App
