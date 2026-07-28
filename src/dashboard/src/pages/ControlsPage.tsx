/**
 * Controls — Cell Signal Deck
 *
 * One place for link integrity (API → ROS → arm → scale → micro-ROS → Condor)
 * plus operator actions. Designed as a live signal spine rather than a card dump.
 */
import { useEffect, useMemo, useRef, useState, type ReactNode } from 'react'
import {
  Activity,
  Bot,
  Cable,
  Cloud,
  Radio,
  Scale,
  Server,
  Settings2,
  Sparkles,
  Waves,
} from 'lucide-react'
import Button from '../components/ui/button'
import StatusBadge, { type StatusTone } from '../components/ui/StatusBadge'
import { useRos } from '../ros/RosContext'
import { ROSLIB } from '../ros/roslib'
import { useRuntimeConfig } from '../config/RuntimeConfig'
import { useConnectionStatus, type ConnStatus } from '../hooks/useConnectionStatus'
import { useShellTelemetry, CLOCK_SKEW_WARN_S } from '../hooks/useShellTelemetry'
import { formatBytes, useSystemMetrics } from '../hooks/useSystemMetrics'

function MetricRow({
  label,
  value,
  pct,
}: {
  label: string
  value: string
  pct?: number | null
}) {
  const clamped =
    pct != null && Number.isFinite(pct) ? Math.max(0, Math.min(100, pct)) : null
  const barTone =
    clamped == null
      ? 'bg-[var(--border)]'
      : clamped >= 90
        ? 'bg-[var(--status-bad-fg)]'
        : clamped >= 75
          ? 'bg-[var(--status-warn-fg)]'
          : 'bg-[var(--accent)]'
  return (
    <div>
      <div className="mb-1 flex items-baseline justify-between gap-2 text-xs">
        <span className="text-[var(--text-muted)]">{label}</span>
        <span className="font-mono font-tabular text-[var(--text-secondary)]">{value}</span>
      </div>
      <div className="h-1.5 overflow-hidden rounded-full bg-[var(--surface-2)]">
        <div
          className={`h-full rounded-full transition-[width] duration-500 ${barTone}`}
          style={{ width: `${clamped ?? 0}%` }}
        />
      </div>
    </div>
  )
}

function MiniStat({
  label,
  value,
  warn,
}: {
  label: string
  value: string
  warn?: boolean
}) {
  return (
    <div className="rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2">
      <div className="text-[10px] uppercase tracking-wider text-[var(--text-faint)]">{label}</div>
      <div
        className={`mt-1 truncate font-mono text-[11px] font-tabular ${
          warn ? 'text-[var(--status-warn-fg)]' : 'text-[var(--text-secondary)]'
        }`}
        title={value}
      >
        {value}
      </div>
    </div>
  )
}

type SensorStatus = 'connected' | 'stale' | 'disconnected'
type LifecycleStatus = 'connected' | 'connecting' | 'disconnected'
type LinkStatus = 'connected' | 'connecting' | 'stale' | 'degraded' | 'disconnected' | 'unknown'

const WEIGHT_TOPIC: string = (import.meta as any).env.VITE_WEIGHT_TOPIC || '/weight'
const MICROROS_HEARTBEAT_TOPIC: string =
  (import.meta as any).env.VITE_MICROROS_HEARTBEAT_TOPIC || '/microros/heartbeat'

type IntegrationsPayload = {
  checked_at?: string
  condor?: {
    status?: string
    base_url?: string
    http?: { reachable?: boolean; latency_ms?: number | null; http_status?: number | null }
    websocket?: { status?: string; detail?: string | null; age_s?: number | null }
  }
  webhook?: {
    status?: string
    health_url?: string
    http?: { reachable?: boolean; latency_ms?: number | null }
  }
}

function toneFor(status: LinkStatus | ConnStatus | SensorStatus | LifecycleStatus): StatusTone {
  if (status === 'connected') return 'good'
  if (status === 'connecting' || status === 'stale' || status === 'degraded' || status === 'unknown') {
    return 'warn'
  }
  return 'bad'
}

function labelFor(status: LinkStatus | string): string {
  if (status === 'connected') return 'Live'
  if (status === 'connecting') return 'Linking'
  if (status === 'stale') return 'Stale'
  if (status === 'degraded') return 'Degraded'
  if (status === 'unknown') return 'Unknown'
  return 'Down'
}

function ago(ts: number | null, now: number): string {
  if (!ts) return 'never'
  return `${Math.max(0, Math.round((now - ts) / 1000))}s ago`
}

type SpineNode = {
  id: string
  title: string
  subtitle: string
  status: LinkStatus
  detail?: string
  icon: ReactNode
}

/** Flip to `'spine'` to restore the sequential link-integrity strip. */
const SIGNAL_LAYOUT: 'hub' | 'spine' = 'hub'

function orbClass(status: LinkStatus): string {
  const tone = toneFor(status)
  const live = status === 'connected'
  return `signal-orb relative z-[1] flex items-center justify-center rounded-2xl border ${
    live
      ? 'border-[var(--status-good-fg)]/40 bg-[var(--status-good-bg)] text-[var(--status-good-fg)]'
      : tone === 'warn'
        ? 'border-[var(--status-warn-fg)]/35 bg-[var(--status-warn-bg)] text-[var(--status-warn-fg)]'
        : 'border-[var(--status-bad-fg)]/35 bg-[var(--status-bad-bg)] text-[var(--status-bad-fg)]'
  } ${live ? 'signal-orb--pulse' : ''}`
}

function SignalNode({ node, index, total }: { node: SpineNode; index: number; total: number }) {
  const tone = toneFor(node.status)
  const live = node.status === 'connected'
  return (
    <div className="signal-node relative flex min-w-0 flex-1 flex-col items-center text-center">
      {index < total - 1 && (
        <div
          className={`signal-link absolute left-[calc(50%+22px)] right-[-50%] top-[22px] hidden h-[2px] md:block ${
            live ? 'signal-link--live' : 'signal-link--dead'
          }`}
          aria-hidden
        />
      )}
      <div className={`${orbClass(node.status)} h-11 w-11`}>{node.icon}</div>
      <div className="mt-3 w-full px-1">
        <div className="font-display text-sm font-semibold tracking-tight text-[var(--text-primary)]">
          {node.title}
        </div>
        <div className="mt-0.5 truncate text-[11px] text-[var(--text-faint)]" title={node.subtitle}>
          {node.subtitle}
        </div>
        <div className="mt-2 flex justify-center">
          <StatusBadge label={labelFor(node.status)} tone={tone} pulse={live} title={node.detail} />
        </div>
        {node.detail && (
          <p className="mt-2 line-clamp-2 text-[10px] leading-snug text-[var(--text-muted)]" title={node.detail}>
            {node.detail}
          </p>
        )}
      </div>
    </div>
  )
}

function HubSpoke({ node }: { node: SpineNode }) {
  const tone = toneFor(node.status)
  const live = node.status === 'connected'
  return (
    <div className="signal-hub-spoke flex flex-col items-center text-center">
      <div className={`${orbClass(node.status)} h-12 w-12`}>{node.icon}</div>
      <div className="mt-2 font-display text-sm font-semibold tracking-tight">{node.title}</div>
      <div className="mt-0.5 max-w-[9rem] truncate text-[11px] text-[var(--text-faint)]" title={node.subtitle}>
        {node.subtitle}
      </div>
      <div className="mt-2">
        <StatusBadge label={labelFor(node.status)} tone={tone} pulse={live} title={node.detail} />
      </div>
      {node.detail ? (
        <p className="mt-1.5 line-clamp-2 max-w-[10rem] text-[10px] leading-snug text-[var(--text-muted)]" title={node.detail}>
          {node.detail}
        </p>
      ) : null}
    </div>
  )
}

function SignalHub({
  hub,
  spokes,
}: {
  hub: SpineNode
  spokes: SpineNode[]
}) {
  const hubLive = hub.status === 'connected'
  // Expect 4 spokes: top-left, top-right, bottom-left, bottom-right around Pi.
  const [tl, tr, bl, br] = [
    spokes[0],
    spokes[1],
    spokes[2],
    spokes[3],
  ]
  return (
    <div className="signal-hub relative mx-auto w-full max-w-xl">
      <svg
        className="pointer-events-none absolute inset-[12%] hidden md:block"
        viewBox="0 0 100 100"
        preserveAspectRatio="none"
        aria-hidden
      >
        {[
          { n: tl, x: 18, y: 18 },
          { n: tr, x: 82, y: 18 },
          { n: bl, x: 18, y: 82 },
          { n: br, x: 82, y: 82 },
        ].map(({ n, x, y }) =>
          n ? (
            <line
              key={n.id}
              x1="50"
              y1="50"
              x2={x}
              y2={y}
              className={
                n.status === 'connected' && hubLive
                  ? 'signal-hub-ray--live'
                  : 'signal-hub-ray--dead'
              }
              strokeWidth="1.2"
            />
          ) : null,
        )}
      </svg>

      <div className="relative grid grid-cols-2 gap-x-6 gap-y-8 md:gap-y-12">
        {tl ? <HubSpoke node={tl} /> : <div />}
        {tr ? <HubSpoke node={tr} /> : <div />}
        <div className="col-span-2 flex flex-col items-center justify-center py-1">
          <div className={`${orbClass(hub.status)} h-16 w-16`}>{hub.icon}</div>
          <div className="mt-3 font-display text-base font-semibold tracking-tight">{hub.title}</div>
          <div className="mt-0.5 text-[11px] text-[var(--text-faint)]">{hub.subtitle}</div>
          <div className="mt-2">
            <StatusBadge
              label={labelFor(hub.status)}
              tone={toneFor(hub.status)}
              pulse={hubLive}
              title={hub.detail}
            />
          </div>
          <p className="mt-2 max-w-[16rem] text-center text-[10px] leading-snug text-[var(--text-muted)]">
            Hub view — peers hang off the Pi independently (not a sequence).
          </p>
        </div>
        {bl ? <HubSpoke node={bl} /> : <div />}
        {br ? <HubSpoke node={br} /> : <div />}
      </div>
    </div>
  )
}

export default function ControlsPage() {
  const ros = useRos()
  const { apiBase, rosbridgeUrl, setApiBase, setRosbridgeUrl } = useRuntimeConfig()
  const {
    weightStale,
    armStale,
    browserUnix,
    piUnix,
    niryoUnix,
    piSkew,
    niryoSkew,
    clockSkewWarn,
    formatClock,
    skewLabel,
  } = useShellTelemetry()
  const { pi, piError, niryo, hardwareTopic } = useSystemMetrics()
  const { apiStatus, rosStatus, hostName } = useConnectionStatus(weightStale)

  const recordSrvRef = useRef<any>(null)
  const [lastWeight, setLastWeight] = useState<number | null>(null)
  const [lastCamera, setLastCamera] = useState<number | null>(null)
  const [lastVibrationSent, setLastVibrationSent] = useState<number | null>(null)
  const [vibValue, setVibValue] = useState(0.3)
  const [microRosLifecycleStatus, setMicroRosLifecycleStatus] = useState<LifecycleStatus>('disconnected')
  const [microRosLifecycleLabel, setMicroRosLifecycleLabel] = useState('unknown')
  const [microRosPending, setMicroRosPending] = useState<'start' | 'stop' | null>(null)
  const [lastMicroRosHeartbeat, setLastMicroRosHeartbeat] = useState<number | null>(null)
  const [nowTs, setNowTs] = useState(Date.now())
  const [endpointsOpen, setEndpointsOpen] = useState(false)
  const [apiInput, setApiInput] = useState(apiBase)
  const [rosInput, setRosInput] = useState(rosbridgeUrl)
  const [integrations, setIntegrations] = useState<IntegrationsPayload | null>(null)
  const [integrationsError, setIntegrationsError] = useState<string | null>(null)

  const [recordName, setRecordName] = useState('')
  const [recordJoints, setRecordJoints] = useState(false)
  const [recordMsg, setRecordMsg] = useState('')

  useEffect(() => setApiInput(apiBase), [apiBase])
  useEffect(() => setRosInput(rosbridgeUrl), [rosbridgeUrl])

  const weightStatus: SensorStatus = lastWeight
    ? nowTs - lastWeight < 1500
      ? 'connected'
      : 'stale'
    : weightStale
      ? 'disconnected'
      : 'connected'
  const cameraStatus: SensorStatus = lastCamera
    ? nowTs - lastCamera < 3000
      ? 'connected'
      : 'stale'
    : 'disconnected'
  const microRosHeartbeatFresh = lastMicroRosHeartbeat ? nowTs - lastMicroRosHeartbeat < 3000 : false
  const microRosDisplayStatus: LifecycleStatus = microRosHeartbeatFresh
    ? 'connected'
    : microRosLifecycleStatus
  const armStatus: LinkStatus = armStale ? 'disconnected' : 'connected'
  const scaleStatus: LinkStatus =
    weightStatus === 'connected' ? 'connected' : weightStatus === 'stale' ? 'stale' : 'disconnected'
  const condorStatus = (integrations?.condor?.status || 'unknown') as LinkStatus
  const webhookStatus = (integrations?.webhook?.status || 'unknown') as LinkStatus

  useEffect(() => {
    if (!ros) return
    const weightSub = new ROSLIB.Topic({ ros, name: WEIGHT_TOPIC, messageType: 'std_msgs/Float64' })
    weightSub.subscribe(() => setLastWeight(Date.now()))
    const cameraSub = new ROSLIB.Topic({
      ros,
      name: '/scan_qr/camera_info',
      messageType: 'sensor_msgs/CameraInfo',
    })
    cameraSub.subscribe(() => setLastCamera(Date.now()))
    const microRosHeartbeatSub = new ROSLIB.Topic({
      ros,
      name: MICROROS_HEARTBEAT_TOPIC,
      messageType: 'std_msgs/Empty',
    })
    microRosHeartbeatSub.subscribe(() => setLastMicroRosHeartbeat(Date.now()))
    recordSrvRef.current = new ROSLIB.Service({
      ros,
      name: '/record_target',
      serviceType: 'robot_common_msgs/srv/RecordTarget',
    })
    return () => {
      weightSub.unsubscribe()
      cameraSub.unsubscribe()
      microRosHeartbeatSub.unsubscribe()
    }
  }, [ros])

  useEffect(() => {
    const interval = window.setInterval(() => setNowTs(Date.now()), 1000)
    return () => window.clearInterval(interval)
  }, [])

  useEffect(() => {
    let cancelled = false
    const load = async () => {
      try {
        const res = await fetch(`${apiBase}/integrations/status`, { cache: 'no-store' })
        if (!res.ok) throw new Error(`HTTP ${res.status}`)
        const data = (await res.json()) as IntegrationsPayload
        if (cancelled) return
        setIntegrations(data)
        setIntegrationsError(null)
      } catch (err) {
        if (cancelled) return
        setIntegrationsError(err instanceof Error ? err.message : 'probe failed')
      }
    }
    void load()
    const id = window.setInterval(load, 4000)
    return () => {
      cancelled = true
      window.clearInterval(id)
    }
  }, [apiBase])

  const changeMicroRosLifecycleState = async (transitionId: number) => {
    if (!ros) return
    const srv = new ROSLIB.Service({
      ros,
      name: '/micro_ros_launcher/change_state',
      serviceType: 'lifecycle_msgs/srv/ChangeState',
    })
    return new Promise<void>((resolve, reject) => {
      srv.callService(
        new ROSLIB.ServiceRequest({ transition: { id: transitionId } }),
        () => resolve(),
        (err: any) => reject(err),
      )
    })
  }

  const getMicroRosLifecycleState = async () => {
    if (!ros) return
    const srv = new ROSLIB.Service({
      ros,
      name: '/micro_ros_launcher/get_state',
      serviceType: 'lifecycle_msgs/srv/GetState',
    })
    return new Promise<{ label: string }>((resolve, reject) => {
      srv.callService(
        new ROSLIB.ServiceRequest({}),
        (res: any) => resolve({ label: res?.current_state?.label || 'unknown' }),
        (err: any) => reject(err),
      )
    })
  }

  const getRosNodes = async () => {
    if (!ros) return []
    const srv = new ROSLIB.Service({
      ros,
      name: '/rosapi/nodes',
      serviceType: 'rosapi/GetNodes',
    })
    return new Promise<string[]>((resolve, reject) => {
      srv.callService(
        new ROSLIB.ServiceRequest({}),
        (res: any) => resolve(Array.isArray(res?.nodes) ? res.nodes : []),
        (err: any) => reject(err),
      )
    })
  }

  const driveMicroRosLifecycle = async (labelOverride?: string) => {
    const label = labelOverride ?? microRosLifecycleLabel
    if (!microRosPending) return
    if (['configuring', 'activating', 'deactivating', 'cleaningup'].includes(label)) return
    try {
      if (microRosPending === 'start') {
        if (label === 'unconfigured' || label === 'unknown') {
          await changeMicroRosLifecycleState(1)
          return
        }
        if (label === 'inactive') {
          await changeMicroRosLifecycleState(3)
          return
        }
        if (label === 'active') setMicroRosPending(null)
        return
      }
      if (microRosPending === 'stop') {
        if (label === 'active') {
          await changeMicroRosLifecycleState(4)
          return
        }
        if (label === 'inactive') {
          await changeMicroRosLifecycleState(2)
          return
        }
        if (label === 'unconfigured' || label === 'unknown') setMicroRosPending(null)
      }
    } catch {
      setMicroRosPending(null)
      setMicroRosLifecycleStatus('disconnected')
    }
  }

  const refreshMicroRosLifecycle = async () => {
    try {
      const nodes = await getRosNodes()
      const hasLauncher = nodes.some((n) => n === '/micro_ros_launcher' || n === 'micro_ros_launcher')
      const hasAgent = nodes.some((n) => n === '/micro_ros_agent' || n === 'micro_ros_agent')
      if (hasLauncher) {
        const res = await getMicroRosLifecycleState()
        if (!res) return
        const label = res.label
        setMicroRosLifecycleLabel(label)
        if (label === 'active') setMicroRosLifecycleStatus('connected')
        else if (label === 'configuring' || label === 'activating') setMicroRosLifecycleStatus('connecting')
        else setMicroRosLifecycleStatus('disconnected')
        void driveMicroRosLifecycle(label)
        return
      }
      if (hasAgent) {
        setMicroRosLifecycleLabel('active')
        setMicroRosLifecycleStatus('connected')
        return
      }
      setMicroRosLifecycleLabel('unknown')
      setMicroRosLifecycleStatus('disconnected')
    } catch {
      try {
        const res = await getMicroRosLifecycleState()
        if (!res) return
        setMicroRosLifecycleLabel(res.label)
        setMicroRosLifecycleStatus(res.label === 'active' ? 'connected' : 'disconnected')
      } catch {
        setMicroRosLifecycleLabel('unknown')
        setMicroRosLifecycleStatus('disconnected')
      }
    }
  }

  useEffect(() => {
    if (!ros) return
    void refreshMicroRosLifecycle()
    const transitionSub = new ROSLIB.Topic({
      ros,
      name: '/micro_ros_launcher/transition_event',
      messageType: 'lifecycle_msgs/TransitionEvent',
    })
    transitionSub.subscribe(() => {
      void refreshMicroRosLifecycle()
    })
    const interval = window.setInterval(() => {
      void refreshMicroRosLifecycle()
    }, 2000)
    return () => {
      transitionSub.unsubscribe()
      window.clearInterval(interval)
    }
  }, [ros])

  const publishVibration = () => {
    if (!ros) return
    const pub = new ROSLIB.Topic({ ros, name: '/vibration/intensity', messageType: 'std_msgs/Float64' })
    pub.publish(new ROSLIB.Message({ data: vibValue }))
    setLastVibrationSent(Date.now())
  }

  const recordTarget = () => {
    if (!ros || !recordSrvRef.current || !recordName) return
    setRecordMsg('')
    recordSrvRef.current.callService(
      new ROSLIB.ServiceRequest({ name: recordName, joints: recordJoints }),
      (res: any) => setRecordMsg(res?.message || ''),
    )
  }

  const hubStatus: LinkStatus =
    apiStatus === 'disconnected' || rosStatus === 'disconnected'
      ? 'disconnected'
      : apiStatus === 'connecting' || rosStatus === 'connecting'
        ? 'connecting'
        : 'connected'

  const hub: SpineNode = useMemo(
    () => ({
      id: 'pi',
      title: 'Pi cell',
      subtitle: hostName || 'API + rosbridge',
      status: hubStatus,
      detail: `${apiBase} · ${rosbridgeUrl}`,
      icon: <Server className="h-7 w-7" />,
    }),
    [hubStatus, hostName, apiBase, rosbridgeUrl],
  )

  const spokes: SpineNode[] = useMemo(
    () => [
      {
        id: 'arm',
        title: 'Arm',
        subtitle: '/joint_states',
        status: armStatus,
        detail: armStale ? 'No fresh joint_states' : 'Joint stream live',
        icon: <Bot className="h-5 w-5" />,
      },
      {
        id: 'scale',
        title: 'Scale',
        subtitle: WEIGHT_TOPIC,
        status: scaleStatus,
        detail: `Last ${ago(lastWeight, nowTs)}`,
        icon: <Scale className="h-5 w-5" />,
      },
      {
        id: 'microros',
        title: 'micro-ROS',
        subtitle: microRosLifecycleLabel,
        status: microRosDisplayStatus,
        detail: `Heartbeat ${ago(lastMicroRosHeartbeat, nowTs)}`,
        icon: <Waves className="h-5 w-5" />,
      },
      {
        id: 'condor',
        title: 'Condor',
        subtitle: integrations?.condor?.base_url?.replace(/^https?:\/\//, '') || ':5002',
        status: condorStatus,
        detail:
          integrations?.condor?.websocket?.detail ||
          integrationsError ||
          (integrations?.condor?.http?.reachable
            ? `HTTP ${integrations.condor.http.latency_ms ?? '—'}ms`
            : 'Agent unreachable'),
        icon: <Cloud className="h-5 w-5" />,
      },
    ],
    [
      armStatus,
      armStale,
      scaleStatus,
      lastWeight,
      nowTs,
      microRosDisplayStatus,
      microRosLifecycleLabel,
      lastMicroRosHeartbeat,
      condorStatus,
      integrations,
      integrationsError,
    ],
  )

  // Kept for spine layout revert (`SIGNAL_LAYOUT = 'spine'`).
  const spine: SpineNode[] = useMemo(
    () => [
      {
        id: 'api',
        title: 'API',
        subtitle: hostName || 'backend',
        status: apiStatus,
        detail: apiBase,
        icon: <Cable className="h-5 w-5" />,
      },
      {
        id: 'ros',
        title: 'rosbridge',
        subtitle: 'DDS bridge',
        status: rosStatus,
        detail: rosbridgeUrl,
        icon: <Radio className="h-5 w-5" />,
      },
      ...spokes,
    ],
    [apiStatus, hostName, apiBase, rosStatus, rosbridgeUrl, spokes],
  )

  const peerNodes = SIGNAL_LAYOUT === 'hub' ? [hub, ...spokes] : spine
  const downCount = peerNodes.filter((n) => n.status === 'disconnected').length
  const warnCount = peerNodes.filter((n) =>
    ['stale', 'degraded', 'connecting', 'unknown'].includes(n.status),
  ).length
  const allLive = downCount === 0 && warnCount === 0 && !clockSkewWarn

  return (
    <div className="signal-deck px-5 py-5 lg:px-6">
      <div className="mb-6 flex flex-wrap items-end justify-between gap-4">
        <div>
          <div className="mb-1 inline-flex items-center gap-2 text-[11px] font-medium uppercase tracking-[0.18em] text-[var(--accent)]">
            <Sparkles className="h-3.5 w-3.5" />
            Cell signal deck
          </div>
          <h2 className="font-display text-2xl font-semibold tracking-tight text-[var(--text-primary)] md:text-3xl">
            Link integrity
          </h2>
          <p className="mt-1 max-w-2xl text-sm text-[var(--text-muted)]">
            {SIGNAL_LAYOUT === 'hub'
              ? 'Pi is the hub. Arm, scale, micro-ROS, and Condor are independent peers — not a pipeline.'
              : 'Live path from this browser through the Pi stack to the arm, sensors, and Condor MES bridge.'}{' '}
            Fix red nodes here before starting a weighment.
          </p>
        </div>
        <div className="flex flex-wrap items-center gap-2">
          <StatusBadge
            label={allLive ? 'Cell clear' : downCount ? `${downCount} down` : `${warnCount} attention`}
            tone={allLive ? 'good' : downCount ? 'bad' : 'warn'}
            pulse={allLive}
          />
          <StatusBadge
            label={clockSkewWarn ? 'Clock skew' : 'Clocks OK'}
            tone={clockSkewWarn ? 'warn' : 'good'}
            title={`Warn threshold ±${CLOCK_SKEW_WARN_S}s`}
          />
          <StatusBadge
            label={webhookStatus === 'connected' ? 'Webhook OK' : `Webhook ${labelFor(webhookStatus)}`}
            tone={toneFor(webhookStatus)}
            title={integrations?.webhook?.health_url}
          />
          <Button variant="ghost" size="sm" onClick={() => setEndpointsOpen((v) => !v)}>
            <Settings2 className="mr-1.5 h-3.5 w-3.5" />
            Endpoints
          </Button>
        </div>
      </div>

      {/* Signal map — hub (default) or spine (SIGNAL_LAYOUT) */}
      <section className="signal-spine relative overflow-hidden rounded-[var(--radius-lg)] border border-[var(--border)] bg-[var(--card-surface)] p-5 shadow-card md:p-6">
        <div className="pointer-events-none absolute inset-0 signal-spine-glow" aria-hidden />
        <div className="relative">
          {SIGNAL_LAYOUT === 'hub' ? (
            <SignalHub hub={hub} spokes={spokes} />
          ) : (
            <div className="flex flex-col gap-6 md:flex-row md:items-start md:justify-between">
              {spine.map((node, i) => (
                <SignalNode key={node.id} node={node} index={i} total={spine.length} />
              ))}
            </div>
          )}
        </div>

        {/* Condor callout */}
        <div className="relative mt-6 grid gap-3 border-t border-[var(--border)] pt-5 md:grid-cols-3">
          <div className="rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-4 py-3">
            <div className="text-[11px] uppercase tracking-wider text-[var(--text-faint)]">Condor agent</div>
            <div className="mt-1 font-display text-lg font-semibold">
              {labelFor(condorStatus)}
            </div>
            <p className="mt-1 text-xs text-[var(--text-muted)]">
              HTTP listener
              {integrations?.condor?.http?.latency_ms != null
                ? ` · ${integrations.condor.http.latency_ms}ms`
                : ''}
              {integrations?.condor?.http?.http_status != null
                ? ` · status ${integrations.condor.http.http_status}`
                : ''}
            </p>
            <p className="mt-1 text-xs text-[var(--text-secondary)]">
              Cloud WS:{' '}
              {integrations?.condor?.websocket?.status || '—'}
              {integrations?.condor?.websocket?.age_s != null
                ? ` · log ${integrations.condor.websocket.age_s}s old`
                : ''}
            </p>
          </div>
          <div className="rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-4 py-3 md:col-span-2">
            <div className="text-[11px] uppercase tracking-wider text-[var(--text-faint)]">Clock strip</div>
            <div className="mt-2 grid gap-2 font-mono text-xs sm:grid-cols-3">
              <div>
                <span className="text-[var(--text-faint)]">Browser </span>
                {formatClock(browserUnix)}
              </div>
              <div>
                <span className="text-[var(--text-faint)]">Pi </span>
                {formatClock(piUnix)}
                {piSkew != null ? (
                  <span className={Math.abs(piSkew) > CLOCK_SKEW_WARN_S ? ' text-[var(--status-warn-fg)]' : ''}>
                    {' '}
                    ({skewLabel(piSkew)})
                  </span>
                ) : null}
              </div>
              <div>
                <span className="text-[var(--text-faint)]">Niryo </span>
                {formatClock(niryoUnix)}
                {niryoSkew != null ? (
                  <span className={Math.abs(niryoSkew) > CLOCK_SKEW_WARN_S ? ' text-[var(--status-warn-fg)]' : ''}>
                    {' '}
                    ({skewLabel(niryoSkew)})
                  </span>
                ) : (
                  armStale && <span className=" text-[var(--status-bad-fg)]"> (no joint_states)</span>
                )}
              </div>
            </div>
          </div>
        </div>

        {endpointsOpen && (
          <div className="relative mt-4 grid gap-3 rounded-[var(--radius-sm)] border border-dashed border-[var(--border-strong)] bg-[var(--surface-2)] p-4 md:grid-cols-2">
            <label className="flex flex-col gap-1 text-xs text-[var(--text-muted)]">
              API base
              <input
                value={apiInput}
                onChange={(e) => setApiInput(e.target.value)}
                className="rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-1)] px-3 py-2 text-sm text-[var(--text-primary)]"
              />
            </label>
            <label className="flex flex-col gap-1 text-xs text-[var(--text-muted)]">
              rosbridge URL
              <input
                value={rosInput}
                onChange={(e) => setRosInput(e.target.value)}
                className="rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-1)] px-3 py-2 text-sm text-[var(--text-primary)]"
              />
            </label>
            <div className="md:col-span-2">
              <Button
                onClick={() => {
                  if (!apiInput.trim() || !rosInput.trim()) return
                  setApiBase(apiInput.trim())
                  setRosbridgeUrl(rosInput.trim())
                }}
              >
                Save & reconnect
              </Button>
              <span className="ml-3 text-xs text-[var(--text-faint)]">Stored in this browser only.</span>
            </div>
          </div>
        )}
      </section>

      {/* System metrics — Pi host + Niryo board/motors */}
      <section className="mt-6 grid gap-4 lg:grid-cols-2">
        <div className="rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--card-surface)] p-5 shadow-card">
          <div className="mb-4 flex items-center justify-between gap-2">
            <div>
              <h3 className="font-display text-base font-semibold">Pi system</h3>
              <p className="mt-0.5 text-xs text-[var(--text-muted)]">
                {hostName || 'API host'} · {pi?.source || 'awaiting metrics'}
              </p>
            </div>
            <StatusBadge
              label={piError ? 'Unavailable' : pi ? 'Live' : '…'}
              tone={piError ? 'bad' : pi ? 'good' : 'warn'}
            />
          </div>
          {piError ? (
            <p className="text-xs text-[var(--status-bad-fg)]">{piError}</p>
          ) : (
            <div className="space-y-3">
              <MetricRow
                label="CPU busy"
                value={
                  pi?.cpu_pct != null
                    ? `${pi.cpu_pct.toFixed(0)}%${
                        pi.cpu_cores != null ? ` of ${pi.cpu_cores} cores` : ''
                      }`
                    : '—'
                }
                pct={pi?.cpu_pct}
              />
              <MetricRow
                label="Memory"
                value={
                  pi?.mem_pct != null
                    ? `${pi.mem_pct.toFixed(0)}% · ${formatBytes(pi.mem_used_bytes)} / ${formatBytes(pi.mem_total_bytes)}`
                    : '—'
                }
                pct={pi?.mem_pct}
              />
              <MetricRow
                label={`Disk${pi?.disk_path ? ` (${pi.disk_path})` : ''}`}
                value={
                  pi?.disk_pct != null
                    ? `${pi.disk_pct.toFixed(0)}% · ${formatBytes(pi.disk_used_bytes)} / ${formatBytes(pi.disk_total_bytes)}`
                    : '—'
                }
                pct={pi?.disk_pct}
              />
              <div className="grid grid-cols-2 gap-3 pt-1 text-xs">
                <div className="rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2">
                  <div className="text-[10px] uppercase tracking-wider text-[var(--text-faint)]">
                    Load 1 / 5 / 15
                  </div>
                  <div className="mt-1 font-mono font-tabular text-[var(--text-secondary)]">
                    {pi?.load1 != null ? pi.load1.toFixed(2) : '—'}
                    {' / '}
                    {pi?.load5 != null ? pi.load5.toFixed(2) : '—'}
                    {' / '}
                    {pi?.load15 != null ? pi.load15.toFixed(2) : '—'}
                  </div>
                  {pi?.load_pressure_pct != null && pi.cpu_cores != null ? (
                    <div className="mt-1 text-[10px] text-[var(--text-faint)]">
                      ≈ {pi.load_pressure_pct.toFixed(0)}% queue pressure vs{' '}
                      {pi.cpu_cores} cores
                    </div>
                  ) : null}
                </div>
                <div className="rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2">
                  <div className="text-[10px] uppercase tracking-wider text-[var(--text-faint)]">
                    SoC temp
                  </div>
                  <div
                    className={`mt-1 font-mono font-tabular ${
                      pi?.temp_c != null && pi.temp_c >= 75
                        ? 'text-[var(--status-warn-fg)]'
                        : 'text-[var(--text-secondary)]'
                    }`}
                  >
                    {pi?.temp_c != null ? `${pi.temp_c.toFixed(1)} °C` : '—'}
                  </div>
                </div>
              </div>
            </div>
          )}
        </div>

        <div className="rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--card-surface)] p-5 shadow-card">
          <div className="mb-4 flex items-center justify-between gap-2">
            <div>
              <h3 className="font-display text-base font-semibold">Niryo system</h3>
              <p className="mt-0.5 text-xs text-[var(--text-muted)]">
                {niryo.hardware_version || 'Ned'} ·{' '}
                <code className="font-mono text-[10px]">{hardwareTopic}</code>
              </p>
            </div>
            <StatusBadge
              label={
                !ros
                  ? 'No ROS'
                  : niryo.fresh
                    ? 'Live'
                    : niryo.age_ms != null
                      ? 'Stale'
                      : 'No data'
              }
              tone={
                !ros || (!niryo.fresh && niryo.age_ms == null)
                  ? 'bad'
                  : niryo.fresh
                    ? 'good'
                    : 'warn'
              }
              pulse={niryo.fresh}
            />
          </div>
          <div className="mb-3 grid grid-cols-2 gap-3 text-xs sm:grid-cols-4">
            <MiniStat
              label="Board temp"
              value={
                niryo.rpi_temperature != null ? `${niryo.rpi_temperature} °C` : '—'
              }
              warn={
                Boolean(niryo.rpi_overheating) ||
                (niryo.rpi_temperature != null && niryo.rpi_temperature >= 70)
              }
            />
            <MiniStat
              label="Motors link"
              value={
                niryo.connection_up == null
                  ? '—'
                  : niryo.connection_up
                    ? 'Up'
                    : 'Down'
              }
              warn={niryo.connection_up === false}
            />
            <MiniStat
              label="Robot status"
              value={niryo.robot_status_str || '—'}
            />
            <MiniStat
              label="Calibration"
              value={
                niryo.calibration_in_progress
                  ? 'In progress'
                  : niryo.calibration_needed
                    ? 'Needed'
                    : niryo.calibration_needed === false
                      ? 'OK'
                      : '—'
              }
              warn={Boolean(niryo.calibration_needed)}
            />
          </div>
          {niryo.error_message ? (
            <p className="mb-3 text-xs text-[var(--status-warn-fg)]">{niryo.error_message}</p>
          ) : null}
          {niryo.robot_message ? (
            <p className="mb-3 text-xs text-[var(--text-muted)]">{niryo.robot_message}</p>
          ) : null}
          {niryo.motors.length === 0 ? (
            <p className="text-xs text-[var(--text-muted)]">
              Waiting for hardware_status (arm bridge + rosbridge must be up).
            </p>
          ) : (
            <div className="overflow-x-auto rounded-[var(--radius-sm)] border border-[var(--border)]">
              <table className="w-full min-w-[20rem] text-left text-xs">
                <thead className="bg-[var(--surface-2)] text-[10px] uppercase tracking-wider text-[var(--text-faint)]">
                  <tr>
                    <th className="px-3 py-2 font-medium">Motor</th>
                    <th className="px-3 py-2 font-medium">Temp</th>
                    <th className="px-3 py-2 font-medium">V</th>
                    <th className="px-3 py-2 font-medium">Err</th>
                  </tr>
                </thead>
                <tbody>
                  {niryo.motors.map((m) => (
                    <tr key={m.name} className="border-t border-[var(--border)]">
                      <td className="px-3 py-1.5 font-mono text-[var(--text-secondary)]">
                        {m.name}
                        {m.type ? (
                          <span className="text-[var(--text-faint)]"> · {m.type}</span>
                        ) : null}
                      </td>
                      <td
                        className={`px-3 py-1.5 font-mono font-tabular ${
                          m.temp_c != null && m.temp_c >= 55
                            ? 'text-[var(--status-warn-fg)]'
                            : ''
                        }`}
                      >
                        {m.temp_c != null ? `${m.temp_c}°` : '—'}
                      </td>
                      <td className="px-3 py-1.5 font-mono font-tabular">
                        {m.voltage != null ? m.voltage.toFixed(1) : '—'}
                      </td>
                      <td
                        className={`px-3 py-1.5 font-mono ${
                          m.error ? 'text-[var(--status-bad-fg)]' : 'text-[var(--text-faint)]'
                        }`}
                        title={m.error_message || undefined}
                      >
                        {m.error || '—'}
                      </td>
                    </tr>
                  ))}
                </tbody>
              </table>
            </div>
          )}
        </div>
      </section>

      {/* Actuator bay */}
      <div className="mt-6 grid grid-cols-1 gap-4 lg:grid-cols-3">
        <section className="rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--card-surface)] p-5 shadow-card lg:col-span-1">
          <div className="mb-4 flex items-center justify-between">
            <h3 className="font-display text-base font-semibold">Vibration</h3>
            <StatusBadge label={ros ? 'Ready' : 'Offline'} tone={ros ? 'info' : 'bad'} />
          </div>
          <div className="font-display text-4xl font-semibold font-tabular tracking-tight">
            {vibValue.toFixed(2)}
          </div>
          <input
            type="range"
            min={0}
            max={1}
            step={0.01}
            value={vibValue}
            onChange={(e) => setVibValue(parseFloat(e.target.value))}
            className="mt-4 w-full accent-[var(--accent)]"
          />
          <div className="mt-4 flex gap-2">
            <Button onClick={publishVibration} disabled={!ros}>
              Publish
            </Button>
            <Button variant="outline" onClick={() => setVibValue(0)} disabled={!ros}>
              Zero
            </Button>
          </div>
          <p className="mt-3 text-xs text-[var(--text-faint)]">
            Last send: {ago(lastVibrationSent, nowTs)}
          </p>
        </section>

        <section className="rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--card-surface)] p-5 shadow-card">
          <div className="mb-4 flex items-center justify-between">
            <h3 className="font-display text-base font-semibold">micro-ROS</h3>
            <StatusBadge
              label={microRosHeartbeatFresh ? 'Heartbeat' : microRosLifecycleLabel}
              tone={toneFor(microRosDisplayStatus)}
              pulse={microRosHeartbeatFresh}
            />
          </div>
          <p className="text-xs text-[var(--text-muted)]">
            Lifecycle node <code className="font-mono">/micro_ros_launcher</code>. Heartbeat topic{' '}
            <code className="font-mono">{MICROROS_HEARTBEAT_TOPIC}</code>.
          </p>
          <Button
            className="mt-4"
            disabled={!ros || !!microRosPending}
            onClick={async () => {
              if (!ros || microRosPending) return
              setMicroRosLifecycleStatus('connecting')
              try {
                const next = microRosLifecycleLabel === 'active' ? 'stop' : 'start'
                setMicroRosPending(next)
                await driveMicroRosLifecycle()
              } catch {
                setMicroRosPending(null)
                setMicroRosLifecycleStatus('disconnected')
              }
            }}
          >
            {microRosLifecycleLabel === 'active' ? 'Stop micro-ROS' : 'Start micro-ROS'}
          </Button>
        </section>

        <section className="rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--card-surface)] p-5 shadow-card">
          <div className="mb-4 flex items-center justify-between">
            <h3 className="font-display text-base font-semibold">Record target</h3>
            <StatusBadge label={ros ? 'Ready' : 'Offline'} tone={ros ? 'info' : 'bad'} />
          </div>
          <input
            placeholder="Target name"
            value={recordName}
            onChange={(e) => setRecordName(e.target.value)}
            className="w-full rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2 text-sm"
          />
          <label className="mt-3 flex items-center gap-2 text-sm text-[var(--text-secondary)]">
            <input
              type="checkbox"
              checked={recordJoints}
              onChange={(e) => setRecordJoints(e.target.checked)}
            />
            Record joints (not pose)
          </label>
          <Button className="mt-4" onClick={recordTarget} disabled={!ros || !recordName}>
            Record
          </Button>
          {recordMsg && <p className="mt-2 text-xs text-[var(--text-muted)]">{recordMsg}</p>}
        </section>
      </div>

      {/* Secondary sensors */}
      <div className="mt-4 grid grid-cols-1 gap-4 md:grid-cols-2">
        <section className="rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--card-surface-soft)] p-4">
          <div className="flex items-center justify-between gap-3">
            <div>
              <h3 className="font-display text-sm font-semibold">Camera</h3>
              <p className="text-xs text-[var(--text-faint)]">/scan_qr/camera_info · {ago(lastCamera, nowTs)}</p>
            </div>
            <StatusBadge label={labelFor(cameraStatus)} tone={toneFor(cameraStatus)} />
          </div>
        </section>
        <section className="rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--card-surface-soft)] p-4">
          <div className="flex items-center justify-between gap-3">
            <div>
              <h3 className="font-display text-sm font-semibold">Scale notes</h3>
              <p className="text-xs text-[var(--text-faint)]">
                Compose <code className="font-mono">scale_launcher</code> · not a ROS lifecycle node
              </p>
            </div>
            <Activity className="h-4 w-4 text-[var(--text-faint)]" />
          </div>
        </section>
      </div>
    </div>
  )
}
