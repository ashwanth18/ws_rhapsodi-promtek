// ControlsPage: live controls + sensor connectivity
// This page connects directly to ROS 2 through a WebSocket rosbridge (ros2-web-bridge)
// using roslibjs. We avoid going through the FastAPI backend for low-latency robot control.
//
// Key pieces implemented here:
// - Persistent rosbridge connection (ROSLIB.Ros) so we can subscribe/publish efficiently
// - Subscriptions to sensor topics to show connectivity (weight, camera, vibration)
// - Service client to record targets (pose or joints) via /record_target
//
// Design notes:
// - We keep a single ROS connection alive while the page is mounted
// - We use refs for the service/action clients to avoid re-creating them on each render
import { useEffect, useMemo, useRef, useState } from 'react'
import SidebarLayout from './SidebarLayout'
import Button from '../components/ui/button'
import { SubsystemCard } from '../components/ui/ConfirmDialog'
import { SectionHeader } from '../components/ui/SectionHeader'
import { useRos } from '../ros/RosContext'
import { ROSLIB } from '../ros/roslib'

type SensorStatus = 'connected' | 'stale' | 'disconnected'
type LifecycleStatus = 'connected' | 'connecting' | 'disconnected'

// Configure rosbridge endpoint and topics through Vite env vars where possible
const ROSBRIDGE_URL: string = (import.meta as any).env.VITE_ROSBRIDGE_URL || 'ws://localhost:9090'
const WEIGHT_TOPIC: string = (import.meta as any).env.VITE_WEIGHT_TOPIC || '/weight'
const MICROROS_HEARTBEAT_TOPIC: string = (import.meta as any).env.VITE_MICROROS_HEARTBEAT_TOPIC || '/microros/heartbeat'

export default function ControlsPage() {
  const ros = useRos()
  const recordSrvRef = useRef<any>(null)

  // last-seen timestamps
  const [lastWeight, setLastWeight] = useState<number | null>(null)
  const [lastCamera, setLastCamera] = useState<number | null>(null)
  const [lastVibrationSent, setLastVibrationSent] = useState<number | null>(null)
  const [vibValue, setVibValue] = useState<number>(0.3)
  const [scaleLifecycleStatus, setScaleLifecycleStatus] = useState<LifecycleStatus>('disconnected')
  const [scaleLifecycleLabel, setScaleLifecycleLabel] = useState<string>('unknown')
  const [scalePending, setScalePending] = useState<'start' | 'stop' | null>(null)
  const [microRosLifecycleStatus, setMicroRosLifecycleStatus] = useState<LifecycleStatus>('disconnected')
  const [microRosLifecycleLabel, setMicroRosLifecycleLabel] = useState<string>('unknown')
  const [microRosPending, setMicroRosPending] = useState<'start' | 'stop' | null>(null)
  const [lastMicroRosHeartbeat, setLastMicroRosHeartbeat] = useState<number | null>(null)
  const [nowTs, setNowTs] = useState<number>(Date.now())

  // derived statuses
  const weightStatus: SensorStatus = lastWeight ? (nowTs - lastWeight < 1500 ? 'connected' : 'stale') : 'disconnected'
  const cameraStatus: SensorStatus = lastCamera ? (nowTs - lastCamera < 3000 ? 'connected' : 'stale') : 'disconnected'
  // Vibration is a publisher-only control; consider it "connected" if ROS is available
  const vibrationPubStatus: SensorStatus = ros ? 'connected' : 'disconnected'
  const microRosHeartbeatFresh = lastMicroRosHeartbeat ? (nowTs - lastMicroRosHeartbeat < 3000) : false
  const microRosDisplayStatus: LifecycleStatus = microRosHeartbeatFresh ? 'connected' : microRosLifecycleStatus
  const microRosDisplayLabel: string = microRosHeartbeatFresh ? 'heartbeat' : microRosLifecycleLabel

  // ROS wiring: use shared ROS connection for topics/service/action
  useEffect(() => {
    if (!ros) return

    // Subscribe to a lightweight, high-frequency topic for each device.
    // We only care about "last seen" timestamps to compute a simple status badge.
    const weightSub = new ROSLIB.Topic({ ros, name: WEIGHT_TOPIC, messageType: 'std_msgs/Float64' })
    weightSub.subscribe(() => setLastWeight(Date.now()))

    const cameraSub = new ROSLIB.Topic({ ros, name: '/scan_qr/camera_info', messageType: 'sensor_msgs/CameraInfo' })
    cameraSub.subscribe(() => setLastCamera(Date.now()))

    const microRosHeartbeatSub = new ROSLIB.Topic({
      ros,
      name: MICROROS_HEARTBEAT_TOPIC,
      messageType: 'std_msgs/Empty',
    })
    microRosHeartbeatSub.subscribe(() => setLastMicroRosHeartbeat(Date.now()))

    // Set up service client once
    // - /record_target (robot_common_msgs/srv/RecordTarget): store a named pose/joints into YAML
    recordSrvRef.current = new ROSLIB.Service({
      ros,
      name: '/record_target',
      serviceType: 'robot_common_msgs/srv/RecordTarget',
    })

    return () => {
      // Clean-up subscriptions and close the websocket on unmount
      weightSub.unsubscribe()
      cameraSub.unsubscribe()
      microRosHeartbeatSub.unsubscribe()
      
    }
  }, [ros])

  useEffect(() => {
    const interval = window.setInterval(() => {
      setNowTs(Date.now())
    }, 1000)
    return () => {
      window.clearInterval(interval)
    }
  }, [])

  const getScaleLifecycleState = async () => {
    if (!ros) return
    const srv = new ROSLIB.Service({
      ros,
      name: '/weighing_scale_launcher/get_state',
      serviceType: 'lifecycle_msgs/srv/GetState',
    })
    return new Promise<{ label: string }>((resolve, reject) => {
      srv.callService(
        new ROSLIB.ServiceRequest({}),
        (res: any) => resolve({ label: res?.current_state?.label || 'unknown' }),
        (err: any) => reject(err)
      )
    })
  }

  const changeScaleLifecycleState = async (transitionId: number) => {
    if (!ros) return
    const srv = new ROSLIB.Service({
      ros,
      name: '/weighing_scale_launcher/change_state',
      serviceType: 'lifecycle_msgs/srv/ChangeState',
    })
    return new Promise<void>((resolve, reject) => {
      const req = new ROSLIB.ServiceRequest({ transition: { id: transitionId } })
      srv.callService(
        req,
        () => resolve(),
        (err: any) => reject(err)
      )
    })
  }

  const changeMicroRosLifecycleState = async (transitionId: number) => {
    if (!ros) return
    const srv = new ROSLIB.Service({
      ros,
      name: '/micro_ros_launcher/change_state',
      serviceType: 'lifecycle_msgs/srv/ChangeState',
    })
    return new Promise<void>((resolve, reject) => {
      const req = new ROSLIB.ServiceRequest({ transition: { id: transitionId } })
      srv.callService(
        req,
        () => resolve(),
        (err: any) => reject(err)
      )
    })
  }

  const refreshScaleLifecycle = async () => {
    try {
      const res = await getScaleLifecycleState()
      if (!res) return
      const label = res.label
      setScaleLifecycleLabel(label)
      if (label === 'active') {
        setScaleLifecycleStatus('connected')
      } else if (label === 'configuring' || label === 'activating') {
        setScaleLifecycleStatus('connecting')
      } else {
        setScaleLifecycleStatus('disconnected')
      }
      void driveScaleLifecycle(label)
    } catch {
      setScaleLifecycleLabel('unknown')
      setScaleLifecycleStatus('disconnected')
    }
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
        (err: any) => reject(err)
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
        (err: any) => reject(err)
      )
    })
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
        if (label === 'active') {
          setMicroRosLifecycleStatus('connected')
        } else if (label === 'configuring' || label === 'activating') {
          setMicroRosLifecycleStatus('connecting')
        } else {
          setMicroRosLifecycleStatus('disconnected')
        }
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
        const label = res.label
        setMicroRosLifecycleLabel(label)
        if (label === 'active') {
          setMicroRosLifecycleStatus('connected')
        } else if (label === 'configuring' || label === 'activating') {
          setMicroRosLifecycleStatus('connecting')
        } else {
          setMicroRosLifecycleStatus('disconnected')
        }
      } catch {
        setMicroRosLifecycleLabel('unknown')
        setMicroRosLifecycleStatus('disconnected')
      }
    }
  }

  const driveMicroRosLifecycle = async (labelOverride?: string) => {
    const label = labelOverride ?? microRosLifecycleLabel
    if (!microRosPending) return
    if (label === 'configuring' || label === 'activating' || label === 'deactivating' || label === 'cleaningup') {
      return
    }
    try {
      if (microRosPending === 'start') {
        if (label === 'unconfigured' || label === 'unknown') {
          await changeMicroRosLifecycleState(1) // configure
          return
        }
        if (label === 'inactive') {
          await changeMicroRosLifecycleState(3) // activate
          return
        }
        if (label === 'active') {
          setMicroRosPending(null)
        }
        return
      }
      if (microRosPending === 'stop') {
        if (label === 'active') {
          await changeMicroRosLifecycleState(4) // deactivate
          return
        }
        if (label === 'inactive') {
          await changeMicroRosLifecycleState(2) // cleanup
          return
        }
        if (label === 'unconfigured' || label === 'unknown') {
          setMicroRosPending(null)
        }
      }
    } catch {
      setMicroRosPending(null)
      setMicroRosLifecycleStatus('disconnected')
    }
  }

  const driveScaleLifecycle = async (labelOverride?: string) => {
    const label = labelOverride ?? scaleLifecycleLabel
    if (!scalePending) return
    if (label === 'configuring' || label === 'activating' || label === 'deactivating' || label === 'cleaningup') {
      return
    }
    try {
      if (scalePending === 'start') {
        if (label === 'unconfigured' || label === 'unknown') {
          await changeScaleLifecycleState(1) // configure
          return
        }
        if (label === 'inactive') {
          await changeScaleLifecycleState(3) // activate
          return
        }
        if (label === 'active') {
          setScalePending(null)
        }
        return
      }
      if (scalePending === 'stop') {
        if (label === 'active') {
          await changeScaleLifecycleState(4) // deactivate
          return
        }
        if (label === 'inactive') {
          await changeScaleLifecycleState(2) // cleanup
          return
        }
        if (label === 'unconfigured' || label === 'unknown') {
          setScalePending(null)
        }
      }
    } catch {
      setScalePending(null)
      setScaleLifecycleStatus('disconnected')
    }
  }

  useEffect(() => {
    if (!ros) return
    refreshScaleLifecycle()
    const transitionSub = new ROSLIB.Topic({
      ros,
      name: '/weighing_scale_launcher/transition_event',
      messageType: 'lifecycle_msgs/TransitionEvent',
    })
    transitionSub.subscribe(() => {
      refreshScaleLifecycle()
    })
    return () => {
      transitionSub.unsubscribe()
    }
  }, [ros])

  useEffect(() => {
    if (!ros) return
    refreshMicroRosLifecycle()
    const transitionSub = new ROSLIB.Topic({
      ros,
      name: '/micro_ros_launcher/transition_event',
      messageType: 'lifecycle_msgs/TransitionEvent',
    })
    transitionSub.subscribe(() => {
      refreshMicroRosLifecycle()
    })
    const interval = window.setInterval(() => {
      refreshMicroRosLifecycle()
    }, 2000)
    return () => {
      transitionSub.unsubscribe()
      window.clearInterval(interval)
    }
  }, [ros])

  // Publisher for vibration
  const publishVibration = () => {
    // Simple publisher example. In a real system you might wrap this in a debounced input.
    if (!ros) return
    const pub = new ROSLIB.Topic({ ros, name: '/vibration/intensity', messageType: 'std_msgs/Float64' })
    pub.publish(new ROSLIB.Message({ data: vibValue }))
    setLastVibrationSent(Date.now())
  }

  // Record target helpers
  const [recordName, setRecordName] = useState<string>('')
  const [recordJoints, setRecordJoints] = useState<boolean>(false)
  const [recordMsg, setRecordMsg] = useState<string>('')
  const recordTarget = async () => {
    // Calls the ROS service to record a target by name.
    // If joints=false, the server stores a Cartesian pose (base_link frame) for the current tool.
    if (!ros || !recordSrvRef.current || !recordName) return
    setRecordMsg('')
    const req = new ROSLIB.ServiceRequest({ name: recordName, joints: recordJoints })
    recordSrvRef.current.callService(req, (res: any) => {
      setRecordMsg(res?.message || '')
    })
  }

  // MoveTo action removed in favor of service-only approach on this page

  const SensorStatusPill = ({ status }: { status: SensorStatus | LifecycleStatus }) => {
    const classes = status === 'connected'
      ? 'bg-[var(--status-good-bg)] text-[var(--status-good-fg)]'
      : status === 'connecting' || status === 'stale'
        ? 'bg-[var(--status-warn-bg)] text-[var(--status-warn-fg)]'
        : 'bg-[var(--status-bad-bg)] text-[var(--status-bad-fg)]'
    return (
      <span className={`text-xs px-2 py-0.5 rounded-md ${classes}`}>{status}</span>
    )
  }

  return (
    <SidebarLayout>
      <div className="px-5 py-5 lg:px-6">
        <SectionHeader
          title="Controls & Sensors"
          description="Monitor subsystem connectivity and send low-latency ROS commands."
        />

        <div className="grid grid-cols-1 gap-4 md:grid-cols-2 xl:grid-cols-3">
          <SubsystemCard
            title="Weighing Scale"
            statusLabel={weightStatus}
            statusTone={weightStatus === 'connected' ? 'good' : weightStatus === 'stale' ? 'warn' : 'bad'}
            lastSeen={lastWeight ? `${Math.round((nowTs - lastWeight) / 1000)}s ago` : 'never'}
          >
            <div className="space-y-3 text-sm">
              <div className="flex items-center gap-2">
                <span className="text-[var(--text-muted)]">Lifecycle</span>
                <SensorStatusPill status={scaleLifecycleStatus} />
                <span className="text-xs text-[var(--text-faint)]">{scaleLifecycleLabel}</span>
              </div>
              <div className="text-xs text-[var(--text-muted)]">Topic: {WEIGHT_TOPIC}</div>
              <Button
                onClick={async () => {
                  if (!ros || scalePending) return
                  setScaleLifecycleStatus('connecting')
                  try {
                    const nextAction = scaleLifecycleLabel === 'active' ? 'stop' : 'start'
                    setScalePending(nextAction)
                    await driveScaleLifecycle()
                  } catch {
                    setScalePending(null)
                    setScaleLifecycleStatus('disconnected')
                  }
                }}
                disabled={!ros || !!scalePending}
              >
                {scaleLifecycleLabel === 'active' ? 'Stop scale' : 'Start scale'}
              </Button>
            </div>
          </SubsystemCard>

          <SubsystemCard
            title="micro-ROS"
            statusLabel={microRosDisplayLabel}
            statusTone={microRosDisplayStatus === 'connected' ? 'good' : microRosDisplayStatus === 'connecting' ? 'warn' : 'bad'}
            lastSeen={
              lastMicroRosHeartbeat
                ? `${Math.round((nowTs - lastMicroRosHeartbeat) / 1000)}s ago`
                : 'never'
            }
          >
            <div className="space-y-3 text-sm">
              <div className="text-xs text-[var(--text-muted)]">Node: /micro_ros_launcher</div>
              <Button
                onClick={async () => {
                  if (!ros || microRosPending) return
                  setMicroRosLifecycleStatus('connecting')
                  try {
                    const nextAction = microRosLifecycleLabel === 'active' ? 'stop' : 'start'
                    setMicroRosPending(nextAction)
                    await driveMicroRosLifecycle()
                  } catch {
                    setMicroRosPending(null)
                    setMicroRosLifecycleStatus('disconnected')
                  }
                }}
                disabled={!ros || !!microRosPending}
              >
                {microRosLifecycleLabel === 'active' ? 'Stop micro-ROS' : 'Start micro-ROS'}
              </Button>
            </div>
          </SubsystemCard>

          <SubsystemCard
            title="Camera"
            statusLabel={cameraStatus}
            statusTone={cameraStatus === 'connected' ? 'good' : cameraStatus === 'stale' ? 'warn' : 'bad'}
            lastSeen={lastCamera ? `${Math.round((nowTs - lastCamera) / 1000)}s ago` : 'never'}
          >
            <div className="text-xs text-[var(--text-muted)]">Topic: /scan_qr/camera_info</div>
          </SubsystemCard>

          <SubsystemCard
            title="Vibration"
            statusLabel={vibrationPubStatus}
            statusTone={vibrationPubStatus === 'connected' ? 'good' : 'bad'}
            lastSeen={
              lastVibrationSent
                ? `${Math.round((nowTs - lastVibrationSent) / 1000)}s ago`
                : 'never'
            }
          >
            <div className="space-y-3">
              <div className="font-display text-3xl font-semibold font-tabular">
                {vibValue.toFixed(2)}
              </div>
              <input
                type="range"
                min={0}
                max={1}
                step={0.01}
                value={vibValue}
                onChange={(e) => setVibValue(parseFloat(e.target.value))}
                className="w-full"
              />
              <div className="flex gap-2">
                <Button onClick={publishVibration} disabled={!ros}>
                  Publish
                </Button>
                <Button variant="outline" onClick={() => setVibValue(0)} disabled={!ros}>
                  Stop
                </Button>
              </div>
            </div>
          </SubsystemCard>

          <SubsystemCard title="Record Target" statusLabel={ros ? 'ready' : 'offline'} statusTone={ros ? 'info' : 'bad'}>
            <div className="space-y-3">
              <input
                placeholder="Target name"
                value={recordName}
                onChange={(e) => setRecordName(e.target.value)}
                className="w-full rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2 text-sm"
              />
              <label className="flex items-center gap-2 text-sm text-[var(--text-secondary)]">
                <input
                  type="checkbox"
                  checked={recordJoints}
                  onChange={(e) => setRecordJoints(e.target.checked)}
                />
                Record joints (not pose)
              </label>
              <Button onClick={recordTarget} disabled={!ros || !recordName}>
                Record
              </Button>
              {recordMsg && <div className="text-xs text-[var(--text-muted)]">{recordMsg}</div>}
            </div>
          </SubsystemCard>
        </div>
      </div>
    </SidebarLayout>
  )
}


