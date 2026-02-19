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
import GlassCard from '../components/GlassCard'
import Button from '../components/ui/button'
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
  const [vibValue, setVibValue] = useState<number>(30)
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
    const pub = new ROSLIB.Topic({ ros, name: '/motor_speed', messageType: 'std_msgs/Int32' })
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

  const StatusBadge = ({ status }: { status: SensorStatus | LifecycleStatus }) => {
    const classes = status === 'connected'
      ? 'bg-emerald-400/15 text-emerald-400'
      : status === 'connecting' || status === 'stale'
        ? 'bg-amber-400/15 text-amber-400'
        : 'bg-rose-400/15 text-rose-400'
    return (
      <span className={`text-xs px-2 py-0.5 rounded-md ${classes}`}>{status}</span>
    )
  }

  return (
    <SidebarLayout>
      <div className="px-6 py-6">
        <div className="flex items-end justify-between mb-4">
          <div>
            <h1 className="text-2xl font-bold" style={{ fontFamily: 'Space Grotesk' }}>Controls & Sensors</h1>
            <p className="text-white/70">Monitor sensor connectivity and control actuators</p>
          </div>
        </div>

        <div className="grid grid-cols-12 gap-4">
          {/* Weighing Scale */}
          <div className="col-span-12 md:col-span-4">
            <GlassCard>
              <div className="flex flex-col gap-2">
                <h3 className="text-lg font-semibold">Weighing Scale</h3>
                <div className="flex items-center gap-2">
                  <span className="text-sm text-white/70">Data</span>
                  <StatusBadge status={weightStatus} />
                </div>
                <div className="flex items-center gap-2">
                  <span className="text-sm text-white/70">Lifecycle</span>
                  <StatusBadge status={scaleLifecycleStatus} />
                  <span className="text-xs text-white/50">{scaleLifecycleLabel}</span>
                </div>
                <div className="text-xs text-white/60">Topic: {WEIGHT_TOPIC}</div>
                <div className="flex items-center gap-2 mt-2">
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
                    {scaleLifecycleLabel === 'active' ? 'Stop' : 'Start'}
                  </Button>
                </div>
              </div>
            </GlassCard>
          </div>

          {/* micro-ROS */}
          <div className="col-span-12 md:col-span-4">
            <GlassCard>
              <div className="flex flex-col gap-2">
                <h3 className="text-lg font-semibold">micro-ROS</h3>
                <div className="flex items-center gap-2">
                  <span className="text-sm text-white/70">Lifecycle</span>
                  <StatusBadge status={microRosDisplayStatus} />
                  <span className="text-xs text-white/50">{microRosDisplayLabel}</span>
                </div>
                <div className="text-xs text-white/60">Node: /micro_ros_launcher</div>
                <div className="flex items-center gap-2 mt-2">
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
                    {microRosLifecycleLabel === 'active' ? 'Stop' : 'Start'}
                  </Button>
                </div>
              </div>
            </GlassCard>
          </div>

          {/* Record target (pose/joints) */}
          <div className="col-span-12 md:col-span-6">
            <GlassCard>
              <div className="flex flex-col gap-3">
                <h3 className="text-lg font-semibold">Record Target</h3>
                <div className="flex items-center gap-3">
                  <input placeholder="name" value={recordName} onChange={e => setRecordName(e.target.value)} className="px-2 py-1 bg-transparent border border-slate-800 rounded" />
                  <label className="flex items-center gap-2 text-sm text-white/80">
                    <input type="checkbox" checked={recordJoints} onChange={e => setRecordJoints(e.target.checked)} /> Joints
                  </label>
                  <Button onClick={recordTarget} disabled={!ros || !recordName}>Record</Button>
                </div>
                {recordMsg && <div className="text-xs text-white/60">{recordMsg}</div>}
              </div>
            </GlassCard>
          </div>

          {/* Move To controls removed (actions not used via rosbridge) */}
          {/* Camera */}
          <div className="col-span-12 md:col-span-4">
            <GlassCard>
              <div className="flex flex-col gap-2">
                <h3 className="text-lg font-semibold">Camera</h3>
                <div className="flex items-center gap-2"><span className="text-sm text-white/70">Status</span> <StatusBadge status={cameraStatus} /></div>
                <div className="text-xs text-white/60">Topic: /camera_info</div>
              </div>
            </GlassCard>
          </div>

          {/* Vibration Control (publisher only) */}
          <div className="col-span-12 md:col-span-4">
            <GlassCard>
              <div className="flex flex-col gap-3">
                <h3 className="text-lg font-semibold">Vibration</h3>
                <div className="flex items-center gap-2"><span className="text-sm text-white/70">Status</span> <StatusBadge status={vibrationPubStatus} /></div>
                <div className="text-xs text-white/60">Publisher → /motor_speed (std_msgs/Int32)</div>
                <div className="flex items-center gap-3">
                  <input type="range" min={0} max={255} value={vibValue} onChange={(e) => setVibValue(parseInt(e.target.value))} className="w-full" />
                  <span className="w-10 text-right text-sm">{vibValue}</span>
                  <Button onClick={publishVibration} disabled={!ros}>Publish</Button>
                </div>
                {lastVibrationSent && (
                  <div className="text-xs text-white/60">Last sent: {Math.round((Date.now() - lastVibrationSent)/1000)}s ago</div>
                )}
              </div>
            </GlassCard>
          </div>
        </div>
      </div>
    </SidebarLayout>
  )
}


