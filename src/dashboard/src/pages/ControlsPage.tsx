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
// - A demo mode simulates connectivity without requiring ROS
import { useEffect, useMemo, useRef, useState } from 'react'
import SidebarLayout from './SidebarLayout'
import GlassCard from '../components/GlassCard'
import Button from '../components/ui/button'
import { useRos } from '../ros/RosContext'
import { ROSLIB } from '../ros/roslib'

type SensorStatus = 'connected' | 'stale' | 'disconnected'

// Configure rosbridge endpoint and topics through Vite env vars where possible
const ROSBRIDGE_URL: string = (import.meta as any).env.VITE_ROSBRIDGE_URL || 'ws://localhost:9090'
const WEIGHT_TOPIC: string = (import.meta as any).env.VITE_WEIGHT_TOPIC || '/weight'

export default function ControlsPage() {
  const [demo, setDemo] = useState<boolean>(false)
  const ros = useRos()
  const recordSrvRef = useRef<any>(null)
  useEffect(() => {
    // Initialize demo mode from localStorage so the toggle persists across reloads
    const stored = window.localStorage.getItem('demoMode') === '1'
    setDemo(stored)
  }, [])

  // last-seen timestamps
  const [lastWeight, setLastWeight] = useState<number | null>(null)
  const [lastCamera, setLastCamera] = useState<number | null>(null)
  const [lastVibrationSent, setLastVibrationSent] = useState<number | null>(null)
  const [vibValue, setVibValue] = useState<number>(30)

  // derived statuses
  const now = Date.now()
  const weightStatus: SensorStatus = lastWeight ? (now - lastWeight < 1500 ? 'connected' : 'stale') : 'disconnected'
  const cameraStatus: SensorStatus = lastCamera ? (now - lastCamera < 3000 ? 'connected' : 'stale') : 'disconnected'
  // Vibration is a publisher-only control; consider it "connected" if ROS is available
  const vibrationPubStatus: SensorStatus = ros ? 'connected' : 'disconnected'

  // ROS wiring: use shared ROS connection for topics/service/action
  useEffect(() => {
    if (demo) return
    if (!ros) return

    // Subscribe to a lightweight, high-frequency topic for each device.
    // We only care about "last seen" timestamps to compute a simple status badge.
    const weightSub = new ROSLIB.Topic({ ros, name: WEIGHT_TOPIC, messageType: 'std_msgs/Float64' })
    weightSub.subscribe(() => setLastWeight(Date.now()))

    const cameraSub = new ROSLIB.Topic({ ros, name: '/scan_qr/camera_info', messageType: 'sensor_msgs/CameraInfo' })
    cameraSub.subscribe(() => setLastCamera(Date.now()))

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
      
    }
  }, [demo, ros])

  // Demo simulation
  useEffect(() => {
    if (!demo) return
    const t = setInterval(() => {
      const d = Date.now()
      setLastWeight(d)
      setLastCamera(d)
    }, 1000)
    return () => clearInterval(t)
  }, [demo])

  // Publisher for vibration
  const publishVibration = () => {
    // Simple publisher example. In a real system you might wrap this in a debounced input.
    if (demo) {
      setLastVibrationSent(Date.now())
      return
    }
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
    if (demo || !ros || !recordSrvRef.current || !recordName) return
    setRecordMsg('')
    const req = new ROSLIB.ServiceRequest({ name: recordName, joints: recordJoints })
    recordSrvRef.current.callService(req, (res: any) => {
      setRecordMsg(res?.message || '')
    })
  }

  // MoveTo action removed in favor of service-only approach on this page

  const StatusBadge = ({ status }: { status: SensorStatus }) => (
    <span className={`text-xs px-2 py-0.5 rounded-md ${
      status === 'connected' ? 'bg-emerald-400/15 text-emerald-400' : status === 'stale' ? 'bg-amber-400/15 text-amber-400' : 'bg-rose-400/15 text-rose-400'
    }`}>{status}</span>
  )

  return (
    <SidebarLayout>
      <div className="px-6 py-6">
        <div className="flex items-end justify-between mb-4">
          <div>
            <h1 className="text-2xl font-bold" style={{ fontFamily: 'Space Grotesk' }}>Controls & Sensors</h1>
            <p className="text-white/70">Monitor sensor connectivity and control actuators</p>
          </div>
          <div className="flex items-center gap-2">
            <span className={`text-xs ${demo ? 'text-emerald-400' : 'text-white/60'}`}>{demo ? 'Demo' : 'Live'}</span>
            <Button onClick={() => { const v = !demo; setDemo(v); window.localStorage.setItem('demoMode', v ? '1' : '0') }}>{demo ? 'Disable Demo' : 'Enable Demo'}</Button>
          </div>
        </div>

        <div className="grid grid-cols-12 gap-4">
          {/* Weighing Scale */}
          <div className="col-span-12 md:col-span-4">
            <GlassCard>
              <div className="flex flex-col gap-2">
                <h3 className="text-lg font-semibold">Weighing Scale</h3>
                <div className="flex items-center gap-2"><span className="text-sm text-white/70">Status</span> <StatusBadge status={weightStatus} /></div>
                <div className="text-xs text-white/60">Topic: {WEIGHT_TOPIC}</div>
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
                  <Button onClick={publishVibration} disabled={!demo && !ros}>Publish</Button>
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


