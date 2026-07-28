import { useEffect, useState } from 'react'
import { useRos } from '../ros/RosContext'
import { ROSLIB } from '../ros/roslib'
import { useRuntimeConfig } from '../config/RuntimeConfig'

const HARDWARE_STATUS_TOPIC =
  (import.meta as any).env.VITE_NIRYO_HARDWARE_STATUS_TOPIC ||
  '/niryo_robot_hardware_interface/hardware_status'
const ROBOT_STATUS_TOPIC =
  (import.meta as any).env.VITE_NIRYO_ROBOT_STATUS_TOPIC ||
  '/niryo_robot_status/robot_status'

export type HostMetrics = {
  source?: string
  cpu_pct?: number | null
  load1?: number | null
  load5?: number | null
  load15?: number | null
  cpu_cores?: number | null
  load_pressure_pct?: number | null
  mem_pct?: number | null
  mem_used_bytes?: number | null
  mem_total_bytes?: number | null
  disk_pct?: number | null
  disk_used_bytes?: number | null
  disk_total_bytes?: number | null
  disk_path?: string | null
  temp_c?: number | null
}

export type NiryoMotorMetric = {
  name: string
  type?: string
  temp_c: number | null
  voltage: number | null
  error?: number | null
  error_message?: string | null
}

export type NiryoMetrics = {
  fresh: boolean
  age_ms: number | null
  rpi_temperature: number | null
  hardware_version: string | null
  hardware_state: number | null
  connection_up: boolean | null
  calibration_needed: boolean | null
  calibration_in_progress: boolean | null
  error_message: string | null
  motors: NiryoMotorMetric[]
  robot_status_str: string | null
  robot_message: string | null
  rpi_overheating: boolean | null
}

const HW_STALE_MS = 4000

const EMPTY_NIRYO: NiryoMetrics = {
  fresh: false,
  age_ms: null,
  rpi_temperature: null,
  hardware_version: null,
  hardware_state: null,
  connection_up: null,
  calibration_needed: null,
  calibration_in_progress: null,
  error_message: null,
  motors: [],
  robot_status_str: null,
  robot_message: null,
  rpi_overheating: null,
}

export function formatBytes(n?: number | null): string {
  if (n == null || !Number.isFinite(n)) return '—'
  const units = ['B', 'KB', 'MB', 'GB', 'TB']
  let v = n
  let i = 0
  while (v >= 1024 && i < units.length - 1) {
    v /= 1024
    i += 1
  }
  return `${v.toFixed(v >= 10 || i === 0 ? 0 : 1)} ${units[i]}`
}

export function useSystemMetrics() {
  const ros = useRos()
  const { apiBase } = useRuntimeConfig()
  const [pi, setPi] = useState<HostMetrics | null>(null)
  const [piError, setPiError] = useState<string | null>(null)
  const [niryo, setNiryo] = useState<NiryoMetrics>(EMPTY_NIRYO)
  const [lastHwTs, setLastHwTs] = useState<number | null>(null)

  useEffect(() => {
    let cancelled = false
    const poll = async () => {
      try {
        const res = await fetch(`${apiBase}/host_info`, { cache: 'no-store' })
        if (!res.ok) throw new Error(`host_info ${res.status}`)
        const data = await res.json()
        if (cancelled) return
        setPi((data?.metrics as HostMetrics) || null)
        setPiError(null)
      } catch (err) {
        if (!cancelled) {
          setPiError(err instanceof Error ? err.message : String(err))
        }
      }
    }
    void poll()
    const id = window.setInterval(poll, 2500)
    return () => {
      cancelled = true
      window.clearInterval(id)
    }
  }, [apiBase])

  useEffect(() => {
    if (!ros) {
      setNiryo(EMPTY_NIRYO)
      setLastHwTs(null)
      return
    }
    const hw = new ROSLIB.Topic({
      ros,
      name: HARDWARE_STATUS_TOPIC,
      messageType: 'niryo_ned_ros2_interfaces/HardwareStatus',
    })
    hw.subscribe((msg: Record<string, unknown>) => {
      const names = (msg.motor_names as string[]) || []
      const types = (msg.motor_types as string[]) || []
      const temps = (msg.temperatures as number[]) || []
      const volts = (msg.voltages as number[]) || []
      const errors = (msg.hardware_errors as number[]) || []
      const errMsgs = (msg.hardware_errors_message as string[]) || []
      const motors: NiryoMotorMetric[] = names.map((name, i) => ({
        name,
        type: types[i],
        temp_c: typeof temps[i] === 'number' ? temps[i] : null,
        voltage: typeof volts[i] === 'number' ? volts[i] : null,
        error: typeof errors[i] === 'number' ? errors[i] : null,
        error_message: errMsgs[i] || null,
      }))
      setLastHwTs(Date.now())
      setNiryo((prev) => ({
        ...prev,
        fresh: true,
        age_ms: 0,
        rpi_temperature:
          typeof msg.rpi_temperature === 'number' ? msg.rpi_temperature : null,
        hardware_version:
          typeof msg.hardware_version === 'string' ? msg.hardware_version : null,
        hardware_state:
          typeof msg.hardware_state === 'number' ? msg.hardware_state : null,
        connection_up:
          typeof msg.connection_up === 'boolean' ? msg.connection_up : null,
        calibration_needed:
          typeof msg.calibration_needed === 'boolean'
            ? msg.calibration_needed
            : null,
        calibration_in_progress:
          typeof msg.calibration_in_progress === 'boolean'
            ? msg.calibration_in_progress
            : null,
        error_message:
          typeof msg.error_message === 'string' ? msg.error_message : null,
        motors,
      }))
    })

    const status = new ROSLIB.Topic({
      ros,
      name: ROBOT_STATUS_TOPIC,
      messageType: 'niryo_ned_ros2_interfaces/RobotStatus',
    })
    status.subscribe((msg: Record<string, unknown>) => {
      setNiryo((prev) => ({
        ...prev,
        robot_status_str:
          typeof msg.robot_status_str === 'string' ? msg.robot_status_str : null,
        robot_message:
          typeof msg.robot_message === 'string' ? msg.robot_message : null,
        rpi_overheating:
          typeof msg.rpi_overheating === 'boolean' ? msg.rpi_overheating : null,
      }))
    })

    return () => {
      hw.unsubscribe()
      status.unsubscribe()
    }
  }, [ros])

  useEffect(() => {
    const id = window.setInterval(() => {
      const now = Date.now()
      setNiryo((prev) => {
        if (!lastHwTs) {
          return { ...prev, fresh: false, age_ms: null }
        }
        const age = now - lastHwTs
        return { ...prev, fresh: age <= HW_STALE_MS, age_ms: age }
      })
    }, 500)
    return () => window.clearInterval(id)
  }, [lastHwTs])

  return { pi, piError, niryo, hardwareTopic: HARDWARE_STATUS_TOPIC }
}
