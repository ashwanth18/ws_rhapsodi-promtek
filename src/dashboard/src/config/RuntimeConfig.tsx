import { createContext, useContext, useMemo, useState } from 'react'

type RuntimeConfig = {
  apiBase: string
  rosbridgeUrl: string
  setApiBase: (value: string) => void
  setRosbridgeUrl: (value: string) => void
}

const API_DEFAULT = (import.meta as any).env.VITE_API_BASE || 'http://localhost:8000'
const ROS_DEFAULT = (import.meta as any).env.VITE_ROSBRIDGE_URL || 'ws://localhost:9090'
const API_KEY = 'rhapsodi.apiBase'
const ROS_KEY = 'rhapsodi.rosbridgeUrl'

/** roslib WebSockets require ws: or wss: — http(s):// here throws DOMException in the browser. */
export function normalizeRosbridgeUrl(raw: string): string {
  let s = raw.trim()
  if (!s) return s
  if (s.startsWith('http://')) s = `ws://${s.slice('http://'.length)}`
  else if (s.startsWith('https://')) s = `wss://${s.slice('https://'.length)}`
  return s
}

function initialApiBase(): string {
  const stored = localStorage.getItem(API_KEY)?.trim()
  return stored || API_DEFAULT.trim()
}

function initialRosbridgeUrl(): string {
  const stored = localStorage.getItem(ROS_KEY)?.trim()
  if (!stored) return normalizeRosbridgeUrl(ROS_DEFAULT) || ROS_DEFAULT
  const n = normalizeRosbridgeUrl(stored)
  return n || ROS_DEFAULT
}

const RuntimeConfigCtx = createContext<RuntimeConfig | null>(null)

export function RuntimeConfigProvider({ children }: { children: React.ReactNode }) {
  const [apiBase, setApiBaseState] = useState<string>(initialApiBase)
  const [rosbridgeUrl, setRosbridgeUrlState] = useState<string>(initialRosbridgeUrl)

  const setApiBase = (value: string) => {
    const next = value.trim()
    setApiBaseState(next)
    localStorage.setItem(API_KEY, next)
  }

  const setRosbridgeUrl = (value: string) => {
    const next = normalizeRosbridgeUrl(value)
    setRosbridgeUrlState(next)
    localStorage.setItem(ROS_KEY, next)
  }

  const value = useMemo(
    () => ({ apiBase, rosbridgeUrl, setApiBase, setRosbridgeUrl }),
    [apiBase, rosbridgeUrl]
  )

  return (
    <RuntimeConfigCtx.Provider value={value}>
      {children}
    </RuntimeConfigCtx.Provider>
  )
}

export function useRuntimeConfig(): RuntimeConfig {
  const ctx = useContext(RuntimeConfigCtx)
  if (!ctx) {
    throw new Error('useRuntimeConfig must be used within RuntimeConfigProvider')
  }
  return ctx
}
