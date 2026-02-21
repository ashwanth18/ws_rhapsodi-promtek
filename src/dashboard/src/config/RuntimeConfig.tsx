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

const RuntimeConfigCtx = createContext<RuntimeConfig | null>(null)

export function RuntimeConfigProvider({ children }: { children: React.ReactNode }) {
  const [apiBase, setApiBaseState] = useState<string>(
    () => localStorage.getItem(API_KEY) || API_DEFAULT
  )
  const [rosbridgeUrl, setRosbridgeUrlState] = useState<string>(
    () => localStorage.getItem(ROS_KEY) || ROS_DEFAULT
  )

  const setApiBase = (value: string) => {
    const next = value.trim()
    setApiBaseState(next)
    localStorage.setItem(API_KEY, next)
  }

  const setRosbridgeUrl = (value: string) => {
    const next = value.trim()
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
