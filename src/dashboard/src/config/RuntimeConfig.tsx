import { createContext, useContext, useMemo, useState } from 'react'

type RuntimeConfig = {
  apiBase: string
  rosbridgeUrl: string
  setApiBase: (value: string) => void
  setRosbridgeUrl: (value: string) => void
}

const ENV_API = ((import.meta as any).env.VITE_API_BASE as string | undefined)?.trim() || ''
const ENV_ROS = ((import.meta as any).env.VITE_ROSBRIDGE_URL as string | undefined)?.trim() || ''
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

function isLoopbackHost(host: string): boolean {
  const h = host.trim().toLowerCase()
  return h === 'localhost' || h === '127.0.0.1' || h === '[::1]' || h === '::1'
}

function isLoopbackUrl(raw: string): boolean {
  try {
    const u = new URL(raw.includes('://') ? raw : `http://${raw}`)
    return isLoopbackHost(u.hostname)
  } catch {
    return false
  }
}

/** Derive API / rosbridge from the page host so remote Tailscale access works. */
export function pageHostDefaults(): { apiBase: string; rosbridgeUrl: string } {
  if (typeof window === 'undefined') {
    return { apiBase: 'http://localhost:8000', rosbridgeUrl: 'ws://localhost:9090' }
  }
  const host = window.location.hostname || 'localhost'
  const http = window.location.protocol === 'https:' ? 'https' : 'http'
  const ws = http === 'https' ? 'wss' : 'ws'
  return {
    apiBase: `${http}://${host}:8000`,
    rosbridgeUrl: `${ws}://${host}:9090`,
  }
}

function resolveUrl(
  stored: string | null | undefined,
  envDefault: string,
  pageDefault: string,
): string {
  const s = stored?.trim()
  if (s) {
    // Stale localhost from a prior session breaks remote MagicDNS access.
    if (isLoopbackUrl(s) && typeof window !== 'undefined' && !isLoopbackHost(window.location.hostname)) {
      return pageDefault
    }
    return s
  }
  const env = envDefault.trim()
  if (
    (!env || isLoopbackUrl(env)) &&
    typeof window !== 'undefined' &&
    !isLoopbackHost(window.location.hostname)
  ) {
    return pageDefault
  }
  return env || pageDefault
}

function initialApiBase(): string {
  return resolveUrl(localStorage.getItem(API_KEY), ENV_API, pageHostDefaults().apiBase)
}

function initialRosbridgeUrl(): string {
  const page = pageHostDefaults().rosbridgeUrl
  const resolved = resolveUrl(localStorage.getItem(ROS_KEY), ENV_ROS, page)
  return normalizeRosbridgeUrl(resolved) || page
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
    [apiBase, rosbridgeUrl],
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
