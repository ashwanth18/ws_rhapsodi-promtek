import { useCallback, useEffect, useState } from 'react'

export type RuntimeMode = {
  mode: string
  environment: string
  active_run?: {
    id: number
    status: string
    weightment_id: number | null
    event_id: string | null
    batch_id: string | null
    kind?: string
  } | null
}

export type CapabilityMode = {
  mode: string
  label?: string
  description?: string
  allowed_environments?: string[]
}

export type RuntimeCapabilities = {
  modes: CapabilityMode[]
  sim_allowed?: boolean
  default_mode?: string
}

export function useRuntimeMode(apiBase: string, pollMs = 5000) {
  const [runtime, setRuntime] = useState<RuntimeMode | null>(null)
  const [capabilities, setCapabilities] = useState<RuntimeCapabilities | null>(null)
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)

  const refresh = useCallback(async () => {
    try {
      const [modeRes, capRes] = await Promise.all([
        fetch(`${apiBase}/runtime/mode`),
        fetch(`${apiBase}/runtime/capabilities`),
      ])
      if (!modeRes.ok) throw new Error('Failed to load runtime mode')
      if (!capRes.ok) throw new Error('Failed to load capabilities')
      setRuntime((await modeRes.json()) as RuntimeMode)
      setCapabilities((await capRes.json()) as RuntimeCapabilities)
      setError(null)
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to load runtime')
    } finally {
      setLoading(false)
    }
  }, [apiBase])

  useEffect(() => {
    void refresh()
    const id = window.setInterval(() => void refresh(), pollMs)
    return () => window.clearInterval(id)
  }, [refresh, pollMs])

  const mesSinkDisabled =
    runtime?.mode === 'mock-local' || runtime?.mode === 'lightsout'

  return { runtime, capabilities, loading, error, refresh, mesSinkDisabled }
}
