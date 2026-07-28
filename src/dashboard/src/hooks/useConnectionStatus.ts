import { useEffect, useState } from 'react'
import { useRos } from '../ros/RosContext'
import { useRuntimeConfig } from '../config/RuntimeConfig'

export type ConnStatus = 'connected' | 'connecting' | 'disconnected'

function rosIsConnected(ros: { isConnected?: boolean } | null): boolean {
  return Boolean(ros && ros.isConnected)
}

export function useConnectionStatus(weightStale?: boolean) {
  const ros = useRos()
  const { apiBase } = useRuntimeConfig()
  const [apiStatus, setApiStatus] = useState<ConnStatus>('connecting')
  const [rosStatus, setRosStatus] = useState<ConnStatus>(() =>
    rosIsConnected(ros) ? 'connected' : ros ? 'connecting' : 'disconnected',
  )
  const [hostName, setHostName] = useState('—')

  useEffect(() => {
    let cancelled = false
    const ping = async () => {
      try {
        const res = await fetch(`${apiBase}/host_info`, { cache: 'no-store' })
        if (!res.ok) throw new Error('api-failed')
        const data = await res.json()
        if (cancelled) return
        setApiStatus('connected')
        setHostName(data?.hostname || 'unknown')
      } catch {
        if (cancelled) return
        setApiStatus('disconnected')
        setHostName('—')
      }
    }
    void ping()
    const id = window.setInterval(ping, 5000)
    return () => {
      cancelled = true
      window.clearInterval(id)
    }
  }, [apiBase])

  useEffect(() => {
    if (!ros) {
      setRosStatus('disconnected')
      return
    }
    // Page remounts must not flash "connecting" — the shared RosProvider
    // socket is often already open, and 'connection' will not fire again.
    setRosStatus(rosIsConnected(ros) ? 'connected' : 'connecting')
    const onConnect = () => setRosStatus('connected')
    const onDisconnect = () => setRosStatus('disconnected')
    ros.on('connection', onConnect)
    ros.on('close', onDisconnect)
    ros.on('error', onDisconnect)
    return () => {
      ros.off('connection', onConnect)
      ros.off('close', onDisconnect)
      ros.off('error', onDisconnect)
    }
  }, [ros])

  const scaleStatus: ConnStatus =
    weightStale === undefined
      ? 'connecting'
      : weightStale
        ? 'disconnected'
        : 'connected'

  return { apiStatus, rosStatus, scaleStatus, hostName }
}
