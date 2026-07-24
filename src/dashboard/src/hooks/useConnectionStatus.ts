import { useEffect, useState } from 'react'
import { useRos } from '../ros/RosContext'
import { useRuntimeConfig } from '../config/RuntimeConfig'

export type ConnStatus = 'connected' | 'connecting' | 'disconnected'

export function useConnectionStatus(weightStale?: boolean) {
  const ros = useRos()
  const { apiBase } = useRuntimeConfig()
  const [apiStatus, setApiStatus] = useState<ConnStatus>('connecting')
  const [rosStatus, setRosStatus] = useState<ConnStatus>('connecting')
  const [hostName, setHostName] = useState('—')

  useEffect(() => {
    let cancelled = false
    setApiStatus('connecting')
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
    ping()
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
    setRosStatus('connecting')
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
