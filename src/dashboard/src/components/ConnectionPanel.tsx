import { useEffect, useMemo, useState } from 'react'
import GlassCard from './GlassCard'
import Button from './ui/button'
import { useRos } from '../ros/RosContext'
import { useRuntimeConfig } from '../config/RuntimeConfig'

type ConnStatus = 'connected' | 'connecting' | 'disconnected'

const statusStyle = (status: ConnStatus) => {
  if (status === 'connected') return 'bg-emerald-400/15 text-emerald-400'
  if (status === 'connecting') return 'bg-amber-400/15 text-amber-400'
  return 'bg-rose-400/15 text-rose-400'
}

export default function ConnectionPanel() {
  const ros = useRos()
  const { apiBase, rosbridgeUrl, setApiBase, setRosbridgeUrl } = useRuntimeConfig()
  const [open, setOpen] = useState(false)
  const [apiInput, setApiInput] = useState(apiBase)
  const [rosInput, setRosInput] = useState(rosbridgeUrl)
  const [apiStatus, setApiStatus] = useState<ConnStatus>('connecting')
  const [rosStatus, setRosStatus] = useState<ConnStatus>('connecting')
  const [hostName, setHostName] = useState<string>('—')

  useEffect(() => setApiInput(apiBase), [apiBase])
  useEffect(() => setRosInput(rosbridgeUrl), [rosbridgeUrl])

  useEffect(() => {
    let cancelled = false
    const ping = async () => {
      setApiStatus('connecting')
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
    ros.on('connection', () => setRosStatus('connected'))
    ros.on('close', () => setRosStatus('disconnected'))
    ros.on('error', () => setRosStatus('disconnected'))
  }, [ros])

  const canSave = useMemo(
    () => apiInput.trim().length > 0 && rosInput.trim().length > 0,
    [apiInput, rosInput]
  )

  return (
    <GlassCard>
      <div className="flex flex-col gap-3">
        <div className="flex items-center justify-between">
          <div>
            <h3 className="text-lg font-semibold">Connection</h3>
            <div className="text-xs text-white/60">Connected host: {hostName}</div>
          </div>
          <Button variant="ghost" size="sm" onClick={() => setOpen((v) => !v)}>
            {open ? 'Hide' : 'Edit'}
          </Button>
        </div>
        <div className="flex items-center gap-3">
          <div className="flex items-center gap-2">
            <span className="text-xs text-white/70">API</span>
            <span className={`text-xs px-2 py-0.5 rounded-md ${statusStyle(apiStatus)}`}>{apiStatus}</span>
          </div>
          <div className="flex items-center gap-2">
            <span className="text-xs text-white/70">rosbridge</span>
            <span className={`text-xs px-2 py-0.5 rounded-md ${statusStyle(rosStatus)}`}>{rosStatus}</span>
          </div>
        </div>

        {open && (
          <div className="flex flex-col gap-3">
            <div className="flex flex-col gap-1">
              <label className="text-xs text-white/60">API base</label>
              <input
                value={apiInput}
                onChange={(e) => setApiInput(e.target.value)}
                placeholder="http://<pi-ip>:8000"
                className="px-2 py-1 bg-transparent border border-slate-800 rounded text-sm"
              />
            </div>
            <div className="flex flex-col gap-1">
              <label className="text-xs text-white/60">rosbridge URL</label>
              <input
                value={rosInput}
                onChange={(e) => setRosInput(e.target.value)}
                placeholder="ws://<pi-ip>:9090"
                className="px-2 py-1 bg-transparent border border-slate-800 rounded text-sm"
              />
            </div>
            <div className="flex items-center gap-2">
              <Button
                onClick={() => {
                  if (!canSave) return
                  setApiBase(apiInput)
                  setRosbridgeUrl(rosInput)
                }}
                disabled={!canSave}
              >
                Save & Reconnect
              </Button>
              <span className="text-xs text-white/50">Changes persist in this browser.</span>
            </div>
          </div>
        )}
      </div>
    </GlassCard>
  )
}
