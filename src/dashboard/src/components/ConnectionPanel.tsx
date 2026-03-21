import { useEffect, useMemo, useState } from 'react'
import GlassCard from './GlassCard'
import Button from './ui/button'
import { useRos } from '../ros/RosContext'
import { useRuntimeConfig } from '../config/RuntimeConfig'

type ConnStatus = 'connected' | 'connecting' | 'disconnected'

const statusStyle = (status: ConnStatus) => {
  if (status === 'connected')
    return 'bg-[var(--status-good-bg)] text-[var(--status-good-fg)]'
  if (status === 'connecting')
    return 'bg-[var(--status-warn-bg)] text-[var(--status-warn-fg)]'
  return 'bg-[var(--status-bad-bg)] text-[var(--status-bad-fg)]'
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
            <div className="text-xs text-[var(--text-muted)]">Connected host: {hostName}</div>
          </div>
          <Button variant="ghost" size="sm" onClick={() => setOpen((v) => !v)}>
            {open ? 'Hide' : 'Edit'}
          </Button>
        </div>
        <div className="flex items-center gap-3">
          <div className="flex items-center gap-2">
            <span className="text-xs text-[var(--text-secondary)]">API</span>
            <span className={`text-xs px-2 py-0.5 rounded-md ${statusStyle(apiStatus)}`}>{apiStatus}</span>
          </div>
          <div className="flex items-center gap-2">
            <span className="text-xs text-[var(--text-secondary)]">rosbridge</span>
            <span className={`text-xs px-2 py-0.5 rounded-md ${statusStyle(rosStatus)}`}>{rosStatus}</span>
          </div>
        </div>

        {open && (
          <div className="flex flex-col gap-3">
            <div className="flex flex-col gap-1">
              <label className="text-xs text-[var(--text-muted)]">API base</label>
              <input
                value={apiInput}
                onChange={(e) => setApiInput(e.target.value)}
                placeholder="http://<pi-ip>:8000"
                className="rounded border border-[var(--border)] bg-transparent px-2 py-1 text-sm text-[var(--text-primary)]"
              />
            </div>
            <div className="flex flex-col gap-1">
              <label className="text-xs text-[var(--text-muted)]">rosbridge URL</label>
              <input
                value={rosInput}
                onChange={(e) => setRosInput(e.target.value)}
                placeholder="ws://<pi-ip>:9090"
                className="rounded border border-[var(--border)] bg-transparent px-2 py-1 text-sm text-[var(--text-primary)]"
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
              <span className="text-xs text-[var(--text-faint)]">Changes persist in this browser.</span>
            </div>
          </div>
        )}
      </div>
    </GlassCard>
  )
}
