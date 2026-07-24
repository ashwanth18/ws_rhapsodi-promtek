import { useEffect, useMemo, useState } from 'react'
import Button from './ui/button'
import { useRos } from '../ros/RosContext'
import { useRuntimeConfig } from '../config/RuntimeConfig'
import { ConnStatus } from '../hooks/useConnectionStatus'
import { X } from 'lucide-react'

function statusLabel(status: ConnStatus) {
  if (status === 'connected') return 'Connected'
  if (status === 'connecting') return 'Connecting'
  return 'Disconnected'
}

type Props = {
  open: boolean
  onClose: () => void
  apiStatus: ConnStatus
  rosStatus: ConnStatus
  hostName: string
}

export default function SettingsDrawer({
  open,
  onClose,
  apiStatus,
  rosStatus,
  hostName,
}: Props) {
  const { apiBase, rosbridgeUrl, setApiBase, setRosbridgeUrl } = useRuntimeConfig()
  const [apiInput, setApiInput] = useState(apiBase)
  const [rosInput, setRosInput] = useState(rosbridgeUrl)

  useEffect(() => setApiInput(apiBase), [apiBase])
  useEffect(() => setRosInput(rosbridgeUrl), [rosbridgeUrl])

  const canSave = useMemo(
    () => apiInput.trim().length > 0 && rosInput.trim().length > 0,
    [apiInput, rosInput]
  )

  if (!open) return null

  return (
    <div className="fixed inset-0 z-50 flex justify-end">
      <button
        type="button"
        className="absolute inset-0 bg-[var(--overlay-backdrop)]"
        onClick={onClose}
        aria-label="Close settings"
      />
      <aside className="relative z-10 flex h-full w-full max-w-md flex-col border-l border-[var(--border)] bg-[var(--surface-1)] shadow-card-md">
        <div className="flex items-center justify-between border-b border-[var(--border)] px-5 py-4">
          <div>
            <h2 className="font-display text-lg font-semibold">Connection Settings</h2>
            <p className="text-xs text-[var(--text-muted)]">Host: {hostName}</p>
          </div>
          <Button variant="ghost" size="sm" onClick={onClose} aria-label="Close">
            <X className="h-4 w-4" />
          </Button>
        </div>
        <div className="flex-1 space-y-5 overflow-y-auto p-5">
          <div className="grid grid-cols-2 gap-3 text-sm">
            <div className="rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] p-3">
              <div className="text-xs text-[var(--text-faint)]">API</div>
              <div className="mt-1 font-medium">{statusLabel(apiStatus)}</div>
            </div>
            <div className="rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] p-3">
              <div className="text-xs text-[var(--text-faint)]">rosbridge</div>
              <div className="mt-1 font-medium">{statusLabel(rosStatus)}</div>
            </div>
          </div>
          <div className="space-y-2">
            <label className="text-xs font-medium text-[var(--text-muted)]">API base</label>
            <input
              value={apiInput}
              onChange={(e) => setApiInput(e.target.value)}
              placeholder="http://169.254.200.201:8000"
              className="w-full rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2 text-sm text-[var(--text-primary)] outline-none focus:border-[var(--accent)]"
            />
          </div>
          <div className="space-y-2">
            <label className="text-xs font-medium text-[var(--text-muted)]">rosbridge URL</label>
            <input
              value={rosInput}
              onChange={(e) => setRosInput(e.target.value)}
              placeholder="ws://169.254.200.201:9090"
              className="w-full rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2 text-sm text-[var(--text-primary)] outline-none focus:border-[var(--accent)]"
            />
          </div>
        </div>
        <div className="border-t border-[var(--border)] p-5">
          <Button
            className="w-full"
            disabled={!canSave}
            onClick={() => {
              if (!canSave) return
              setApiBase(apiInput)
              setRosbridgeUrl(rosInput)
              onClose()
            }}
          >
            Save & Reconnect
          </Button>
          <p className="mt-2 text-center text-xs text-[var(--text-faint)]">
            Settings persist in this browser.
          </p>
        </div>
      </aside>
    </div>
  )
}
