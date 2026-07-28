/**
 * Legacy Connection panel — connection monitoring now lives on Controls
 * (Cell Signal Deck). Kept as a thin endpoint editor for any old embeds.
 */
import { useEffect, useMemo, useState } from 'react'
import GlassCard from './GlassCard'
import Button from './ui/button'
import { useRuntimeConfig } from '../config/RuntimeConfig'
import { Link } from 'react-router-dom'

export default function ConnectionPanel() {
  const { apiBase, rosbridgeUrl, setApiBase, setRosbridgeUrl } = useRuntimeConfig()
  const [apiInput, setApiInput] = useState(apiBase)
  const [rosInput, setRosInput] = useState(rosbridgeUrl)

  useEffect(() => setApiInput(apiBase), [apiBase])
  useEffect(() => setRosInput(rosbridgeUrl), [rosbridgeUrl])

  const canSave = useMemo(
    () => apiInput.trim().length > 0 && rosInput.trim().length > 0,
    [apiInput, rosInput],
  )

  return (
    <GlassCard>
      <div className="flex flex-col gap-3">
        <div>
          <h3 className="text-lg font-semibold">Connection</h3>
          <p className="text-xs text-[var(--text-muted)]">
            Full link integrity (API, rosbridge, arm, scale, Condor) is on{' '}
            <Link className="text-[var(--accent)] underline-offset-2 hover:underline" to="/controls">
              Controls → Cell Signal Deck
            </Link>
            .
          </p>
        </div>
        <div className="flex flex-col gap-2">
          <label className="text-xs text-[var(--text-muted)]">API base</label>
          <input
            value={apiInput}
            onChange={(e) => setApiInput(e.target.value)}
            className="rounded border border-[var(--border)] bg-transparent px-2 py-1 text-sm"
          />
          <label className="text-xs text-[var(--text-muted)]">rosbridge URL</label>
          <input
            value={rosInput}
            onChange={(e) => setRosInput(e.target.value)}
            className="rounded border border-[var(--border)] bg-transparent px-2 py-1 text-sm"
          />
          <Button
            disabled={!canSave}
            onClick={() => {
              if (!canSave) return
              setApiBase(apiInput.trim())
              setRosbridgeUrl(rosInput.trim())
            }}
          >
            Save & Reconnect
          </Button>
        </div>
      </div>
    </GlassCard>
  )
}
