import { useEffect, useState } from 'react'
import GlassCard from '../GlassCard'
import StatusBadge from '../ui/StatusBadge'
import { useRuntimeConfig } from '../../config/RuntimeConfig'

function formatControlLaw(law: string | null): string {
  if (!law) return '—'
  if (law === 'pid_inflight') return 'PID+inflight'
  if (law === 'pid') return 'PID'
  if (law === 'bangbang') return 'Bang-bang'
  return law
}

export { formatControlLaw }

const SHOW_CARD_KEY = 'dashboard.showPourControlLawCard'

export function usePourControlLawVisibility() {
  const [visible, setVisible] = useState(() => {
    try {
      return localStorage.getItem(SHOW_CARD_KEY) !== '0'
    } catch {
      return true
    }
  })

  const setShowCard = (checked: boolean) => {
    setVisible(checked)
    try {
      localStorage.setItem(SHOW_CARD_KEY, checked ? '1' : '0')
    } catch {
      /* ignore */
    }
  }

  return { visible, setShowCard }
}

/** Compact checkbox for SectionHeader — keeps a way to re-show when card is hidden. */
export function PourControlLawToggle({
  visible,
  onChange,
}: {
  visible: boolean
  onChange: (next: boolean) => void
}) {
  return (
    <label className="flex cursor-pointer items-center gap-1.5 rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-2.5 py-1.5 text-xs text-[var(--text-muted)]">
      <input
        type="checkbox"
        className="h-3.5 w-3.5 accent-[var(--accent)]"
        checked={visible}
        onChange={(e) => onChange(e.target.checked)}
      />
      Pour law
    </label>
  )
}

export default function PourControlLawCard() {
  const { apiBase } = useRuntimeConfig()
  const [controlLaw, setControlLaw] = useState<string | null>(null)

  useEffect(() => {
    let cancelled = false
    const poll = async () => {
      try {
        const res = await fetch(`${apiBase}/host_info`, { cache: 'no-store' })
        if (!res.ok) return
        const data = await res.json()
        if (cancelled) return
        const law = String(data?.pour_control_law || '').trim().toLowerCase()
        setControlLaw(law || null)
      } catch {
        /* ignore */
      }
    }
    void poll()
    const id = window.setInterval(poll, 5000)
    return () => {
      cancelled = true
      window.clearInterval(id)
    }
  }, [apiBase])

  return (
    <GlassCard className="h-full">
      <div className="flex items-start justify-between gap-2">
        <div className="text-xs font-medium uppercase tracking-wider text-[var(--text-faint)]">
          Pour Control Law
        </div>
        <StatusBadge
          label={formatControlLaw(controlLaw)}
          tone={
            controlLaw === 'pid' || controlLaw === 'pid_inflight'
              ? 'info'
              : 'neutral'
          }
          title={`control_law_type=${controlLaw ?? 'unknown'}`}
        />
      </div>
      <div className="mt-2 font-display text-2xl font-semibold font-tabular text-[var(--text-primary)]">
        {formatControlLaw(controlLaw)}
      </div>
      <div className="mt-1 text-xs text-[var(--text-faint)]">
        Applies for the whole batch / pour cycle
      </div>
    </GlassCard>
  )
}
