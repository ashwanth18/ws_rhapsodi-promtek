import { useEffect, useState } from 'react'
import GlassCard from '../GlassCard'
import StatusBadge, { type StatusTone } from '../ui/StatusBadge'
import { formatNumber } from './operationsUtils'

type EpisodeRow = {
  id: number
  episode_index: number | null
  target_weight_g: number | null
  net_weight_g: number | null
  signed_error_g?: number | null
  overshoot_g?: number | null
  pour_outcome?: string | null
}

type Props = {
  apiBase: string
  batchId: string | null
  activeEpisode: number | null
}

function outcomeTone(outcome: string | null | undefined): StatusTone {
  if (outcome === 'achieved') return 'good'
  if (outcome === 'overshoot') return 'bad'
  if (outcome === 'timeout') return 'warn'
  return 'idle'
}

function signedError(row: EpisodeRow): number | null {
  if (row.signed_error_g != null && Number.isFinite(row.signed_error_g)) {
    return row.signed_error_g
  }
  if (row.overshoot_g != null && Number.isFinite(row.overshoot_g)) {
    return row.overshoot_g
  }
  if (
    row.net_weight_g != null &&
    row.target_weight_g != null &&
    Number.isFinite(row.net_weight_g) &&
    Number.isFinite(row.target_weight_g)
  ) {
    return row.net_weight_g - row.target_weight_g
  }
  return null
}

export default function EpisodeStrip({ apiBase, batchId, activeEpisode }: Props) {
  const [rows, setRows] = useState<EpisodeRow[]>([])
  const [loading, setLoading] = useState(false)

  useEffect(() => {
    if (!batchId) {
      setRows([])
      return
    }
    let cancelled = false
    async function load() {
      try {
        setLoading(true)
        const search = new URLSearchParams({
          batch_id: batchId!,
          limit: '50',
          time_sort: 'asc',
          time_sort_field: 'start',
        })
        const res = await fetch(`${apiBase}/lightsout_processed?${search}`)
        const json = await res.json()
        if (!cancelled) setRows((json.rows || []) as EpisodeRow[])
      } catch {
        if (!cancelled) setRows([])
      } finally {
        if (!cancelled) setLoading(false)
      }
    }
    load()
    const id = setInterval(load, 5000)
    return () => {
      cancelled = true
      clearInterval(id)
    }
  }, [apiBase, batchId])

  if (!batchId) return null
  if (!loading && rows.length === 0) return null

  return (
    <GlassCard>
      <div className="mb-4 flex flex-wrap items-center justify-between gap-2">
        <div>
          <h3 className="font-display text-sm font-semibold uppercase tracking-wider text-[var(--text-faint)]">
            Episodes
          </h3>
          <p className="mt-1 text-xs text-[var(--text-muted)]">
            {rows.length} processed · batch {batchId}
            {loading ? ' · refreshing…' : ''}
          </p>
        </div>
      </div>
      <div className="flex gap-3 overflow-x-auto pb-1">
        {rows.map((row) => {
          const err = signedError(row)
          const isActive = row.episode_index === activeEpisode
          const tol =
            row.target_weight_g != null ? Math.abs(row.target_weight_g) * 0.02 : null
          const inTol =
            err != null && tol != null ? Math.abs(err) <= tol : null
          return (
            <div
              key={row.id}
              className={`min-w-[160px] shrink-0 rounded-[var(--radius-sm)] border px-3 py-3 ${
                isActive
                  ? 'border-[var(--accent)] bg-[var(--accent-muted)]'
                  : 'border-[var(--border)] bg-[var(--surface-2)]'
              }`}
            >
              <div className="flex items-center justify-between gap-2">
                <span className="text-xs font-semibold text-[var(--text-primary)]">
                  Ep {row.episode_index ?? '—'}
                </span>
                <StatusBadge
                  label={row.pour_outcome || (inTol ? 'ok' : inTol === false ? 'off' : 'done')}
                  tone={
                    row.pour_outcome
                      ? outcomeTone(row.pour_outcome)
                      : inTol
                        ? 'good'
                        : inTol === false
                          ? 'bad'
                          : 'idle'
                  }
                />
              </div>
              <div className="mt-2 text-xs text-[var(--text-muted)]">
                tgt{' '}
                <span className="font-tabular text-[var(--text-secondary)]">
                  {row.target_weight_g != null
                    ? `${formatNumber(row.target_weight_g, 1)} g`
                    : '—'}
                </span>
              </div>
              <div className="mt-1 text-xs text-[var(--text-muted)]">
                net{' '}
                <span className="font-tabular text-[var(--text-secondary)]">
                  {row.net_weight_g != null
                    ? `${formatNumber(row.net_weight_g, 1)} g`
                    : '—'}
                </span>
              </div>
              <div className="mt-1 text-xs text-[var(--text-muted)]">
                err{' '}
                <span className="font-tabular text-[var(--text-secondary)]">
                  {err != null
                    ? `${err >= 0 ? '+' : ''}${formatNumber(err, 1)} g`
                    : '—'}
                </span>
              </div>
            </div>
          )
        })}
      </div>
    </GlassCard>
  )
}
