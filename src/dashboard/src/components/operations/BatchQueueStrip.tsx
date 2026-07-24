import GlassCard from '../GlassCard'
import StatusBadge from '../ui/StatusBadge'

type QueueItem = {
  weightment_id: number
  ingredient_name: string | null
  ingredient_id: string | null
  target_weight_kg: number | null
  completed: boolean
  robot_status: string | null
}

type Props = {
  items: QueueItem[]
  activeWeightmentId?: number | null
}

export default function BatchQueueStrip({ items, activeWeightmentId }: Props) {
  if (items.length === 0) return null

  const completed = items.filter((i) => i.completed).length

  return (
    <GlassCard>
      <div className="mb-4 flex flex-wrap items-center justify-between gap-2">
        <div>
          <h3 className="font-display text-sm font-semibold uppercase tracking-wider text-[var(--text-faint)]">
            Batch Queue
          </h3>
          <p className="mt-1 text-xs text-[var(--text-muted)]">
            {completed} of {items.length} weightments complete
          </p>
        </div>
        <div className="h-2 w-40 overflow-hidden rounded-full bg-[var(--surface-strong)]">
          <div
            className="h-full rounded-full bg-[var(--status-good-fg)] transition-all"
            style={{ width: `${(completed / items.length) * 100}%` }}
          />
        </div>
      </div>
      <div className="flex gap-3 overflow-x-auto pb-1">
        {items.map((item) => {
          const isActive =
            item.weightment_id === activeWeightmentId ||
            item.robot_status === 'running' ||
            item.robot_status === 'starting'
          const tone = item.completed ? 'good' : isActive ? 'warn' : 'idle'
          return (
            <div
              key={item.weightment_id}
              className={`min-w-[180px] shrink-0 rounded-[var(--radius-sm)] border px-3 py-3 ${
                isActive
                  ? 'border-[var(--accent)] bg-[var(--accent-muted)]'
                  : 'border-[var(--border)] bg-[var(--surface-2)]'
              }`}
            >
              <div className="flex items-center justify-between gap-2">
                <span className="text-xs font-semibold text-[var(--text-primary)]">
                  Wt {item.weightment_id}
                </span>
                <StatusBadge
                  label={item.completed ? 'Done' : isActive ? 'Active' : 'Pending'}
                  tone={tone}
                  pulse={isActive}
                />
              </div>
              <div className="mt-2 text-xs text-[var(--text-muted)]">
                {item.ingredient_name || item.ingredient_id || '—'}
              </div>
              <div className="mt-1 text-xs font-tabular text-[var(--text-secondary)]">
                {item.target_weight_kg != null
                  ? `${item.target_weight_kg.toFixed(3)} kg`
                  : '—'}
              </div>
            </div>
          )
        })}
      </div>
    </GlassCard>
  )
}
