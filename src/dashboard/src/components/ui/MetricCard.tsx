import { ReactNode } from 'react'
import { cn } from '../../lib/cn'

type Props = {
  label: string
  value: ReactNode
  unit?: string
  help?: string
  className?: string
}

export default function MetricCard({
  label,
  value,
  unit,
  help,
  className,
}: Props) {
  return (
    <div
      className={cn(
        'rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--card-surface)] p-4 shadow-card',
        className
      )}
    >
      <div className="text-xs font-medium uppercase tracking-wider text-[var(--text-faint)]">
        {label}
      </div>
      <div className="mt-2 flex items-baseline gap-1.5">
        <div className="font-display text-2xl font-semibold font-tabular text-[var(--text-primary)]">
          {value}
        </div>
        {unit && (
          <span className="text-sm text-[var(--text-muted)]">{unit}</span>
        )}
      </div>
      {help && (
        <div className="mt-1 text-xs text-[var(--text-faint)]">{help}</div>
      )}
    </div>
  )
}
