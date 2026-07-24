import type { ButtonHTMLAttributes, ReactNode } from 'react'
import { cn } from '../lib/cn'

export function StatusBadge({
  label,
  tone = 'neutral',
  pulse = false,
}: {
  label: string
  tone?: 'good' | 'warn' | 'bad' | 'info' | 'neutral'
  pulse?: boolean
}) {
  const tones: Record<string, string> = {
    good: 'bg-[var(--status-good-bg)] text-[var(--status-good-fg)]',
    warn: 'bg-[var(--status-warn-bg)] text-[var(--status-warn-fg)]',
    bad: 'bg-[var(--status-bad-bg)] text-[var(--status-bad-fg)]',
    info: 'bg-[var(--status-info-bg)] text-[var(--status-info-fg)]',
    neutral: 'bg-[var(--status-neutral-bg)] text-[var(--status-neutral-fg)]',
  }
  return (
    <span
      className={cn(
        'inline-flex items-center gap-1.5 rounded-full px-2.5 py-0.5 text-xs font-medium',
        tones[tone],
      )}
    >
      {pulse ? (
        <span className="relative flex h-1.5 w-1.5">
          <span className="absolute inline-flex h-full w-full animate-ping rounded-full bg-current opacity-60" />
          <span className="relative inline-flex h-1.5 w-1.5 rounded-full bg-current" />
        </span>
      ) : null}
      {label}
    </span>
  )
}

export function MetricCard({
  label,
  value,
  unit,
  help,
}: {
  label: string
  value: ReactNode
  unit?: string
  help?: string
}) {
  return (
    <div className="rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--card-surface)] p-4">
      <div className="text-xs uppercase tracking-wide text-[var(--text-muted)]">{label}</div>
      <div className="mt-2 flex items-baseline gap-1">
        <div className="font-display text-2xl font-semibold text-[var(--text-primary)]">
          {value}
        </div>
        {unit ? <div className="text-sm text-[var(--text-muted)]">{unit}</div> : null}
      </div>
      {help ? <div className="mt-1 text-xs text-[var(--text-muted)]">{help}</div> : null}
    </div>
  )
}

export function Button({
  children,
  variant = 'primary',
  size = 'md',
  className,
  ...props
}: ButtonHTMLAttributes<HTMLButtonElement> & {
  variant?: 'primary' | 'ghost' | 'danger' | 'outline'
  size?: 'sm' | 'md' | 'lg'
}) {
  const variants = {
    primary:
      'bg-[var(--button-primary-bg)] text-[var(--button-primary-text)] hover:bg-[var(--button-primary-hover)]',
    ghost: 'bg-transparent text-[var(--text-secondary)] hover:bg-white/5',
    danger: 'bg-[var(--status-bad-fg)] text-black hover:opacity-90',
    outline:
      'border border-[var(--border)] bg-transparent text-[var(--text-secondary)] hover:bg-white/5',
  }
  const sizes = {
    sm: 'px-2.5 py-1 text-xs',
    md: 'px-3.5 py-2 text-sm',
    lg: 'px-4 py-2.5 text-sm',
  }
  return (
    <button
      className={cn(
        'inline-flex items-center justify-center gap-2 rounded-[var(--radius-sm)] font-medium transition disabled:cursor-not-allowed disabled:opacity-50',
        variants[variant],
        sizes[size],
        className,
      )}
      {...props}
    >
      {children}
    </button>
  )
}

export function SectionHeader({
  title,
  description,
  action,
}: {
  title: string
  description?: string
  action?: ReactNode
}) {
  return (
    <div className="mb-4 flex flex-wrap items-start justify-between gap-3">
      <div>
        <h2 className="font-display text-lg font-semibold text-[var(--text-primary)]">
          {title}
        </h2>
        {description ? (
          <p className="mt-1 text-sm text-[var(--text-muted)]">{description}</p>
        ) : null}
      </div>
      {action}
    </div>
  )
}

export function DataTable<T>({
  columns,
  rows,
  rowKey,
  emptyMessage = 'No data available',
  onRowClick,
}: {
  columns: {
    key: string
    header: string
    render: (row: T) => ReactNode
    className?: string
  }[]
  rows: T[]
  rowKey: (row: T) => string | number
  emptyMessage?: string
  onRowClick?: (row: T) => void
}) {
  if (rows.length === 0) {
    return (
      <div className="rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--card-surface)] px-6 py-12 text-center text-sm text-[var(--text-muted)]">
        {emptyMessage}
      </div>
    )
  }
  return (
    <div className="overflow-hidden rounded-[var(--radius-md)] border border-[var(--border)]">
      <table className="min-w-full divide-y divide-[var(--border)] text-sm">
        <thead className="bg-[var(--surface-1)]">
          <tr>
            {columns.map((col) => (
              <th
                key={col.key}
                className={cn(
                  'px-4 py-3 text-left text-xs font-medium uppercase tracking-wide text-[var(--text-muted)]',
                  col.className,
                )}
              >
                {col.header}
              </th>
            ))}
          </tr>
        </thead>
        <tbody className="divide-y divide-[var(--border)] bg-[var(--card-surface)]">
          {rows.map((row) => (
            <tr
              key={rowKey(row)}
              className={cn(
                'transition hover:bg-white/[0.03]',
                onRowClick ? 'cursor-pointer' : '',
              )}
              onClick={() => onRowClick?.(row)}
            >
              {columns.map((col) => (
                <td key={col.key} className={cn('px-4 py-3', col.className)}>
                  {col.render(row)}
                </td>
              ))}
            </tr>
          ))}
        </tbody>
      </table>
    </div>
  )
}

export function ConfirmDialog({
  open,
  title,
  message,
  confirmLabel = 'Confirm',
  loading,
  onConfirm,
  onCancel,
}: {
  open: boolean
  title: string
  message: string
  confirmLabel?: string
  loading?: boolean
  onConfirm: () => void
  onCancel: () => void
}) {
  if (!open) return null
  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center bg-black/65 p-4">
      <div className="w-full max-w-md rounded-[var(--radius-lg)] border border-[var(--border)] bg-[var(--surface-2)] p-5 shadow-xl">
        <h3 className="font-display text-lg font-semibold">{title}</h3>
        <p className="mt-2 text-sm text-[var(--text-muted)]">{message}</p>
        <div className="mt-5 flex justify-end gap-2">
          <Button variant="ghost" onClick={onCancel} disabled={loading}>
            Cancel
          </Button>
          <Button onClick={onConfirm} disabled={loading}>
            {loading ? 'Working…' : confirmLabel}
          </Button>
        </div>
      </div>
    </div>
  )
}

export function deployTone(
  status?: string | null,
): 'good' | 'warn' | 'bad' | 'info' | 'neutral' {
  switch (status) {
    case 'success':
      return 'good'
    case 'running':
      return 'info'
    case 'failed':
      return 'bad'
    case 'rolled_back':
      return 'warn'
    default:
      return 'neutral'
  }
}
