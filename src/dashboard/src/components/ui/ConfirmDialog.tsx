import { ReactNode } from 'react'
import Button from './button'

type Props = {
  open: boolean
  title: string
  message: string
  confirmLabel?: string
  onConfirm: () => void
  onCancel: () => void
  loading?: boolean
}

export default function ConfirmDialog({
  open,
  title,
  message,
  confirmLabel = 'Confirm',
  onConfirm,
  onCancel,
  loading = false,
}: Props) {
  if (!open) return null
  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center p-4">
      <button
        type="button"
        className="absolute inset-0 bg-[var(--overlay-backdrop)]"
        onClick={onCancel}
        aria-label="Cancel"
      />
      <div
        role="dialog"
        aria-modal="true"
        aria-labelledby="confirm-dialog-title"
        className="relative z-10 w-full max-w-md rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--surface-1)] p-5 shadow-card-md"
      >
        <h3 id="confirm-dialog-title" className="font-display text-lg font-semibold">
          {title}
        </h3>
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

export function SubsystemCard({
  title,
  statusLabel,
  statusTone = 'neutral',
  lastSeen,
  children,
}: {
  title: string
  statusLabel: string
  statusTone?: 'good' | 'warn' | 'bad' | 'info' | 'neutral' | 'idle'
  lastSeen?: string
  children: ReactNode
}) {
  return (
    <div className="rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--card-surface)] p-5 shadow-card">
      <div className="mb-4 flex items-start justify-between gap-3">
        <div>
          <h3 className="font-display text-base font-semibold">{title}</h3>
          {lastSeen && (
            <p className="mt-1 text-xs text-[var(--text-faint)]">Last signal: {lastSeen}</p>
          )}
        </div>
        <span
          className={`rounded-full px-2.5 py-1 text-xs font-medium ${
            statusTone === 'good'
              ? 'bg-[var(--status-good-bg)] text-[var(--status-good-fg)]'
              : statusTone === 'warn'
                ? 'bg-[var(--status-warn-bg)] text-[var(--status-warn-fg)]'
                : statusTone === 'bad'
                  ? 'bg-[var(--status-bad-bg)] text-[var(--status-bad-fg)]'
                  : statusTone === 'info'
                    ? 'bg-[var(--status-info-bg)] text-[var(--status-info-fg)]'
                    : 'bg-[var(--status-neutral-bg)] text-[var(--status-neutral-fg)]'
          }`}
        >
          {statusLabel}
        </span>
      </div>
      {children}
    </div>
  )
}
