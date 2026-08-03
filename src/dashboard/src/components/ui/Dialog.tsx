import { ReactNode } from 'react'
import { cn } from '../../lib/cn'

type Props = {
  open: boolean
  title: string
  onClose: () => void
  children: ReactNode
  className?: string
  footer?: ReactNode
}

export default function Dialog({
  open,
  title,
  onClose,
  children,
  className,
  footer,
}: Props) {
  if (!open) return null

  return (
    <div className="fixed inset-0 z-50 flex items-center justify-center p-4">
      <button
        type="button"
        className="absolute inset-0 bg-[var(--overlay-backdrop)]"
        onClick={onClose}
        aria-label="Close dialog backdrop"
      />
      <div
        role="dialog"
        aria-modal="true"
        aria-labelledby="dialog-title"
        className={cn(
          'relative z-10 w-full max-w-lg rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--surface)] p-5 shadow-card-md',
          className
        )}
      >
        <div className="mb-4 flex items-center justify-between gap-3">
          <h2 id="dialog-title" className="font-display text-lg font-semibold">
            {title}
          </h2>
          <button
            type="button"
            className="text-[var(--text-muted)] hover:text-[var(--text-primary)]"
            onClick={onClose}
            aria-label="Close"
          >
            ✕
          </button>
        </div>
        {children}
        {footer && <div className="mt-5 flex items-center justify-end gap-2">{footer}</div>}
      </div>
    </div>
  )
}
