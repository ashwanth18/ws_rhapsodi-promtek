import { ReactNode, useId } from 'react'
import { cn } from '../../lib/cn'

type Props = {
  label: string
  error?: string | null
  hint?: string
  children: (id: string) => ReactNode
  className?: string
}

export default function Field({ label, error, hint, children, className }: Props) {
  const autoId = useId()
  const fieldId = `field-${autoId}`

  return (
    <div className={cn('space-y-1', className)}>
      <label htmlFor={fieldId} className="block text-xs font-medium text-[var(--text-muted)]">
        {label}
      </label>
      {children(fieldId)}
      {hint && !error && (
        <p className="text-xs text-[var(--text-faint)]">{hint}</p>
      )}
      {error && <p className="text-xs text-[var(--status-bad-fg)]">{error}</p>}
    </div>
  )
}
