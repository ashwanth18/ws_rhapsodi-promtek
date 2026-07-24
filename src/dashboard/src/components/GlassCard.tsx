import { ReactNode } from 'react'
import { cn } from '../lib/cn'

export default function GlassCard({
  children,
  className = '',
  padding = true,
}: {
  children: ReactNode
  className?: string
  padding?: boolean
}) {
  return (
    <div
      className={cn(
        'rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--card-surface)] text-[var(--text-primary)] shadow-card',
        padding && 'p-5',
        className
      )}
    >
      {children}
    </div>
  )
}
