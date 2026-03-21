import { ReactNode } from 'react'

export default function GlassCard({ children, className = '' }: { children: ReactNode; className?: string }) {
  return (
    <div
      className={
        'rounded-[12px] border border-[var(--border)] bg-[var(--card-surface)] p-4 text-[var(--text-primary)] ' +
        'backdrop-blur-md shadow-[var(--glass-shadow)] ' +
        'transition-transform duration-200 hover:-translate-y-0.5 hover:shadow-[var(--glass-shadow-hover)] ' +
        className
      }
    >
      {children}
    </div>
  )
}


