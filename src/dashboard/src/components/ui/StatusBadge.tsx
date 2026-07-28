import { cn } from '../../lib/cn'

export type StatusTone = 'good' | 'warn' | 'bad' | 'info' | 'neutral' | 'idle'

const toneStyles: Record<StatusTone, string> = {
  good: 'bg-[var(--status-good-bg)] text-[var(--status-good-fg)]',
  warn: 'bg-[var(--status-warn-bg)] text-[var(--status-warn-fg)]',
  bad: 'bg-[var(--status-bad-bg)] text-[var(--status-bad-fg)]',
  info: 'bg-[var(--status-info-bg)] text-[var(--status-info-fg)]',
  neutral: 'bg-[var(--status-neutral-bg)] text-[var(--status-neutral-fg)]',
  idle: 'bg-[var(--status-idle-bg)] text-[var(--status-idle-fg)]',
}

const dotStyles: Record<StatusTone, string> = {
  good: 'bg-[var(--status-good-fg)]',
  warn: 'bg-[var(--status-warn-fg)]',
  bad: 'bg-[var(--status-bad-fg)]',
  info: 'bg-[var(--status-info-fg)]',
  neutral: 'bg-[var(--status-neutral-fg)]',
  idle: 'bg-[var(--status-idle-fg)]',
}

type Props = {
  label: string
  tone?: StatusTone
  pulse?: boolean
  className?: string
  title?: string
}

export default function StatusBadge({
  label,
  tone = 'neutral',
  pulse = false,
  className,
  title,
}: Props) {
  return (
    <span
      title={title}
      className={cn(
        'inline-flex items-center gap-1.5 rounded-full px-2.5 py-1 text-xs font-medium',
        toneStyles[tone],
        className
      )}
    >
      <span
        className={cn(
          'h-1.5 w-1.5 rounded-full',
          dotStyles[tone],
          pulse && 'animate-pulse-dot'
        )}
      />
      {label}
    </span>
  )
}
