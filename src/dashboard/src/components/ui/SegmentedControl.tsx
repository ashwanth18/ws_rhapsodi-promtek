import { KeyboardEvent, useId } from 'react'
import { cn } from '../../lib/cn'

export type Segment<T extends string> = {
  value: T
  label: string
  hint?: string
}

type Props<T extends string> = {
  value: T
  onChange: (value: T) => void
  segments: Segment<T>[]
  disabled?: boolean
  ariaLabel: string
  className?: string
}

export default function SegmentedControl<T extends string>({
  value,
  onChange,
  segments,
  disabled = false,
  ariaLabel,
  className,
}: Props<T>) {
  const groupId = useId()

  const onKeyDown = (event: KeyboardEvent<HTMLButtonElement>, index: number) => {
    if (disabled) return
    let next = index
    if (event.key === 'ArrowRight' || event.key === 'ArrowDown') {
      event.preventDefault()
      next = (index + 1) % segments.length
    } else if (event.key === 'ArrowLeft' || event.key === 'ArrowUp') {
      event.preventDefault()
      next = (index - 1 + segments.length) % segments.length
    } else {
      return
    }
    onChange(segments[next].value)
    const el = document.getElementById(`${groupId}-${segments[next].value}`)
    el?.focus()
  }

  return (
    <div
      role="radiogroup"
      aria-label={ariaLabel}
      className={cn(
        'inline-flex w-full flex-wrap rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] p-0.5 sm:w-auto',
        className
      )}
    >
      {segments.map((segment, index) => {
        const selected = segment.value === value
        return (
          <button
            key={segment.value}
            id={`${groupId}-${segment.value}`}
            type="button"
            role="radio"
            aria-checked={selected}
            title={segment.hint}
            disabled={disabled}
            onClick={() => onChange(segment.value)}
            onKeyDown={(e) => onKeyDown(e, index)}
            className={cn(
              'min-w-0 flex-1 rounded-[calc(var(--radius-sm)-2px)] px-3 py-1.5 text-xs font-medium transition-colors sm:flex-none',
              selected
                ? 'bg-[var(--accent-muted)] text-[var(--accent)]'
                : 'text-[var(--text-muted)] hover:text-[var(--text-primary)]',
              disabled && 'cursor-not-allowed opacity-50'
            )}
          >
            {segment.label}
          </button>
        )
      })}
    </div>
  )
}
