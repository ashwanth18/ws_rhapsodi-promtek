type DateTimeTextProps = {
  value: string | number | null | undefined
  className?: string
  dateClassName?: string
  timeClassName?: string
  emptyLabel?: string
}

function normalizeDate(value: string | number | null | undefined): Date | null {
  if (value == null) {
    return null
  }
  const parsed = new Date(value)
  if (Number.isNaN(parsed.getTime())) {
    return null
  }
  return parsed
}

export default function DateTimeText({
  value,
  className = '',
  dateClassName = '',
  timeClassName = 'text-xs text-[var(--text-muted)]',
  emptyLabel = '—',
}: DateTimeTextProps) {
  if (value == null) {
    return <div className={className}>{emptyLabel}</div>
  }

  const parsed = normalizeDate(value)
  if (!parsed) {
    return <div className={className}>{String(value)}</div>
  }

  return (
    <div className={className}>
      <div className={dateClassName}>{parsed.toLocaleDateString()}</div>
      <div className={timeClassName}>
        {parsed.toLocaleTimeString([], {
          hour: '2-digit',
          minute: '2-digit',
          second: '2-digit',
          hour12: false,
        })}
      </div>
    </div>
  )
}
