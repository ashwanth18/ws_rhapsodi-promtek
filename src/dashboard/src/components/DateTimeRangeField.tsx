import Button from './ui/button'

type Props = {
  from: string
  to: string
  onFromChange: (value: string) => void
  onToChange: (value: string) => void
  onClear: () => void
}

function ClockIcon() {
  return (
    <svg
      className="h-4 w-4 text-[var(--text-muted)]"
      viewBox="0 0 24 24"
      fill="none"
      xmlns="http://www.w3.org/2000/svg"
      aria-hidden="true"
    >
      <circle cx="12" cy="12" r="8" stroke="currentColor" strokeWidth="2" />
      <path
        d="M12 8V12L14.5 14.5"
        stroke="currentColor"
        strokeWidth="2"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
    </svg>
  )
}

export default function DateTimeRangeField({
  from,
  to,
  onFromChange,
  onToChange,
  onClear,
}: Props) {
  return (
    <div className="flex shrink-0 items-end gap-2 rounded-xl border border-[var(--border)] bg-[var(--surface)]/70 px-3 py-2 shadow-sm">
      <div className="flex h-9 items-center gap-2 pr-1">
        <ClockIcon />
        <span className="text-[10px] font-semibold uppercase tracking-[0.14em] text-[var(--text-faint)]">
          Date Range
        </span>
      </div>
      <label className="flex shrink-0 items-center gap-2">
        <span className="text-[10px] font-semibold uppercase tracking-[0.12em] text-[var(--text-faint)]">
          From
        </span>
        <input
          type="date"
          className="h-9 rounded-lg border border-[var(--border)] bg-[var(--surface-elevated)] px-3 text-sm text-[var(--text-primary)] outline-none transition-colors focus:border-cyan-400 focus:ring-2 focus:ring-cyan-400/30"
          value={from}
          onChange={(event) => onFromChange(event.target.value)}
        />
      </label>
      <label className="flex shrink-0 items-center gap-2">
        <span className="text-[10px] font-semibold uppercase tracking-[0.12em] text-[var(--text-faint)]">
          To
        </span>
        <input
          type="date"
          className="h-9 rounded-lg border border-[var(--border)] bg-[var(--surface-elevated)] px-3 text-sm text-[var(--text-primary)] outline-none transition-colors focus:border-cyan-400 focus:ring-2 focus:ring-cyan-400/30"
          value={to}
          onChange={(event) => onToChange(event.target.value)}
        />
      </label>
      {(from || to) && (
        <Button size="sm" variant="ghost" onClick={onClear} className="h-9 shrink-0 px-2 py-1 text-xs">
          Clear
        </Button>
      )}
    </div>
  )
}
