import { cn } from '../../lib/cn'

type Props = {
  id?: string
  value: string
  onChange: (value: string) => void
  unit?: string
  min?: number
  max?: number
  step?: number
  placeholder?: string
  disabled?: boolean
  invalid?: boolean
  className?: string
}

export default function NumberInput({
  id,
  value,
  onChange,
  unit,
  min,
  max,
  step,
  placeholder,
  disabled = false,
  invalid = false,
  className,
}: Props) {
  return (
    <div
      className={cn(
        'flex items-center rounded-[var(--radius-sm)] border bg-[var(--surface-2)] focus-within:border-[var(--accent)]',
        invalid ? 'border-[var(--status-bad-fg)]' : 'border-[var(--border)]',
        className
      )}
    >
      <input
        id={id}
        type="text"
        inputMode="decimal"
        value={value}
        min={min}
        max={max}
        step={step}
        placeholder={placeholder}
        disabled={disabled}
        onChange={(e) => onChange(e.target.value)}
        className="w-full min-w-0 bg-transparent px-3 py-2 text-sm outline-none disabled:cursor-not-allowed disabled:opacity-50"
      />
      {unit ? (
        <span className="shrink-0 pr-3 text-xs text-[var(--text-faint)]">{unit}</span>
      ) : null}
    </div>
  )
}
