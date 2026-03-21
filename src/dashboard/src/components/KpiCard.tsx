import GlassCard from './GlassCard'

type Props = {
  label: string
  value: string | number | null | undefined
  help?: string
}

export default function KpiCard({ label, value, help }: Props) {
  return (
    <GlassCard>
      <div className="flex flex-col gap-2">
        <span className="inline-flex rounded-md bg-[var(--status-info-bg)] px-2 py-0.5 text-xs text-[var(--status-info-fg)]">
          {label}
        </span>
        <span className="text-3xl font-extrabold bg-gradient-to-r from-primary-blue400 to-primary-sky400 bg-clip-text text-transparent leading-none">
          {value ?? '—'}
        </span>
        {help ? <span className="text-xs text-[var(--text-muted)]">{help}</span> : null}
      </div>
    </GlassCard>
  )
}


