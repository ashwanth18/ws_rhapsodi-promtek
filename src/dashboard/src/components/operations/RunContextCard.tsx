import { ReactNode } from 'react'
import GlassCard from '../GlassCard'

export type RunContextField = { label: string; value: ReactNode }

type Props = {
  title?: string
  badges?: ReactNode
  fields: RunContextField[]
  loading?: boolean
  empty?: string
}

function Field({ label, children }: { label: string; children: ReactNode }) {
  return (
    <div>
      <div className="text-[10px] uppercase tracking-wider text-[var(--text-faint)]">{label}</div>
      <div className="mt-1 text-sm text-[var(--text-primary)]">{children}</div>
    </div>
  )
}

export default function RunContextCard({
  title = 'Run Context',
  badges,
  fields,
  loading = false,
  empty = 'No run context yet.',
}: Props) {
  return (
    <GlassCard className="h-full">
      <div className="mb-4 flex items-center justify-between">
        <h3 className="font-display text-sm font-semibold uppercase tracking-wider text-[var(--text-faint)]">
          {title}
        </h3>
        <span className="text-xs text-[var(--text-muted)]">
          {loading ? 'Refreshing…' : 'Live'}
        </span>
      </div>
      {fields.length === 0 ? (
        <p className="text-sm text-[var(--text-muted)]">{empty}</p>
      ) : (
        <>
          {badges && <div className="mb-4 flex flex-wrap gap-2">{badges}</div>}
          <div className="grid gap-4 sm:grid-cols-2 lg:grid-cols-3">
            {fields.map((field) => (
              <Field key={field.label} label={field.label}>
                {field.value}
              </Field>
            ))}
          </div>
        </>
      )}
    </GlassCard>
  )
}
