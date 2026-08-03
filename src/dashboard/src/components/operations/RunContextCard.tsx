import { ReactNode } from 'react'
import DateTimeText from '../DateTimeText'
import GlassCard from '../GlassCard'
import StatusBadge from '../ui/StatusBadge'
import { statusLabel } from './operationsUtils'

type RunRow = {
  trace_run_id: string | null
  weightment_id: number
  batch_id: string | null
  ingredient_name: string | null
  ingredient_id: string | null
  stock_location_code: string | null
  status: string | null
  requested_at: string | null
  started_at: string | null
  finished_at: string | null
  mes_weighment_sent: boolean
  mes_batch_end_sent: boolean
}

type Props = {
  run: RunRow | null
  loading: boolean
  metadataBatchId?: string | null
  mesSinkDisabled?: boolean
}

function Field({ label, children }: { label: string; children: React.ReactNode }) {
  return (
    <div>
      <div className="text-[10px] uppercase tracking-wider text-[var(--text-faint)]">{label}</div>
      <div className="mt-1 text-sm text-[var(--text-primary)]">{children}</div>
    </div>
  )
}

export default function RunContextCard({
  run,
  loading,
  metadataBatchId,
  mesSinkDisabled = false,
}: Props) {
  return (
    <GlassCard className="h-full">
      <div className="mb-4 flex items-center justify-between">
        <h3 className="font-display text-sm font-semibold uppercase tracking-wider text-[var(--text-faint)]">
          Run Context
        </h3>
        <span className="text-xs text-[var(--text-muted)]">
          {loading ? 'Refreshing…' : 'Auto 5s'}
        </span>
      </div>
      {!run ? (
        <p className="text-sm text-[var(--text-muted)]">No robot run recorded yet.</p>
      ) : (
        <>
          <div className="mb-4 flex flex-wrap gap-2">
            <StatusBadge
              label={statusLabel(run.status)}
              tone={
                run.status === 'succeeded'
                  ? 'good'
                  : run.status === 'running' || run.status === 'starting'
                    ? 'warn'
                    : run.status === 'failed'
                      ? 'bad'
                      : 'neutral'
              }
              pulse={run.status === 'running' || run.status === 'starting'}
            />
            <StatusBadge
              label={
                mesSinkDisabled
                  ? 'MES N/A'
                  : run.mes_weighment_sent
                    ? 'MES sent'
                    : 'MES pending'
              }
              tone={
                mesSinkDisabled
                  ? 'neutral'
                  : run.mes_weighment_sent
                    ? 'good'
                    : 'idle'
              }
            />
          </div>
          <div className="grid gap-4 sm:grid-cols-2 lg:grid-cols-3">
            <Field label="Trace">{run.trace_run_id || '—'}</Field>
            <Field label="Weightment">{run.weightment_id}</Field>
            <Field label="Batch">{run.batch_id || metadataBatchId || '—'}</Field>
            <Field label="Ingredient">{run.ingredient_name || run.ingredient_id || '—'}</Field>
            <Field label="Location">{run.stock_location_code || '—'}</Field>
            <Field label="Requested"><DateTimeText value={run.requested_at} /></Field>
            <Field label="Started"><DateTimeText value={run.started_at} /></Field>
            <Field label="Finished"><DateTimeText value={run.finished_at} /></Field>
          </div>
        </>
      )}
    </GlassCard>
  )
}
