import { ReactNode } from 'react'
import { X } from 'lucide-react'
import DateTimeText from './DateTimeText'
import Button from './ui/button'

type Row = {
  weightment_id: number
  ingredient_name: string | null
  stock_item_id: string | null
  robot_status: string | null
  robot_error: string | null
  robot_requested_at: string | null
  robot_started_at: string | null
  robot_finished_at: string | null
  robot_trace_run_id: string | null
  robot_processed_id: number | null
  robot_processed_final_weight_kg: number | null
  robot_processed_pour_duration_s: number | null
  robot_processed_scoop_duration_s: number | null
  robot_processed_settle_time_s: number | null
  robot_processed_overshoot_g: number | null
  robot_mes_weighment_sent: boolean
}

type Props = {
  row: Row | null
  onClose: () => void
}

function Field({ label, children }: { label: string; children: ReactNode }) {
  return (
    <div>
      <div className="text-[10px] uppercase tracking-wider text-[var(--text-faint)]">{label}</div>
      <div className="mt-1 text-sm text-[var(--text-primary)]">{children}</div>
    </div>
  )
}

export default function RunDetailDrawer({ row, onClose }: Props) {
  if (!row) return null
  return (
    <div className="fixed inset-0 z-50 flex justify-end">
      <button type="button" className="absolute inset-0 bg-[var(--overlay-backdrop)]" onClick={onClose} />
      <aside className="relative z-10 flex h-full w-full max-w-lg flex-col border-l border-[var(--border)] bg-[var(--surface-1)] shadow-card-md">
        <div className="flex items-center justify-between border-b border-[var(--border)] px-5 py-4">
          <div>
            <h2 className="font-display text-lg font-semibold">Run Details</h2>
            <p className="text-xs text-[var(--text-muted)]">Weightment {row.weightment_id}</p>
          </div>
          <Button variant="ghost" size="sm" onClick={onClose}>
            <X className="h-4 w-4" />
          </Button>
        </div>
        <div className="flex-1 space-y-4 overflow-y-auto p-5">
          <div className="grid gap-4 sm:grid-cols-2">
            <Field label="Status">{row.robot_status ?? '—'}</Field>
            <Field label="Ingredient">{row.ingredient_name ?? row.stock_item_id ?? '—'}</Field>
            <Field label="Requested"><DateTimeText value={row.robot_requested_at} /></Field>
            <Field label="Started"><DateTimeText value={row.robot_started_at} /></Field>
            <Field label="Finished"><DateTimeText value={row.robot_finished_at} /></Field>
            <Field label="Trace">{row.robot_trace_run_id ?? '—'}</Field>
            <Field label="Processed ID">{row.robot_processed_id ?? '—'}</Field>
            <Field label="Final weight">
              {row.robot_processed_final_weight_kg != null
                ? `${row.robot_processed_final_weight_kg.toFixed(3)} kg`
                : '—'}
            </Field>
          </div>
          {(row.robot_processed_pour_duration_s != null ||
            row.robot_processed_scoop_duration_s != null) && (
            <div className="rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] p-3 text-sm">
              Scoop {row.robot_processed_scoop_duration_s?.toFixed(2) ?? '—'}s · Pour{' '}
              {row.robot_processed_pour_duration_s?.toFixed(2) ?? '—'}s · Settle{' '}
              {row.robot_processed_settle_time_s?.toFixed(2) ?? '—'}s · Error{' '}
              {row.robot_processed_overshoot_g?.toFixed(1) ?? '—'} g
            </div>
          )}
          {row.robot_error && (
            <div className="text-sm text-[var(--status-bad-fg)]">{row.robot_error}</div>
          )}
          <Field label="MES sent">{row.robot_mes_weighment_sent ? 'Yes' : 'No'}</Field>
        </div>
      </aside>
    </div>
  )
}
