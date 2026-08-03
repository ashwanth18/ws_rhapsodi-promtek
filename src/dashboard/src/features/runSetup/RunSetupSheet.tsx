import { useCallback, useEffect, useMemo, useState } from 'react'
import { toast } from 'sonner'
import Button from '../../components/ui/button'
import Dialog from '../../components/ui/Dialog'
import Field from '../../components/ui/Field'
import Select from '../../components/ui/select'
import StatusBadge from '../../components/ui/StatusBadge'

const STORAGE_KEY = 'rhapsodi.lastRunSetup'

export type RunSetupMode = 'lightsout' | 'mock-local'

type Powder = {
  id: string
  name: string
  default_target_weight_g: number
  default_tolerance_g: number
  min_scooped_g: number
}

type SavedSetup = {
  powderId: string
  targetMode: 'fixed' | 'random_fraction' | 'stratified'
  targetWeightG: string
  fracMin: string
  fracMax: string
  targetMinG: string
  targetMaxG: string
  episodes: string
  cycles: string
  stopOn: 'episodes' | 'total_weight_g' | 'duration_min'
  stopValue: string
  batchId: string
  enableScoop: boolean
  lotCode: string
  operator: string
  notes: string
  toleranceG: string
  locationCode: string
}

const DEFAULTS: SavedSetup = {
  powderId: '',
  targetMode: 'stratified',
  targetWeightG: '250',
  fracMin: '0.4',
  fracMax: '0.9',
  targetMinG: '',
  targetMaxG: '',
  episodes: '10',
  cycles: '1',
  stopOn: 'episodes',
  stopValue: '0',
  batchId: '',
  enableScoop: true,
  lotCode: '',
  operator: '',
  notes: '',
  toleranceG: '',
  locationCode: '',
}

function loadSaved(): SavedSetup {
  try {
    const raw = localStorage.getItem(STORAGE_KEY)
    if (!raw) return DEFAULTS
    return { ...DEFAULTS, ...(JSON.parse(raw) as Partial<SavedSetup>) }
  } catch {
    return DEFAULTS
  }
}

function persistSaved(values: SavedSetup) {
  localStorage.setItem(STORAGE_KEY, JSON.stringify(values))
}

type Props = {
  mode: RunSetupMode
  apiBase: string
  open: boolean
  onClose: () => void
  onStarted?: () => void
}

export default function RunSetupSheet({
  mode,
  apiBase,
  open,
  onClose,
  onStarted,
}: Props) {
  const [powders, setPowders] = useState<Powder[]>([])
  const [loadingPowders, setLoadingPowders] = useState(true)
  const [submitting, setSubmitting] = useState(false)
  const [form, setForm] = useState<SavedSetup>(() => loadSaved())

  const selectedPowder = useMemo(
    () => powders.find((p) => p.id === form.powderId) ?? null,
    [powders, form.powderId]
  )

  const patch = (partial: Partial<SavedSetup>) => {
    setForm((current) => {
      const next = { ...current, ...partial }
      persistSaved(next)
      return next
    })
  }

  const loadPowders = useCallback(async () => {
    setLoadingPowders(true)
    try {
      const res = await fetch(`${apiBase}/powders`)
      if (!res.ok) throw new Error('Failed to load powders')
      const json = (await res.json()) as { powders: Powder[] }
      setPowders(json.powders || [])
      if (!form.powderId && json.powders?.[0]) {
        patch({ powderId: json.powders[0].id })
      }
    } catch (err) {
      toast.error(err instanceof Error ? err.message : 'Failed to load powders')
    } finally {
      setLoadingPowders(false)
    }
  }, [apiBase, form.powderId])

  useEffect(() => {
    if (open) void loadPowders()
  }, [open, loadPowders])

  useEffect(() => {
    if (selectedPowder && !form.targetWeightG) {
      patch({ targetWeightG: String(selectedPowder.default_target_weight_g) })
    }
  }, [selectedPowder, form.targetWeightG])

  const inputClass =
    'w-full rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2 text-sm outline-none focus:border-[var(--accent)]'

  const submit = async () => {
    if (!form.powderId) {
      toast.error('Select a powder')
      return
    }
    setSubmitting(true)
    try {
      if (mode === 'lightsout') {
        const body = {
          powder_id: form.powderId,
          target_weight_g: Number.parseFloat(form.targetWeightG),
          episodes: Number.parseInt(form.episodes, 10),
          batch_id: form.batchId.trim(),
          enable_scoop: form.enableScoop,
          lot_code: form.lotCode.trim(),
          operator: form.operator.trim(),
          notes: form.notes.trim(),
          stop_on: form.stopOn,
          stop_value: Number.parseFloat(form.stopValue || '0'),
          target_mode: form.targetMode,
          frac_min: Number.parseFloat(form.fracMin),
          frac_max: Number.parseFloat(form.fracMax),
          target_min_g: Number.parseFloat(form.targetMinG || '0'),
          target_max_g: Number.parseFloat(form.targetMaxG || '0'),
          min_scooped_g: selectedPowder?.min_scooped_g ?? 0,
        }
        const res = await fetch(`${apiBase}/modes/lightsout/runs`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(body),
        })
        const json = await res.json().catch(() => ({}))
        if (!res.ok) {
          const detail =
            typeof json.detail === 'string'
              ? json.detail
              : json.detail?.message || `Start failed (${res.status})`
          throw new Error(detail)
        }
        toast.success(json.message || 'Lights-out run accepted')
      } else {
        const target = Number.parseFloat(form.targetWeightG)
        const body: Record<string, unknown> = {
          powder_id: form.powderId,
          target_weight_g: target,
          cycles: Number.parseInt(form.cycles, 10) || 1,
          lot_code: form.lotCode.trim(),
          operator: form.operator.trim(),
          notes: form.notes.trim(),
        }
        if (form.toleranceG.trim()) {
          body.tolerance_g = Number.parseFloat(form.toleranceG)
        }
        if (form.locationCode.trim()) {
          body.location_code = form.locationCode.trim()
        }
        const res = await fetch(`${apiBase}/modes/mock/runs`, {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(body),
        })
        const json = await res.json().catch(() => ({}))
        if (!res.ok) {
          const detail =
            typeof json.detail === 'string'
              ? json.detail
              : json.detail?.message || `Start failed (${res.status})`
          throw new Error(detail)
        }
        toast.success(`Mock run started (weightment ${json.weightment_id})`)
      }
      onStarted?.()
      onClose()
    } catch (err) {
      toast.error(err instanceof Error ? err.message : 'Failed to start run')
    } finally {
      setSubmitting(false)
    }
  }

  return (
    <Dialog
      open={open}
      title={mode === 'lightsout' ? 'Lights-Out Run Setup' : 'Mock Run Setup'}
      onClose={onClose}
      className="max-w-2xl"
      footer={
        <>
          <Button variant="ghost" onClick={onClose} disabled={submitting}>
            Cancel
          </Button>
          <Button onClick={() => void submit()} disabled={submitting || loadingPowders}>
            {submitting ? 'Starting…' : 'Start run'}
          </Button>
        </>
      }
    >
      <div className="max-h-[70vh] space-y-5 overflow-y-auto pr-1">
        <section className="space-y-3">
          <h3 className="text-xs font-semibold uppercase tracking-wider text-[var(--text-faint)]">
            Powder
          </h3>
          <Field label="Powder">
            {(id) => (
              <Select
                id={id}
                value={form.powderId}
                onChange={(e) => patch({ powderId: e.target.value })}
                disabled={loadingPowders}
                className="w-full"
              >
                <option value="">Select powder…</option>
                {powders.map((p) => (
                  <option key={p.id} value={p.id}>
                    {p.name} ({p.id})
                  </option>
                ))}
              </Select>
            )}
          </Field>
        </section>

        {mode === 'lightsout' && (
          <section className="space-y-3">
            <h3 className="text-xs font-semibold uppercase tracking-wider text-[var(--text-faint)]">
              Target strategy
            </h3>
            <Field label="Target mode">
              {(id) => (
                <Select
                  id={id}
                  value={form.targetMode}
                  onChange={(e) =>
                    patch({
                      targetMode: e.target.value as SavedSetup['targetMode'],
                    })
                  }
                  className="w-full"
                >
                  <option value="fixed">Fixed weight</option>
                  <option value="random_fraction">Random fraction of scoop</option>
                  <option value="stratified">Stratified fractions</option>
                </Select>
              )}
            </Field>
            {form.targetMode !== 'fixed' && (
              <div className="grid gap-3 sm:grid-cols-2">
                <Field label="Fraction min">
                  {(id) => (
                    <input
                      id={id}
                      value={form.fracMin}
                      onChange={(e) => patch({ fracMin: e.target.value })}
                      className={inputClass}
                      inputMode="decimal"
                    />
                  )}
                </Field>
                <Field label="Fraction max">
                  {(id) => (
                    <input
                      id={id}
                      value={form.fracMax}
                      onChange={(e) => patch({ fracMax: e.target.value })}
                      className={inputClass}
                      inputMode="decimal"
                    />
                  )}
                </Field>
              </div>
            )}
          </section>
        )}

        <section className="space-y-3">
          <h3 className="text-xs font-semibold uppercase tracking-wider text-[var(--text-faint)]">
            Targets
          </h3>
          <div className="grid gap-3 sm:grid-cols-2">
            <Field label="Target weight (g)">
              {(id) => (
                <input
                  id={id}
                  value={form.targetWeightG}
                  onChange={(e) => patch({ targetWeightG: e.target.value })}
                  className={inputClass}
                  inputMode="decimal"
                />
              )}
            </Field>
            {mode === 'mock-local' && (
              <Field label="Tolerance (g, optional)">
                {(id) => (
                  <input
                    id={id}
                    value={form.toleranceG}
                    onChange={(e) => patch({ toleranceG: e.target.value })}
                    className={inputClass}
                    inputMode="decimal"
                  />
                )}
              </Field>
            )}
            {mode === 'mock-local' && (
              <Field label="Location code (optional)" className="sm:col-span-2">
                {(id) => (
                  <input
                    id={id}
                    value={form.locationCode}
                    onChange={(e) => patch({ locationCode: e.target.value })}
                    className={inputClass}
                  />
                )}
              </Field>
            )}
          </div>
        </section>

        <section className="space-y-3">
          <h3 className="text-xs font-semibold uppercase tracking-wider text-[var(--text-faint)]">
            {mode === 'lightsout' ? 'Episodes & stop' : 'Cycles'}
          </h3>
          {mode === 'lightsout' ? (
            <div className="grid gap-3 sm:grid-cols-3">
              <Field label="Episodes">
                {(id) => (
                  <input
                    id={id}
                    value={form.episodes}
                    onChange={(e) => patch({ episodes: e.target.value })}
                    className={inputClass}
                    inputMode="numeric"
                  />
                )}
              </Field>
              <Field label="Stop on">
                {(id) => (
                  <Select
                    id={id}
                    value={form.stopOn}
                    onChange={(e) =>
                      patch({ stopOn: e.target.value as SavedSetup['stopOn'] })
                    }
                    className="w-full"
                  >
                    <option value="episodes">Episodes</option>
                    <option value="total_weight_g">Total weight (g)</option>
                    <option value="duration_min">Duration (min)</option>
                  </Select>
                )}
              </Field>
              <Field label="Stop value">
                {(id) => (
                  <input
                    id={id}
                    value={form.stopValue}
                    onChange={(e) => patch({ stopValue: e.target.value })}
                    className={inputClass}
                    inputMode="decimal"
                  />
                )}
              </Field>
              <Field label="Batch ID" className="sm:col-span-2">
                {(id) => (
                  <input
                    id={id}
                    value={form.batchId}
                    onChange={(e) => patch({ batchId: e.target.value })}
                    className={inputClass}
                  />
                )}
              </Field>
              <label className="flex items-center gap-2 text-sm text-[var(--text-secondary)] sm:col-span-3">
                <input
                  type="checkbox"
                  checked={form.enableScoop}
                  onChange={(e) => patch({ enableScoop: e.target.checked })}
                  className="rounded border-[var(--border)]"
                />
                Enable ExecuteScoop
              </label>
            </div>
          ) : (
            <Field label="Cycles">
              {(id) => (
                <input
                  id={id}
                  value={form.cycles}
                  onChange={(e) => patch({ cycles: e.target.value })}
                  className={inputClass}
                  inputMode="numeric"
                />
              )}
            </Field>
          )}
        </section>

        <section className="space-y-3">
          <h3 className="text-xs font-semibold uppercase tracking-wider text-[var(--text-faint)]">
            Labels
          </h3>
          <div className="grid gap-3 sm:grid-cols-2">
            <Field label="Lot code">
              {(id) => (
                <input
                  id={id}
                  value={form.lotCode}
                  onChange={(e) => patch({ lotCode: e.target.value })}
                  className={inputClass}
                />
              )}
            </Field>
            <Field label="Operator">
              {(id) => (
                <input
                  id={id}
                  value={form.operator}
                  onChange={(e) => patch({ operator: e.target.value })}
                  className={inputClass}
                />
              )}
            </Field>
            <Field label="Notes" className="sm:col-span-2">
              {(id) => (
                <textarea
                  id={id}
                  value={form.notes}
                  onChange={(e) => patch({ notes: e.target.value })}
                  rows={2}
                  className={inputClass}
                />
              )}
            </Field>
          </div>
        </section>

        <section className="rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-elevated)] p-3 text-sm">
          <h3 className="mb-2 font-semibold">Review</h3>
          <div className="space-y-1 text-[var(--text-secondary)]">
            <div>
              Powder:{' '}
              <StatusBadge
                label={selectedPowder?.name || form.powderId || '—'}
                tone="info"
              />
            </div>
            <div>
              Mode: <span className="font-mono text-xs">{mode}</span>
            </div>
            <div>Target: {form.targetWeightG} g</div>
            {mode === 'lightsout' ? (
              <>
                <div>
                  Strategy: {form.targetMode} · {form.episodes} episodes · stop{' '}
                  {form.stopOn}={form.stopValue || '0'}
                </div>
                <div>
                  Labels: lot {form.lotCode || '—'} · op {form.operator || '—'}
                </div>
              </>
            ) : (
              <div>
                {form.cycles} cycle(s) · lot {form.lotCode || '—'} · op{' '}
                {form.operator || '—'}
              </div>
            )}
          </div>
        </section>
      </div>
    </Dialog>
  )
}
