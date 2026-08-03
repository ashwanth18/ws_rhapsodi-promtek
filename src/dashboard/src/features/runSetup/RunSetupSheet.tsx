import { useCallback, useEffect, useMemo, useState } from 'react'
import { toast } from 'sonner'
import Button from '../../components/ui/button'
import Dialog from '../../components/ui/Dialog'
import Field from '../../components/ui/Field'
import NumberInput from '../../components/ui/NumberInput'
import SegmentedControl from '../../components/ui/SegmentedControl'
import Select from '../../components/ui/select'
import StatusBadge from '../../components/ui/StatusBadge'

export type RunSetupMode = 'lightsout' | 'mock-local'

type Powder = {
  id: string
  name: string
  container_target: string
  pour_target: string
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
  targetWeightG: '',
  fracMin: '0.4',
  fracMax: '0.9',
  episodes: '10',
  cycles: '1',
  stopOn: 'episodes',
  stopValue: '',
  batchId: '',
  enableScoop: true,
  lotCode: '',
  operator: '',
  notes: '',
  toleranceG: '',
  locationCode: '',
}

function storageKey(mode: RunSetupMode) {
  return `rhapsodi.lastRunSetup.${mode}`
}

function loadSaved(mode: RunSetupMode): SavedSetup {
  try {
    const raw = localStorage.getItem(storageKey(mode))
    if (!raw) return { ...DEFAULTS }
    return { ...DEFAULTS, ...(JSON.parse(raw) as Partial<SavedSetup>) }
  } catch {
    return { ...DEFAULTS }
  }
}

function persistSaved(mode: RunSetupMode, values: SavedSetup) {
  localStorage.setItem(storageKey(mode), JSON.stringify(values))
}

function parseFinite(raw: string): number | null {
  const n = Number.parseFloat(raw)
  return Number.isFinite(n) ? n : null
}

function parseIntFinite(raw: string): number | null {
  const n = Number.parseInt(raw, 10)
  return Number.isFinite(n) ? n : null
}

function validate(form: SavedSetup, mode: RunSetupMode): Record<string, string> {
  const errors: Record<string, string> = {}
  if (!form.powderId.trim()) errors.powderId = 'Select a powder'

  if (mode === 'lightsout') {
    const episodes = parseIntFinite(form.episodes)
    if (episodes == null || episodes < 1) errors.episodes = 'Enter at least 1 episode'

    if (form.targetMode === 'fixed') {
      const target = parseFinite(form.targetWeightG)
      if (target == null || target <= 0) errors.targetWeightG = 'Enter a pour target > 0'
    } else {
      const lo = parseFinite(form.fracMin)
      const hi = parseFinite(form.fracMax)
      if (lo == null || hi == null) {
        errors.fracMin = 'Enter valid fractions'
      } else if (lo < 0 || hi > 1 || lo > hi) {
        errors.fracMin = 'Require 0 ≤ min ≤ max ≤ 1'
        errors.fracMax = 'Require 0 ≤ min ≤ max ≤ 1'
      }
    }

    if (form.stopOn !== 'episodes') {
      const stop = parseFinite(form.stopValue)
      if (stop == null || stop <= 0) {
        errors.stopValue =
          form.stopOn === 'duration_min'
            ? 'Enter a duration > 0'
            : 'Enter a weight > 0'
      }
    }
  } else {
    const cycles = parseIntFinite(form.cycles)
    if (cycles == null || cycles < 1) errors.cycles = 'Enter at least 1 cycle'
    if (form.toleranceG.trim()) {
      const tol = parseFinite(form.toleranceG)
      if (tol == null || tol < 0) errors.toleranceG = 'Tolerance must be ≥ 0'
    }
    const target = parseFinite(form.targetWeightG)
    if (target == null || target <= 0) errors.targetWeightG = 'Enter a target > 0'
  }

  return errors
}

function planSentence(form: SavedSetup, mode: RunSetupMode, powderName: string): string {
  if (mode === 'mock-local') {
    const cycles = form.cycles || '1'
    return `${cycles} cycle(s) at ${form.targetWeightG || '—'} g${
      powderName ? ` · ${powderName}` : ''
    }.`
  }

  const eps = form.episodes || '—'
  let strategy = ''
  if (form.targetMode === 'fixed') {
    strategy = `pouring ${form.targetWeightG || '—'} g each`
  } else if (form.targetMode === 'random_fraction') {
    strategy = `pouring a random ${form.fracMin}-${form.fracMax} × each scoop`
  } else {
    strategy = `pouring ${form.fracMin}-${form.fracMax} × each scoop`
  }

  let stop = ''
  if (form.stopOn === 'episodes') {
    stop = `Stops after ${eps} episodes.`
    return `${eps} episodes, ${strategy}. ${stop}`
  }
  if (form.stopOn === 'duration_min') {
    stop = `Stops after ${form.stopValue || '—'} min.`
    return `Up to ${eps} episodes, ${strategy}. ${stop}`
  }
  stop = `Stops after ${form.stopValue || '—'} g poured.`
  return `Up to ${eps} episodes, ${strategy}. ${stop}`
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
  const [allocatingBatch, setAllocatingBatch] = useState(false)
  const [form, setForm] = useState<SavedSetup>(() => loadSaved(mode))
  const [touchedSubmit, setTouchedSubmit] = useState(false)

  useEffect(() => {
    if (open) {
      setForm(loadSaved(mode))
      setTouchedSubmit(false)
    }
  }, [open, mode])

  const selectedPowder = useMemo(
    () => powders.find((p) => p.id === form.powderId) ?? null,
    [powders, form.powderId]
  )

  const errors = useMemo(() => validate(form, mode), [form, mode])
  const hasErrors = Object.keys(errors).length > 0
  const showError = (key: string) => (touchedSubmit ? errors[key] : undefined)

  const patch = (partial: Partial<SavedSetup>) => {
    setForm((current) => {
      const next = { ...current, ...partial }
      persistSaved(mode, next)
      return next
    })
  }

  const allocateBatchId = useCallback(
    async (quiet = false) => {
      setAllocatingBatch(true)
      try {
        const res = await fetch(
          `${apiBase}/batch_ids/next?mode=${encodeURIComponent(mode)}`
        )
        const json = await res.json().catch(() => ({}))
        if (!res.ok) {
          throw new Error(
            typeof json.detail === 'string' ? json.detail : 'Failed to allocate batch id'
          )
        }
        if (json.batch_id) patch({ batchId: String(json.batch_id) })
      } catch (err) {
        if (!quiet) {
          toast.error(err instanceof Error ? err.message : 'Failed to allocate batch id')
        }
      } finally {
        setAllocatingBatch(false)
      }
    },
    [apiBase, mode]
  )

  const loadPowders = useCallback(async () => {
    setLoadingPowders(true)
    try {
      const res = await fetch(`${apiBase}/powders`)
      if (!res.ok) throw new Error('Failed to load powders')
      const json = (await res.json()) as { powders: Powder[] }
      setPowders(json.powders || [])
      setForm((current) => {
        if (current.powderId || !json.powders?.[0]) return current
        const next = { ...current, powderId: json.powders[0].id }
        persistSaved(mode, next)
        return next
      })
    } catch (err) {
      toast.error(err instanceof Error ? err.message : 'Failed to load powders')
    } finally {
      setLoadingPowders(false)
    }
  }, [apiBase, mode])

  useEffect(() => {
    if (!open) return
    void loadPowders()
  }, [open, loadPowders])

  useEffect(() => {
    if (!open || mode !== 'lightsout') return
    if (form.batchId.trim()) return
    void allocateBatchId(true)
    // Only on open / empty — not on every form change.
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [open, mode])

  useEffect(() => {
    if (!selectedPowder) return
    setForm((current) => {
      let next = current
      let changed = false
      if (!current.targetWeightG.trim()) {
        next = {
          ...next,
          targetWeightG: String(selectedPowder.default_target_weight_g),
        }
        changed = true
      }
      if (mode === 'mock-local' && !current.toleranceG.trim()) {
        next = {
          ...next,
          toleranceG: String(selectedPowder.default_tolerance_g),
        }
        changed = true
      }
      if (changed) persistSaved(mode, next)
      return changed ? next : current
    })
  }, [selectedPowder, mode])

  const submit = async () => {
    setTouchedSubmit(true)
    const currentErrors = validate(form, mode)
    if (Object.keys(currentErrors).length > 0) {
      toast.error('Fix the highlighted fields before starting')
      return
    }
    setSubmitting(true)
    try {
      if (mode === 'lightsout') {
        const body: Record<string, unknown> = {
          powder_id: form.powderId,
          episodes: parseIntFinite(form.episodes),
          batch_id: form.batchId.trim(),
          enable_scoop: form.enableScoop,
          lot_code: form.lotCode.trim(),
          operator: form.operator.trim(),
          notes: form.notes.trim(),
          stop_on: form.stopOn,
          stop_value:
            form.stopOn === 'episodes' ? 0 : parseFinite(form.stopValue) ?? 0,
          target_mode: form.targetMode,
          frac_min: parseFinite(form.fracMin) ?? 0.4,
          frac_max: parseFinite(form.fracMax) ?? 0.9,
          min_scooped_g: selectedPowder?.min_scooped_g ?? 0,
        }
        if (form.targetMode === 'fixed') {
          body.target_weight_g = parseFinite(form.targetWeightG)
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
        const body: Record<string, unknown> = {
          powder_id: form.powderId,
          target_weight_g: parseFinite(form.targetWeightG),
          cycles: parseIntFinite(form.cycles) || 1,
          lot_code: form.lotCode.trim(),
          operator: form.operator.trim(),
          notes: form.notes.trim(),
        }
        if (form.toleranceG.trim()) {
          body.tolerance_g = parseFinite(form.toleranceG)
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

  const preview = planSentence(form, mode, selectedPowder?.name || '')

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
          <Button
            onClick={() => void submit()}
            disabled={submitting || loadingPowders || (touchedSubmit && hasErrors)}
          >
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
          <Field label="Powder" error={showError('powderId')}>
            {(id) => (
              <Select
                id={id}
                value={form.powderId}
                onChange={(e) => {
                  const powder = powders.find((p) => p.id === e.target.value)
                  patch({
                    powderId: e.target.value,
                    targetWeightG: powder
                      ? String(powder.default_target_weight_g)
                      : form.targetWeightG,
                    ...(mode === 'mock-local' && powder
                      ? { toleranceG: String(powder.default_tolerance_g) }
                      : {}),
                  })
                }}
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
          {selectedPowder && (
            <p className="text-xs text-[var(--text-faint)]">
              Container{' '}
              <span className="font-mono text-[var(--text-secondary)]">
                {selectedPowder.container_target}
              </span>{' '}
              → pour{' '}
              <span className="font-mono text-[var(--text-secondary)]">
                {selectedPowder.pour_target}
              </span>{' '}
              · min scoop {selectedPowder.min_scooped_g} g
            </p>
          )}
        </section>

        {mode === 'lightsout' && (
          <section className="space-y-3">
            <h3 className="text-xs font-semibold uppercase tracking-wider text-[var(--text-faint)]">
              Strategy
            </h3>
            <Field label="Target mode" hint="How each episode's pour target is chosen.">
              {() => (
                <SegmentedControl
                  ariaLabel="Target mode"
                  value={form.targetMode}
                  onChange={(targetMode) => patch({ targetMode })}
                  segments={[
                    {
                      value: 'fixed',
                      label: 'Fixed weight',
                      hint: 'Same grams every episode',
                    },
                    {
                      value: 'random_fraction',
                      label: 'Random fraction',
                      hint: 'Random share of each scoop',
                    },
                    {
                      value: 'stratified',
                      label: 'Stratified',
                      hint: 'Even coverage across the range',
                    },
                  ]}
                />
              )}
            </Field>
            {form.targetMode === 'fixed' ? (
              <Field
                label="Pour target"
                hint="Absolute grams poured back each episode."
                error={showError('targetWeightG')}
              >
                {(id) => (
                  <NumberInput
                    id={id}
                    value={form.targetWeightG}
                    onChange={(targetWeightG) => patch({ targetWeightG })}
                    unit="g"
                    invalid={Boolean(showError('targetWeightG'))}
                  />
                )}
              </Field>
            ) : (
              <div className="grid gap-3 sm:grid-cols-2">
                <Field
                  label="Fraction min"
                  hint="Share of the measured scoop, 0 to 1."
                  error={showError('fracMin')}
                >
                  {(id) => (
                    <NumberInput
                      id={id}
                      value={form.fracMin}
                      onChange={(fracMin) => patch({ fracMin })}
                      step={0.05}
                      min={0}
                      max={1}
                      invalid={Boolean(showError('fracMin'))}
                    />
                  )}
                </Field>
                <Field label="Fraction max" error={showError('fracMax')}>
                  {(id) => (
                    <NumberInput
                      id={id}
                      value={form.fracMax}
                      onChange={(fracMax) => patch({ fracMax })}
                      step={0.05}
                      min={0}
                      max={1}
                      invalid={Boolean(showError('fracMax'))}
                    />
                  )}
                </Field>
              </div>
            )}
          </section>
        )}

        <section className="space-y-3">
          <h3 className="text-xs font-semibold uppercase tracking-wider text-[var(--text-faint)]">
            {mode === 'lightsout' ? 'Stop condition' : 'Cycles'}
          </h3>
          {mode === 'lightsout' ? (
            <>
              <Field label="Stop on">
                {() => (
                  <SegmentedControl
                    ariaLabel="Stop condition"
                    value={form.stopOn}
                    onChange={(stopOn) => patch({ stopOn })}
                    segments={[
                      { value: 'episodes', label: 'Episodes' },
                      { value: 'total_weight_g', label: 'Total weight' },
                      { value: 'duration_min', label: 'Duration' },
                    ]}
                  />
                )}
              </Field>
              {form.stopOn === 'episodes' ? (
                <Field
                  label="Episodes"
                  hint="Number of scoop–pour cycles."
                  error={showError('episodes')}
                >
                  {(id) => (
                    <NumberInput
                      id={id}
                      value={form.episodes}
                      onChange={(episodes) => patch({ episodes })}
                      invalid={Boolean(showError('episodes'))}
                    />
                  )}
                </Field>
              ) : (
                <div className="grid gap-3 sm:grid-cols-2">
                  <Field
                    label="Stop after"
                    error={showError('stopValue')}
                    hint={
                      form.stopOn === 'duration_min'
                        ? 'Checked between episodes (never mid-pour).'
                        : 'Cumulative mass poured back into the vessel.'
                    }
                  >
                    {(id) => (
                      <NumberInput
                        id={id}
                        value={form.stopValue}
                        onChange={(stopValue) => patch({ stopValue })}
                        unit={form.stopOn === 'duration_min' ? 'min' : 'g'}
                        invalid={Boolean(showError('stopValue'))}
                      />
                    )}
                  </Field>
                  <Field
                    label="Max episodes"
                    hint="Safety cap. The run stops at whichever comes first."
                    error={showError('episodes')}
                  >
                    {(id) => (
                      <NumberInput
                        id={id}
                        value={form.episodes}
                        onChange={(episodes) => patch({ episodes })}
                        invalid={Boolean(showError('episodes'))}
                      />
                    )}
                  </Field>
                </div>
              )}
            </>
          ) : (
            <div className="grid gap-3 sm:grid-cols-2">
              <Field label="Target weight" error={showError('targetWeightG')}>
                {(id) => (
                  <NumberInput
                    id={id}
                    value={form.targetWeightG}
                    onChange={(targetWeightG) => patch({ targetWeightG })}
                    unit="g"
                    invalid={Boolean(showError('targetWeightG'))}
                  />
                )}
              </Field>
              <Field label="Cycles" error={showError('cycles')}>
                {(id) => (
                  <NumberInput
                    id={id}
                    value={form.cycles}
                    onChange={(cycles) => patch({ cycles })}
                    invalid={Boolean(showError('cycles'))}
                  />
                )}
              </Field>
              <Field
                label="Tolerance"
                hint="Blank uses about 2% of target."
                error={showError('toleranceG')}
              >
                {(id) => (
                  <NumberInput
                    id={id}
                    value={form.toleranceG}
                    onChange={(toleranceG) => patch({ toleranceG })}
                    unit="g"
                    invalid={Boolean(showError('toleranceG'))}
                  />
                )}
              </Field>
              <Field
                label="Location code"
                hint="Blank uses the MOCK targets."
              >
                {(id) => (
                  <input
                    id={id}
                    value={form.locationCode}
                    onChange={(e) => patch({ locationCode: e.target.value })}
                    className="w-full rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2 text-sm outline-none focus:border-[var(--accent)]"
                  />
                )}
              </Field>
            </div>
          )}
        </section>

        <section className="space-y-3">
          <h3 className="text-xs font-semibold uppercase tracking-wider text-[var(--text-faint)]">
            Labels
          </h3>
          <div className="grid gap-3 sm:grid-cols-2">
            {mode === 'lightsout' && (
              <Field
                label="Session label"
                hint="Groups episodes in History and Export."
                className="sm:col-span-2"
              >
                {(id) => (
                  <div className="flex gap-2">
                    <input
                      id={id}
                      value={form.batchId}
                      onChange={(e) => patch({ batchId: e.target.value })}
                      className="w-full rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2 text-sm outline-none focus:border-[var(--accent)]"
                    />
                    <Button
                      type="button"
                      variant="outline"
                      size="sm"
                      disabled={allocatingBatch}
                      onClick={() => void allocateBatchId(false)}
                    >
                      {allocatingBatch ? '…' : 'Auto'}
                    </Button>
                  </div>
                )}
              </Field>
            )}
            <Field label="Lot code">
              {(id) => (
                <input
                  id={id}
                  value={form.lotCode}
                  onChange={(e) => patch({ lotCode: e.target.value })}
                  className="w-full rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2 text-sm outline-none focus:border-[var(--accent)]"
                />
              )}
            </Field>
            <Field label="Operator">
              {(id) => (
                <input
                  id={id}
                  value={form.operator}
                  onChange={(e) => patch({ operator: e.target.value })}
                  className="w-full rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2 text-sm outline-none focus:border-[var(--accent)]"
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
                  className="w-full rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2 text-sm outline-none focus:border-[var(--accent)]"
                />
              )}
            </Field>
          </div>
          {mode === 'lightsout' && (
            <label className="flex items-start gap-2 text-sm text-[var(--text-secondary)]">
              <input
                type="checkbox"
                checked={form.enableScoop}
                onChange={(e) => patch({ enableScoop: e.target.checked })}
                className="mt-0.5 rounded border-[var(--border)]"
              />
              <span>
                Perform scoop
                <span className="mt-0.5 block text-xs text-[var(--text-faint)]">
                  Uncheck to reuse powder already in the scoop.
                </span>
              </span>
            </label>
          )}
        </section>

        <section className="rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] p-3 text-sm">
          <h3 className="mb-2 text-xs font-semibold uppercase tracking-wider text-[var(--text-faint)]">
            Plan
          </h3>
          <p className="text-[var(--text-secondary)]">{preview}</p>
          <div className="mt-2 flex flex-wrap items-center gap-2 text-xs text-[var(--text-faint)]">
            <StatusBadge
              label={selectedPowder?.name || form.powderId || 'No powder'}
              tone="info"
            />
            {mode === 'lightsout' && form.batchId ? (
              <span className="font-mono">{form.batchId}</span>
            ) : null}
          </div>
        </section>
      </div>
    </Dialog>
  )
}
