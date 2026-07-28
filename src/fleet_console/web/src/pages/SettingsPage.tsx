import { useEffect, useState, type ChangeEvent, type ReactNode } from 'react'
import { PlugZap, Save } from 'lucide-react'
import {
  api,
  setToken,
  type ConsoleSettings,
  type SettingsCheck,
  type SettingsTestResult,
} from '../lib/api'
import { Button, SectionHeader, StatusBadge } from '../components/ui'

type FormState = {
  fleet_console_url: string
  image_registry: string
  github_repo: string
  prometheus_url: string
  loki_url: string
  alertmanager_url: string
  grafana_pi_overview_url: string
  fleet_api_token: string
  ci_report_token: string
  github_token: string
}

const EMPTY: FormState = {
  fleet_console_url: '',
  image_registry: '',
  github_repo: '',
  prometheus_url: '',
  loki_url: '',
  alertmanager_url: '',
  grafana_pi_overview_url: '',
  fleet_api_token: '',
  ci_report_token: '',
  github_token: '',
}

const CHECK_LABELS: Record<string, string> = {
  github: 'GitHub',
  prometheus: 'Prometheus',
  loki: 'Loki',
  alertmanager: 'Alertmanager',
  grafana: 'Grafana',
  fleet_console_url: 'Console URL',
  image_registry: 'Image registry',
  fleet_api_token: 'Operator token',
  ci_report_token: 'CI report token',
}

function Field({
  label,
  help,
  check,
  children,
}: {
  label: string
  help?: string
  check?: SettingsCheck
  children: ReactNode
}) {
  return (
    <label className="block space-y-1.5">
      <div className="flex flex-wrap items-center gap-2">
        <div className="text-sm font-medium text-[var(--text-primary)]">{label}</div>
        {check ? (
          <StatusBadge
            label={check.ok ? 'Connected' : 'Failed'}
            tone={check.ok ? 'good' : 'bad'}
          />
        ) : null}
      </div>
      {children}
      {check ? (
        <div
          className={
            check.ok
              ? 'text-xs text-[var(--status-good-fg)]'
              : 'text-xs text-[var(--status-bad-fg)]'
          }
        >
          {check.message}
        </div>
      ) : help ? (
        <div className="text-xs text-[var(--text-muted)]">{help}</div>
      ) : null}
    </label>
  )
}

const inputClass =
  'w-full rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2 text-sm text-[var(--text-primary)] placeholder:text-[var(--text-muted)]'

export default function SettingsPage() {
  const [form, setForm] = useState<FormState>(EMPTY)
  const [meta, setMeta] = useState<ConsoleSettings | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [saved, setSaved] = useState(false)
  const [loading, setLoading] = useState(true)
  const [saving, setSaving] = useState(false)
  const [testing, setTesting] = useState(false)
  const [testResult, setTestResult] = useState<SettingsTestResult | null>(null)

  const load = async () => {
    setLoading(true)
    setError(null)
    try {
      const data = await api.getSettings()
      setMeta(data)
      setForm({
        fleet_console_url: data.values.fleet_console_url || '',
        image_registry: data.values.image_registry || '',
        github_repo: data.values.github_repo || '',
        prometheus_url: data.values.prometheus_url || '',
        loki_url: data.values.loki_url || '',
        alertmanager_url: data.values.alertmanager_url || '',
        grafana_pi_overview_url: data.values.grafana_pi_overview_url || '',
        fleet_api_token: '',
        ci_report_token: '',
        github_token: '',
      })
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err))
    } finally {
      setLoading(false)
    }
  }

  useEffect(() => {
    load()
  }, [])

  const set =
    (key: keyof FormState) => (e: ChangeEvent<HTMLInputElement>) => {
      setForm((prev) => ({ ...prev, [key]: e.target.value }))
      setSaved(false)
      setTestResult(null)
    }

  const draftBody = (): Record<string, string> => {
    const body: Record<string, string> = {
      fleet_console_url: form.fleet_console_url.trim(),
      image_registry: form.image_registry.trim(),
      github_repo: form.github_repo.trim(),
      prometheus_url: form.prometheus_url.trim(),
      loki_url: form.loki_url.trim(),
      alertmanager_url: form.alertmanager_url.trim(),
      grafana_pi_overview_url: form.grafana_pi_overview_url.trim(),
    }
    if (form.fleet_api_token.trim()) {
      body.fleet_api_token = form.fleet_api_token.trim()
    }
    if (form.ci_report_token.trim()) {
      body.ci_report_token = form.ci_report_token.trim()
    }
    if (form.github_token.trim()) {
      body.github_token = form.github_token.trim()
    }
    return body
  }

  const save = async () => {
    setSaving(true)
    setError(null)
    setSaved(false)
    try {
      const body = draftBody()
      const data = await api.updateSettings(body)
      setMeta(data)
      if (form.fleet_api_token.trim()) {
        setToken(form.fleet_api_token.trim())
      }
      setForm((prev) => ({
        ...prev,
        fleet_api_token: '',
        ci_report_token: '',
        github_token: '',
        fleet_console_url: data.values.fleet_console_url || '',
        image_registry: data.values.image_registry || '',
        github_repo: data.values.github_repo || '',
        prometheus_url: data.values.prometheus_url || '',
        loki_url: data.values.loki_url || '',
        alertmanager_url: data.values.alertmanager_url || '',
        grafana_pi_overview_url: data.values.grafana_pi_overview_url || '',
      }))
      setSaved(true)
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err))
    } finally {
      setSaving(false)
    }
  }

  const testConnections = async () => {
    setTesting(true)
    setError(null)
    try {
      const result = await api.testSettings(draftBody())
      setTestResult(result)
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err))
      setTestResult(null)
    } finally {
      setTesting(false)
    }
  }

  const check = (key: string) => testResult?.checks?.[key]

  return (
    <div className="mx-auto max-w-2xl">
      <SectionHeader
        title="Settings"
        description="Console-owned parameters. Leave secret fields blank to keep the current value. Test uses saved values plus any draft fields you typed."
        action={
          <div className="flex flex-wrap gap-2">
            <Button
              variant="outline"
              onClick={testConnections}
              disabled={testing || loading}
            >
              <PlugZap className="h-4 w-4" />
              {testing ? 'Testing…' : 'Test connections'}
            </Button>
            <Button onClick={save} disabled={saving || loading}>
              <Save className="h-4 w-4" />
              {saving ? 'Saving…' : 'Save'}
            </Button>
          </div>
        }
      />

      {error ? (
        <div className="mb-4 rounded-[var(--radius-sm)] border border-[var(--status-bad-fg)]/40 bg-[var(--status-bad-bg)] px-3 py-2 text-sm text-[var(--status-bad-fg)]">
          {error}
        </div>
      ) : null}
      {saved ? (
        <div className="mb-4 rounded-[var(--radius-sm)] border border-[var(--status-good-fg)]/40 bg-[var(--status-good-bg)] px-3 py-2 text-sm text-[var(--status-good-fg)]">
          Settings saved.
        </div>
      ) : null}

      {testResult ? (
        <div className="mb-6 space-y-2 rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--surface-1)] p-4">
          <div className="flex items-center gap-2 text-sm font-medium">
            Connection checks
            <StatusBadge
              label={testResult.ok ? 'All good' : 'Issues found'}
              tone={testResult.ok ? 'good' : 'warn'}
            />
          </div>
          <ul className="space-y-1.5 text-xs">
            {Object.entries(testResult.checks).map(([key, item]) => (
              <li key={key} className="flex gap-2">
                <StatusBadge
                  label={item.ok ? 'ok' : 'fail'}
                  tone={item.ok ? 'good' : 'bad'}
                />
                <span className="text-[var(--text-secondary)]">
                  <span className="text-[var(--text-primary)]">
                    {CHECK_LABELS[key] || key}:
                  </span>{' '}
                  {item.message}
                </span>
              </li>
            ))}
          </ul>
        </div>
      ) : null}

      {meta?.note ? (
        <p className="mb-6 text-sm text-[var(--text-muted)]">{meta.note}</p>
      ) : null}

      <div className="space-y-8">
        <section className="space-y-4">
          <h3 className="text-xs font-semibold uppercase tracking-wide text-[var(--text-muted)]">
            Console
          </h3>
          <Field
            label="Public console URL"
            help="URL agents and CI use to reach this console (e.g. http://jetson:8090)."
            check={check('fleet_console_url')}
          >
            <input
              className={inputClass}
              value={form.fleet_console_url}
              onChange={set('fleet_console_url')}
              placeholder="http://100.x.x.x:8090"
            />
          </Field>
          <Field
            label="Image registry"
            help="Docker Hub namespace / repo prefix."
            check={check('image_registry')}
          >
            <input
              className={inputClass}
              value={form.image_registry}
              onChange={set('image_registry')}
              placeholder="iserenity/rhapsodi-promtek"
            />
          </Field>
          <Field
            label="Operator API token"
            help={
              meta?.secrets_set.fleet_api_token
                ? 'Set — enter a new value to rotate.'
                : 'Not set — console allows unauthenticated Tailnet access.'
            }
            check={check('fleet_api_token')}
          >
            <input
              className={inputClass}
              type="password"
              autoComplete="off"
              value={form.fleet_api_token}
              onChange={set('fleet_api_token')}
              placeholder={
                meta?.secrets_set.fleet_api_token
                  ? '•••• leave blank to keep'
                  : 'Optional shared bearer'
              }
            />
          </Field>
          <Field
            label="CI report token"
            help={
              meta?.secrets_set.ci_report_token
                ? 'Set — enter a new value to rotate. Keep GitHub CI_REPORT_TOKEN in sync.'
                : 'Not set — falls back to operator API token.'
            }
            check={check('ci_report_token')}
          >
            <input
              className={inputClass}
              type="password"
              autoComplete="off"
              value={form.ci_report_token}
              onChange={set('ci_report_token')}
              placeholder={
                meta?.secrets_set.ci_report_token
                  ? '•••• leave blank to keep'
                  : 'Token for POST /api/releases/report'
              }
            />
          </Field>
        </section>

        <section className="space-y-4">
          <h3 className="text-xs font-semibold uppercase tracking-wide text-[var(--text-muted)]">
            GitHub (Build CI from console)
          </h3>
          <Field
            label="Repository"
            help="owner/name used for branches and workflow_dispatch."
          >
            <input
              className={inputClass}
              value={form.github_repo}
              onChange={set('github_repo')}
              placeholder="ashwanth18/ws_rhapsodi-promtek"
            />
          </Field>
          <Field
            label="GitHub token"
            help={
              meta?.secrets_set.github_token
                ? 'Set — needs Actions:write to dispatch builds.'
                : 'Not set — Build CI will fail until configured.'
            }
            check={check('github')}
          >
            <input
              className={inputClass}
              type="password"
              autoComplete="off"
              value={form.github_token}
              onChange={set('github_token')}
              placeholder={
                meta?.secrets_set.github_token
                  ? '•••• leave blank to keep'
                  : 'ghp_… or fine-grained PAT'
              }
            />
          </Field>
        </section>

        <section className="space-y-4">
          <h3 className="text-xs font-semibold uppercase tracking-wide text-[var(--text-muted)]">
            Observability
          </h3>
          <Field label="Prometheus URL" check={check('prometheus')}>
            <input
              className={inputClass}
              value={form.prometheus_url}
              onChange={set('prometheus_url')}
              placeholder="http://127.0.0.1:9091"
            />
          </Field>
          <Field
            label="Loki URL"
            help="Central log store. Promtail on each device pushes here."
            check={check('loki')}
          >
            <input
              className={inputClass}
              value={form.loki_url}
              onChange={set('loki_url')}
              placeholder="http://127.0.0.1:3100"
            />
          </Field>
          <Field label="Alertmanager URL" check={check('alertmanager')}>
            <input
              className={inputClass}
              value={form.alertmanager_url}
              onChange={set('alertmanager_url')}
              placeholder="http://127.0.0.1:9093"
            />
          </Field>
          <Field label="Grafana Pi overview URL" check={check('grafana')}>
            <input
              className={inputClass}
              value={form.grafana_pi_overview_url}
              onChange={set('grafana_pi_overview_url')}
              placeholder="http://…/d/pi-overview/…"
            />
          </Field>
        </section>

        <section className="space-y-3">
          <h3 className="text-xs font-semibold uppercase tracking-wide text-[var(--text-muted)]">
            GitHub Actions secrets (not edited here)
          </h3>
          <p className="text-sm text-[var(--text-muted)]">
            Build runners need these in the repo: Settings → Secrets and variables → Actions.
            They cannot be tested from this console.
          </p>
          <ul className="grid gap-1 font-mono text-xs text-[var(--text-secondary)] sm:grid-cols-2">
            {(meta?.github_actions_secrets || []).map((name) => (
              <li
                key={name}
                className="rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-1)] px-2 py-1.5"
              >
                {name}
              </li>
            ))}
          </ul>
        </section>
      </div>
    </div>
  )
}
