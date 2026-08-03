import { useCallback, useEffect, useMemo, useState } from 'react'
import { Download, RefreshCw } from 'lucide-react'
import { toast } from 'sonner'
import DateTimeRangeField from '../components/DateTimeRangeField'
import Button from '../components/ui/button'
import DataTable, { Column } from '../components/ui/DataTable'
import Select from '../components/ui/select'
import StatusBadge from '../components/ui/StatusBadge'
import { SectionHeader } from '../components/ui/SectionHeader'
import {
  downloadBundleZip,
  downloadRunsCsv,
  downloadTimeseriesCsv,
  ExportFilters,
  ExportRunRow,
  fetchExportRuns,
} from '../api/export'
import { useRuntimeConfig } from '../config/RuntimeConfig'
import { useRuntimeMode } from '../hooks/useRuntimeMode'

const ARTIFACT_KINDS = ['metadata', 'events', 'parquet', 'mcap'] as const

const ARTIFACT_LABELS: Record<(typeof ARTIFACT_KINDS)[number], string> = {
  metadata: 'meta',
  events: 'events',
  parquet: 'parquet',
  mcap: 'mcap',
}

function nsToIso(ns: number | null): string {
  if (ns == null) return '—'
  try {
    return new Date(ns / 1e6).toISOString()
  } catch {
    return '—'
  }
}

export default function DataExportPage() {
  const { apiBase } = useRuntimeConfig()
  const { capabilities } = useRuntimeMode(apiBase)
  const [rows, setRows] = useState<ExportRunRow[]>([])
  const [total, setTotal] = useState(0)
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)
  const [downloading, setDownloading] = useState<string | null>(null)

  const [timeFrom, setTimeFrom] = useState('')
  const [timeTo, setTimeTo] = useState('')
  const [mode, setMode] = useState('')
  const [powderId, setPowderId] = useState('')
  const [batchId, setBatchId] = useState('')
  const [episode, setEpisode] = useState('')

  const filters: ExportFilters = useMemo(
    () => ({
      time_from: timeFrom || undefined,
      time_to: timeTo || undefined,
      mode: mode || undefined,
      powder_id: powderId || undefined,
      batch_id: batchId || undefined,
      episode_index: episode.trim() ? Number.parseInt(episode, 10) : undefined,
      limit: 100,
      offset: 0,
    }),
    [timeFrom, timeTo, mode, powderId, batchId, episode]
  )

  const load = useCallback(async () => {
    setLoading(true)
    setError(null)
    try {
      const json = await fetchExportRuns(apiBase, filters)
      setRows(json.rows)
      setTotal(json.total)
    } catch (err) {
      const message = err instanceof Error ? err.message : 'Failed to load export runs'
      setError(message)
      setRows([])
      setTotal(0)
    } finally {
      setLoading(false)
    }
  }, [apiBase, filters])

  useEffect(() => {
    void load()
  }, [load])

  const runDownload = async (kind: 'runs' | 'timeseries' | 'bundle') => {
    setDownloading(kind)
    try {
      if (kind === 'runs') await downloadRunsCsv(apiBase, filters)
      else if (kind === 'timeseries') await downloadTimeseriesCsv(apiBase, filters)
      else await downloadBundleZip(apiBase, filters)
      toast.success('Download started')
    } catch (err) {
      toast.error(err instanceof Error ? err.message : 'Download failed')
    } finally {
      setDownloading(null)
    }
  }

  const columns: Column<ExportRunRow>[] = [
    {
      key: 'run_key',
      header: 'Run key',
      render: (r) => (
        <span className="font-mono text-xs">{r.run_key || '—'}</span>
      ),
    },
    {
      key: 'mode',
      header: 'Mode',
      render: (r) => r.mode || '—',
    },
    {
      key: 'batch',
      header: 'Batch',
      render: (r) => r.batch_id ?? '—',
    },
    {
      key: 'episode',
      header: 'Episode',
      render: (r) =>
        r.episode_index != null ? String(r.episode_index) : '—',
    },
    {
      key: 'powder',
      header: 'Powder',
      render: (r) => r.powder_name || r.powder_id || '—',
    },
    {
      key: 'weight',
      header: 'Net (g)',
      render: (r) =>
        r.net_weight_g != null ? r.net_weight_g.toFixed(1) : '—',
    },
    {
      key: 'stop_reason',
      header: 'Stop reason',
      render: (r) => r.stop_reason || '—',
    },
    {
      key: 'start',
      header: 'Start',
      render: (r) => (
        <span className="font-mono text-xs">{nsToIso(r.start_time_ns)}</span>
      ),
    },
    {
      key: 'artifacts',
      header: 'Artifacts',
      render: (r) => (
        <div className="flex flex-wrap gap-1">
          {ARTIFACT_KINDS.map((k) => (
            <StatusBadge
              key={k}
              label={ARTIFACT_LABELS[k]}
              tone={r.artifacts?.[k]?.present ? 'good' : 'idle'}
            />
          ))}
        </div>
      ),
    },
    {
      key: 'sync',
      header: 'Synced',
      render: (r) => {
        const syncedAt = r.manifest_sync?.tier0_synced_at
        if (syncedAt) {
          return (
            <StatusBadge
              label="Synced"
              tone="good"
              title={`tier0 synced at ${syncedAt}`}
            />
          )
        }
        return (
          <StatusBadge
            label="Not synced"
            tone="idle"
            title="tier0 not synced"
          />
        )
      },
    },
  ]

  return (
    <div className="px-5 py-5 lg:px-6">
      <SectionHeader
        title="Data Export"
        description="Browse processed runs and download CSV / bundle exports."
        action={
          <Button variant="outline" onClick={() => void load()} disabled={loading}>
            <RefreshCw className="h-4 w-4" />
            Refresh
          </Button>
        }
      />

      <div className="mb-4 grid gap-3 rounded-[var(--radius-md)] border border-[var(--border)] bg-[var(--surface-1)] p-4 lg:grid-cols-2">
        <DateTimeRangeField
          from={timeFrom}
          to={timeTo}
          onFromChange={setTimeFrom}
          onToChange={setTimeTo}
          onClear={() => {
            setTimeFrom('')
            setTimeTo('')
          }}
        />
        <div className="grid gap-3 sm:grid-cols-2">
          <div>
            <label className="mb-1 block text-xs font-medium text-[var(--text-muted)]">
              Mode
            </label>
            <Select
              value={mode}
              onChange={(e) => setMode(e.target.value)}
              className="w-full"
            >
              <option value="">All modes</option>
              {(capabilities?.modes || []).map((m) => (
                <option key={m.mode} value={m.mode}>
                  {m.label || m.mode}
                </option>
              ))}
            </Select>
          </div>
          <div>
            <label className="mb-1 block text-xs font-medium text-[var(--text-muted)]">
              Powder ID
            </label>
            <input
              value={powderId}
              onChange={(e) => setPowderId(e.target.value)}
              className="w-full rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2 text-sm outline-none focus:border-[var(--accent)]"
            />
          </div>
          <div>
            <label className="mb-1 block text-xs font-medium text-[var(--text-muted)]">
              Batch ID
            </label>
            <input
              value={batchId}
              onChange={(e) => setBatchId(e.target.value)}
              className="w-full rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2 text-sm outline-none focus:border-[var(--accent)]"
            />
          </div>
          <div>
            <label className="mb-1 block text-xs font-medium text-[var(--text-muted)]">
              Episode
            </label>
            <input
              value={episode}
              onChange={(e) => setEpisode(e.target.value)}
              inputMode="numeric"
              className="w-full rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-2 text-sm outline-none focus:border-[var(--accent)]"
            />
          </div>
        </div>
        <div className="flex flex-wrap items-start gap-4 lg:col-span-2">
          <div className="flex flex-col gap-1">
            <Button
              variant="outline"
              onClick={() => void runDownload('runs')}
              disabled={!!downloading}
            >
              <Download className="h-4 w-4" />
              {downloading === 'runs' ? '…' : 'Runs CSV'}
            </Button>
            <span className="text-xs text-[var(--text-faint)]">
              Runs CSV - one row per episode
            </span>
          </div>
          <div className="flex flex-col gap-1">
            <Button
              variant="outline"
              onClick={() => void runDownload('timeseries')}
              disabled={!!downloading}
            >
              <Download className="h-4 w-4" />
              {downloading === 'timeseries' ? '…' : 'Timeseries CSV'}
            </Button>
            <span className="text-xs text-[var(--text-faint)]">
              Timeseries CSV - weight samples
            </span>
          </div>
          <div className="flex flex-col gap-1">
            <Button
              variant="outline"
              onClick={() => void runDownload('bundle')}
              disabled={!!downloading}
            >
              <Download className="h-4 w-4" />
              {downloading === 'bundle' ? '…' : 'Bundle ZIP'}
            </Button>
            <span className="text-xs text-[var(--text-faint)]">
              Bundle .zip - artifacts for the filtered runs
            </span>
          </div>
        </div>
      </div>

      <div className="mb-2 text-xs text-[var(--text-muted)]">
        {loading ? 'Loading…' : `${rows.length} of ${total} runs`}
      </div>

      <DataTable
        columns={columns}
        rows={rows}
        rowKey={(r) => r.processed_id}
        loading={loading}
        error={error}
        emptyMessage="No runs match the current filters."
      />
    </div>
  )
}
