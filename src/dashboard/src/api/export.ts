export type ExportArtifactInfo = {
  present: boolean
  size_bytes: number | null
  path: string | null
}

export type ExportRunRow = {
  processed_id: number
  run_db_id: number
  run_key: string | null
  run_id: string | null
  batch_id: string | null
  episode_index: number | null
  mode: string | null
  powder_id: string | null
  powder_name: string | null
  lot_code: string | null
  operator: string | null
  target_weight_g: number | null
  final_weight_g: number | null
  net_weight_g: number | null
  stop_reason: string | null
  start_time_ns: number | null
  end_time_ns: number | null
  artifacts: Record<string, ExportArtifactInfo>
  manifest_sync: {
    tier0_synced_at: string | null
    tier1_acked_at: string | null
    tier1_pruned_at: string | null
    anomaly_flag: boolean
  } | null
}

export type ExportRunsResponse = {
  rows: ExportRunRow[]
  total: number
  limit: number
  offset: number
}

export type ExportFilters = {
  time_from?: string
  time_to?: string
  mode?: string
  batch_id?: string
  episode_index?: number
  powder_id?: string
  run_key?: string
  limit?: number
  offset?: number
}

function buildQuery(filters: ExportFilters): string {
  const params = new URLSearchParams()
  for (const [key, value] of Object.entries(filters)) {
    if (value !== undefined && value !== '' && value !== null) {
      params.set(key, String(value))
    }
  }
  const qs = params.toString()
  return qs ? `?${qs}` : ''
}

export async function fetchExportRuns(
  apiBase: string,
  filters: ExportFilters = {}
): Promise<ExportRunsResponse> {
  const res = await fetch(`${apiBase}/export/runs${buildQuery(filters)}`)
  if (!res.ok) {
    throw new Error(`Export runs failed (${res.status})`)
  }
  return res.json() as Promise<ExportRunsResponse>
}

export async function downloadExportBlob(
  apiBase: string,
  path: string,
  filename: string
): Promise<void> {
  const res = await fetch(`${apiBase}${path}`)
  if (!res.ok) {
    const text = await res.text().catch(() => '')
    throw new Error(text || `Download failed (${res.status})`)
  }
  const blob = await res.blob()
  const url = URL.createObjectURL(blob)
  const anchor = document.createElement('a')
  anchor.href = url
  anchor.download = filename
  document.body.appendChild(anchor)
  anchor.click()
  anchor.remove()
  URL.revokeObjectURL(url)
}

export function downloadRunsCsv(apiBase: string, filters: ExportFilters = {}) {
  return downloadExportBlob(
    apiBase,
    `/export/runs.csv${buildQuery(filters)}`,
    'runs.csv'
  )
}

export function downloadTimeseriesCsv(apiBase: string, filters: ExportFilters = {}) {
  return downloadExportBlob(
    apiBase,
    `/export/timeseries.csv${buildQuery(filters)}`,
    'timeseries.csv'
  )
}

export function downloadBundleZip(apiBase: string, filters: ExportFilters = {}) {
  return downloadExportBlob(
    apiBase,
    `/export/bundle.zip${buildQuery(filters)}`,
    'rhapsodi-export.zip'
  )
}
