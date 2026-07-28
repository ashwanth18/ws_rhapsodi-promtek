const TOKEN_KEY = 'rhapsodi.fleetToken'

export function getApiBase(): string {
  const fromEnv = import.meta.env.VITE_API_BASE as string | undefined
  if (fromEnv) return fromEnv.replace(/\/$/, '')
  return ''
}

export function getToken(): string {
  return localStorage.getItem(TOKEN_KEY) || ''
}

export function setToken(token: string) {
  localStorage.setItem(TOKEN_KEY, token)
}

async function request<T>(path: string, init?: RequestInit): Promise<T> {
  const headers = new Headers(init?.headers)
  headers.set('Accept', 'application/json')
  if (init?.body && !headers.has('Content-Type')) {
    headers.set('Content-Type', 'application/json')
  }
  const token = getToken()
  if (token) headers.set('Authorization', `Bearer ${token}`)

  const resp = await fetch(`${getApiBase()}${path}`, { ...init, headers })
  if (!resp.ok) {
    let detail = resp.statusText
    try {
      const payload = await resp.json()
      detail = payload.detail || JSON.stringify(payload)
    } catch {
      /* ignore */
    }
    throw new Error(typeof detail === 'string' ? detail : JSON.stringify(detail))
  }
  return resp.json() as Promise<T>
}

export type Release = {
  id: number
  branch: string
  git_sha: string
  status: string
  images?: Record<string, string>
  image_registry?: string | null
  workflow_run_url?: string | null
  error_message?: string | null
  reported_at?: string | null
  subject?: string | null
  demo?: boolean
  duration_seconds?: number | null
  platforms?: string[]
  /** Hardware classes this release is intended for (e.g. pi5). Empty = no class gate. */
  device_classes?: string[]
  build_timings?: {
    total_seconds?: number
    roles?: Record<string, number>
    builder?: string
    platforms?: string
  } | null
}

export function formatDuration(seconds?: number | null): string {
  if (seconds == null || Number.isNaN(seconds)) return ''
  const s = Math.max(0, Math.round(seconds))
  const h = Math.floor(s / 3600)
  const m = Math.floor((s % 3600) / 60)
  const r = s % 60
  if (h > 0) return `${h}h ${m}m ${r}s`
  if (m > 0) return `${m}m ${String(r).padStart(2, '0')}s`
  return `${r}s`
}

export type Branch = {
  name: string
  sha?: string
  protected?: boolean
}

export type DeviceTarget = {
  device_id: string
  tracked_branch: string
  profile_id: string
  release_id?: number | null
  auto_update?: boolean
  robot_type?: string | null
  site_id?: string | null
  platform?: string | null
  device_class?: string | null
  compose_file?: string | null
  agent_status?: string | null
  agent_message?: string | null
  agent_applied_release_id?: number | null
  agent_reported_at?: string | null
  has_agent_token?: boolean
}

export type Profile = {
  id: string
  description: string
  robot_type?: string | null
  device_classes?: string[]
  compose_file?: string
}

export type Device = {
  id: string
  hostname: string
  ip: string
  role: string
  online: boolean
  tags: string[]
  provisioned: boolean
  alive: boolean | null
  active: boolean
  robot_type?: string | null
  site_id?: string | null
  platform?: string | null
  device_class?: string | null
  device_id?: string | null
  robot_id?: string | null
  image_tag?: string | null
  running_profile_id?: string | null
  running_source?: 'host_info' | 'agent' | string | null
  desired_branch?: string | null
  desired_profile_id?: string | null
  desired_image_tag?: string | null
  desired_release?: Release | null
  latest_release?: Release | null
  update_available?: boolean | null
  latest_sha?: string | null
  update_kind?: 'release' | 'needs_build' | string | null
  needs_build?: boolean | null
  branch_tip_sha?: string | null
  branch_tip_message?: string | null
  drift?: { profile: boolean; version: boolean; any: boolean }
  target?: DeviceTarget
  agent_status?: string | null
  agent_message?: string | null
  agent_reported_at?: string | null
  agent_applied_release?: Release | null
  version_check?: Record<string, unknown>
  metrics?: { cpu_pct?: number | null; mem_pct?: number | null; disk_pct?: number | null }
  last_deployment?: Deployment | null
  deployments?: Deployment[]
  grafana_url?: string
  /** Robot operator SPA (compose dashboard service, default :8080). */
  dashboard_url?: string | null
  dashboard_url_ip?: string | null
  /** Robot FastAPI (:8000). */
  api_url?: string | null
  api_url_ip?: string | null
  host_info?: Record<string, unknown> | null
  active_run?: Record<string, unknown> | null
}

export type Deployment = {
  id: number
  device_id: string
  action: 'provision' | 'deploy' | 'build' | 'reconcile' | string
  robot_type?: string | null
  site_id?: string | null
  profile_id?: string | null
  tracked_branch?: string | null
  release_id?: number | null
  image_tag: string
  status: 'running' | 'success' | 'failed' | 'rolled_back' | string
  requested_by?: string | null
  error_message?: string | null
  started_at?: string | null
  finished_at?: string | null
}

export type ConsoleSettings = {
  auth_required: boolean
  values: Record<string, string | null>
  secrets_set: Record<string, boolean>
  github_actions_secrets: string[]
  note: string
}

export type SettingsCheck = {
  ok: boolean
  message: string
  [key: string]: unknown
}

export type SettingsTestResult = {
  ok: boolean
  checks: Record<string, SettingsCheck>
  used_draft_overrides?: boolean
}

export type LogLine = {
  ts: number
  container: string
  stream: string
  text: string
}

export type FleetAlert = {
  source: string
  alertname: string
  state: string
  severity: string
  instance: string
  labels: Record<string, string>
  annotations: Record<string, string>
  active_at?: string | number | null
  silenced?: boolean
  inhibited?: boolean
  fingerprint?: string
}

export type DeviceSeries = {
  instance: string
  start: number
  end: number
  step: string
  cpu_pct: Array<{ t: number; v: number }>
  mem_pct: Array<{ t: number; v: number }>
  disk_pct: Array<{ t: number; v: number }>
}

export type FleetSummary = {
  devices_total: number
  devices_up: number
  devices_down: number
  high_mem: string[]
  high_disk: string[]
  devices: Record<
    string,
    {
      alive?: boolean | null
      cpu_pct?: number | null
      mem_pct?: number | null
      disk_pct?: number | null
    }
  >
  alert_counts: Record<string, number>
  alerts_total: number
}

export type PanelType =
  | 'timeseries'
  | 'area'
  | 'bar'
  | 'stat'
  | 'gauge'
  | 'pie'
  | 'table'
  | 'logs'
export type PanelDatasource = 'prometheus' | 'loki'

export type PromBuilderFn =
  | 'raw'
  | 'rate'
  | 'irate'
  | 'increase'
  | 'avg'
  | 'sum'
  | 'max'
  | 'min'
  | 'cpu_pct_from_idle'
  | 'mem_pct_used'
  | 'disk_root_pct'
  | 'temp_max'

export type PanelBuilderState = {
  mode?: 'guided' | 'advanced'
  metric?: string
  matchers?: Array<{ label: string; value: string }>
  fn?: PromBuilderFn
  groupBy?: string[]
  range?: string
  host?: string
  container?: string
  lineFilter?: string
  stream?: string
}

/** Ordered color band ending at `upTo` (absolute value on gauge/stat scale). */
export type ThresholdStep = {
  upTo: number
  color: string
  label?: string
}

export type PanelOptions = {
  showLegend?: boolean
  decimals?: number
  stack?: boolean
  fillOpacity?: number
  smooth?: boolean
  thresholds?: ThresholdStep[]
  showPointer?: boolean
  donut?: boolean
}

export type CatalogMetric = {
  id: string
  label: string
  hint?: string
  category: string
  datasource: PanelDatasource
  query: string
  viz: PanelType | string
  unit?: string
  min?: number
  max?: number
  options?: PanelOptions
}

export type DashboardPanel = {
  id: string
  title: string
  type: PanelType
  datasource: PanelDatasource
  query: string
  unit?: string
  /** Gauge scale minimum (inclusive). */
  min?: number
  /** Gauge scale maximum (inclusive). */
  max?: number
  options?: PanelOptions
  /** Curated catalog entry id when picked from metric catalog. */
  catalog_id?: string
  builder?: PanelBuilderState
  x: number
  y: number
  w: number
  h: number
}

export type CustomDashboard = {
  id: number
  name: string
  scope: 'fleet' | 'device_template' | string
  panels: DashboardPanel[]
  created_at?: string | null
  updated_at?: string | null
}

export type QueryResult = {
  ds: string
  expr: string
  status?: string
  resultType?: string
  result?: unknown[]
  lines?: LogLine[]
  start?: number
  end?: number
}

export const api = {
  getSettings: () => request<ConsoleSettings>('/api/settings'),
  updateSettings: (body: Record<string, string>) =>
    request<ConsoleSettings>('/api/settings', {
      method: 'PUT',
      body: JSON.stringify(body),
    }),
  testSettings: (body?: Record<string, string>) =>
    request<SettingsTestResult>('/api/settings/test', {
      method: 'POST',
      body: JSON.stringify(body || {}),
    }),
  listDevices: () => request<{ devices: Device[] }>('/api/devices'),
  getDevice: (id: string) => request<{ device: Device }>(`/api/devices/${id}`),
  listProfiles: (opts?: { robotType?: string; deviceClass?: string }) => {
    const qs = new URLSearchParams()
    if (opts?.robotType) qs.set('robot_type', opts.robotType)
    if (opts?.deviceClass) qs.set('device_class', opts.deviceClass)
    const suffix = qs.toString() ? `?${qs}` : ''
    return request<{ profiles: Profile[] }>(`/api/profiles${suffix}`)
  },
  listRobotTypes: () => request<{ robot_types: string[] }>('/api/robot_types'),
  listBranches: () => request<{ branches: Branch[] }>('/api/branches'),
  listReleases: (params?: {
    branch?: string
    status?: string
    sync?: boolean
    platform?: string
    device_class?: string
    device_id?: string
  }) => {
    const qs = new URLSearchParams()
    if (params?.branch) qs.set('branch', params.branch)
    if (params?.status) qs.set('status', params.status)
    if (params?.sync) qs.set('sync', 'true')
    if (params?.platform) qs.set('platform', params.platform)
    if (params?.device_class) qs.set('device_class', params.device_class)
    if (params?.device_id) qs.set('device_id', params.device_id)
    const suffix = qs.toString() ? `?${qs}` : ''
    return request<{ releases: Release[]; synced?: number; platform?: string | null }>(
      `/api/releases${suffix}`,
    )
  },
  syncReleases: () =>
    request<{ created: number; releases: Release[] }>('/api/releases/sync', {
      method: 'POST',
    }),
  cancelDeployment: (id: number) =>
    request<{ deployment: Deployment; signalled: boolean }>(
      `/api/deployments/${id}/cancel`,
      { method: 'POST' },
    ),
  updateTarget: (
    id: string,
    body: {
      tracked_branch?: string
      profile_id?: string
      release_id?: number | null
      auto_update?: boolean
      robot_type?: string
      site_id?: string
    },
  ) =>
    request<{ target: DeviceTarget }>(`/api/devices/${id}/target`, {
      method: 'PUT',
      body: JSON.stringify(body),
    }),
  versionCheck: (id: string) =>
    request<Record<string, unknown>>(`/api/devices/${id}/version_check`),
  listDeployments: (params?: {
    device_id?: string
    status?: string
    page?: number
    limit?: number
  }) => {
    const qs = new URLSearchParams()
    if (params?.device_id) qs.set('device_id', params.device_id)
    if (params?.status) qs.set('status', params.status)
    if (params?.page) qs.set('page', String(params.page))
    if (params?.limit) qs.set('limit', String(params.limit))
    const suffix = qs.toString() ? `?${qs}` : ''
    return request<{
      deployments: Deployment[]
      page: number
      limit: number
      total: number
      pages: number
    }>(`/api/deployments${suffix}`)
  },
  getDeployment: (id: number) =>
    request<{ deployment: Deployment }>(`/api/deployments/${id}`),
  getLogs: (id: number) =>
    request<{ deployment_id: number; log: string; status: string }>(
      `/api/deployments/${id}/logs`,
    ),
  provision: (
    id: string,
    body: {
      robot_type: string
      site_id: string
      release_id: number
      profile_id: string
      tracked_branch: string
    },
  ) =>
    request<{ deployment: Deployment }>(`/api/devices/${id}/provision`, {
      method: 'POST',
      body: JSON.stringify(body),
    }),
  deploy: (
    id: string,
    body: { release_id: number; profile_id?: string; tracked_branch?: string },
  ) =>
    request<{ deployment: Deployment; target: DeviceTarget }>(
      `/api/devices/${id}/deploy`,
      {
        method: 'POST',
        body: JSON.stringify(body),
      },
    ),
  build: (id: string, body: { branch?: string }) =>
    request<{ deployment: Deployment; workflow?: Record<string, unknown> }>(
      `/api/devices/${id}/build`,
      {
        method: 'POST',
        body: JSON.stringify(body),
      },
    ),
  /** Fleet-level CI build (preferred). Not tied to a device. */
  triggerBuild: (body: { branch?: string }) =>
    request<{ deployment: Deployment; workflow?: Record<string, unknown> | null }>(
      '/api/builds',
      {
        method: 'POST',
        body: JSON.stringify(body),
      },
    ),
  listWorkflowRuns: (params?: { branch?: string; limit?: number }) => {
    const qs = new URLSearchParams()
    if (params?.branch) qs.set('branch', params.branch)
    if (params?.limit) qs.set('limit', String(params.limit))
    const suffix = qs.toString() ? `?${qs}` : ''
    return request<{
      runs: Array<{
        id?: number
        status?: string
        conclusion?: string | null
        html_url?: string
        head_branch?: string
        head_sha?: string
        created_at?: string
        updated_at?: string
      }>
    }>(`/api/workflow_runs${suffix}`)
  },
  fleetMetrics: () => request<FleetSummary>('/api/metrics/fleet'),
  deviceSeries: (deviceId: string, params?: { since?: string; step?: string }) => {
    const qs = new URLSearchParams()
    if (params?.since) qs.set('since', params.since)
    if (params?.step) qs.set('step', params.step)
    const suffix = qs.toString() ? `?${qs}` : ''
    return request<DeviceSeries>(
      `/api/metrics/devices/${encodeURIComponent(deviceId)}/series${suffix}`,
    )
  },
  listAlerts: (instance?: string) => {
    const qs = instance
      ? `?instance=${encodeURIComponent(instance)}`
      : ''
    return request<{
      alerts: FleetAlert[]
      counts_by_instance: Record<string, number>
    }>(`/api/alerts${qs}`)
  },
  deviceLogContainers: (deviceId: string) =>
    request<{ device_id: string; containers: string[] }>(
      `/api/logs/devices/${encodeURIComponent(deviceId)}/containers`,
    ),
  deviceLogs: (
    deviceId: string,
    params?: {
      container?: string
      q?: string
      since?: string
      start?: number
      end?: number
      limit?: number
      direction?: string
    },
  ) => {
    const qs = new URLSearchParams()
    if (params?.container) qs.set('container', params.container)
    if (params?.q) qs.set('q', params.q)
    if (params?.since) qs.set('since', params.since)
    if (params?.start != null) qs.set('start', String(params.start))
    if (params?.end != null) qs.set('end', String(params.end))
    if (params?.limit != null) qs.set('limit', String(params.limit))
    if (params?.direction) qs.set('direction', params.direction)
    const suffix = qs.toString() ? `?${qs}` : ''
    return request<{
      device_id: string
      query: string
      start: number
      end: number
      lines: LogLine[]
    }>(`/api/logs/devices/${encodeURIComponent(deviceId)}/query${suffix}`)
  },
  listMetricCatalog: () =>
    request<{ metrics: CatalogMetric[] }>('/api/metrics/catalog'),
  query: (params: {
    ds: PanelDatasource | string
    expr: string
    start?: number
    end?: number
    step?: string
    limit?: number
    direction?: string
    device_id?: string
  }) => {
    const qs = new URLSearchParams()
    qs.set('ds', params.ds)
    qs.set('expr', params.expr)
    if (params.start != null) qs.set('start', String(params.start))
    if (params.end != null) qs.set('end', String(params.end))
    if (params.step) qs.set('step', params.step)
    if (params.limit != null) qs.set('limit', String(params.limit))
    if (params.direction) qs.set('direction', params.direction)
    if (params.device_id) qs.set('device_id', params.device_id)
    return request<QueryResult>(`/api/query?${qs}`)
  },
  metaPromMetrics: (match?: string, limit = 2000) => {
    const qs = new URLSearchParams()
    if (match) qs.set('match', match)
    if (limit) qs.set('limit', String(limit))
    const suffix = qs.toString() ? `?${qs}` : ''
    return request<{ metrics: string[] }>(
      `/api/query/meta/prometheus/metrics${suffix}`,
    )
  },
  metaPromLabels: (metric?: string) => {
    const qs = metric
      ? `?metric=${encodeURIComponent(metric)}`
      : ''
    return request<{ labels: string[] }>(
      `/api/query/meta/prometheus/labels${qs}`,
    )
  },
  metaPromLabelValues: (params: {
    label: string
    metric?: string
    match?: string
  }) => {
    const qs = new URLSearchParams()
    qs.set('label', params.label)
    if (params.metric) qs.set('metric', params.metric)
    if (params.match) qs.set('match', params.match)
    return request<{ label: string; values: string[] }>(
      `/api/query/meta/prometheus/label-values?${qs}`,
    )
  },
  metaLokiLabels: () =>
    request<{ labels: string[] }>('/api/query/meta/loki/labels'),
  metaLokiLabelValues: (label: string, match?: string) => {
    const qs = new URLSearchParams()
    qs.set('label', label)
    if (match) qs.set('match', match)
    return request<{ label: string; values: string[] }>(
      `/api/query/meta/loki/label-values?${qs}`,
    )
  },
  listDashboards: (scope?: string) => {
    const qs = scope ? `?scope=${encodeURIComponent(scope)}` : ''
    return request<{ dashboards: CustomDashboard[] }>(`/api/dashboards${qs}`)
  },
  getDashboard: (id: number) =>
    request<{ dashboard: CustomDashboard }>(`/api/dashboards/${id}`),
  createDashboard: (body: {
    name: string
    scope?: string
    panels?: DashboardPanel[]
  }) =>
    request<{ dashboard: CustomDashboard }>('/api/dashboards', {
      method: 'POST',
      body: JSON.stringify(body),
    }),
  updateDashboard: (
    id: number,
    body: { name?: string; scope?: string; panels?: DashboardPanel[] },
  ) =>
    request<{ dashboard: CustomDashboard }>(`/api/dashboards/${id}`, {
      method: 'PUT',
      body: JSON.stringify(body),
    }),
  deleteDashboard: (id: number) =>
    request<{ ok: boolean; id: number }>(`/api/dashboards/${id}`, {
      method: 'DELETE',
    }),
  duplicateDashboard: (id: number) =>
    request<{ dashboard: CustomDashboard }>(
      `/api/dashboards/${id}/duplicate`,
      { method: 'POST' },
    ),
}

export function logsStreamUrl(deploymentId: number): string {
  const base = getApiBase() || window.location.origin
  const token = getToken()
  const url = new URL(`${base}/api/deployments/${deploymentId}/logs/stream`)
  if (token) url.searchParams.set('access_token', token)
  return url.toString()
}
