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

export const api = {
  listDevices: () => request<{ devices: Device[] }>('/api/devices'),
  getDevice: (id: string) => request<{ device: Device }>(`/api/devices/${id}`),
  listProfiles: (robotType?: string) => {
    const qs = robotType ? `?robot_type=${encodeURIComponent(robotType)}` : ''
    return request<{ profiles: Profile[] }>(`/api/profiles${qs}`)
  },
  listRobotTypes: () => request<{ robot_types: string[] }>('/api/robot_types'),
  listBranches: () => request<{ branches: Branch[] }>('/api/branches'),
  listReleases: (params?: { branch?: string; status?: string; sync?: boolean }) => {
    const qs = new URLSearchParams()
    if (params?.branch) qs.set('branch', params.branch)
    if (params?.status) qs.set('status', params.status)
    if (params?.sync) qs.set('sync', 'true')
    const suffix = qs.toString() ? `?${qs}` : ''
    return request<{ releases: Release[]; synced?: number }>(`/api/releases${suffix}`)
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
  listDeployments: (params?: { device_id?: string; status?: string }) => {
    const qs = new URLSearchParams()
    if (params?.device_id) qs.set('device_id', params.device_id)
    if (params?.status) qs.set('status', params.status)
    const suffix = qs.toString() ? `?${qs}` : ''
    return request<{ deployments: Deployment[] }>(`/api/deployments${suffix}`)
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
}

export function logsStreamUrl(deploymentId: number): string {
  const base = getApiBase() || window.location.origin
  const token = getToken()
  const url = new URL(`${base}/api/deployments/${deploymentId}/logs/stream`)
  if (token) url.searchParams.set('access_token', token)
  return url.toString()
}
