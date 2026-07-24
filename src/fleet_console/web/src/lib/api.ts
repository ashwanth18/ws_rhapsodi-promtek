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

export type DeviceTarget = {
  device_id: string
  tracked_branch: string
  profile_id: string
  pinned_image_tag?: string | null
  auto_update?: boolean
  robot_type?: string | null
  site_id?: string | null
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
  desired_branch?: string | null
  desired_profile_id?: string | null
  desired_image_tag?: string | null
  update_available?: boolean | null
  latest_sha?: string | null
  drift?: { profile: boolean; version: boolean; any: boolean }
  target?: DeviceTarget
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
  action: 'provision' | 'deploy' | 'build' | string
  robot_type?: string | null
  site_id?: string | null
  profile_id?: string | null
  tracked_branch?: string | null
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
  updateTarget: (id: string, body: Partial<DeviceTarget>) =>
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
  imageTags: () => request<{ image_tags: string[] }>('/api/image_tags'),
  provision: (
    id: string,
    body: {
      robot_type: 'niryo' | 'jaka'
      site_id: string
      image_tag: string
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
    body: { image_tag: string; profile_id?: string; tracked_branch?: string },
  ) =>
    request<{ deployment: Deployment }>(`/api/devices/${id}/deploy`, {
      method: 'POST',
      body: JSON.stringify(body),
    }),
  build: (
    id: string,
    body: { branch?: string; deploy_after?: boolean; profile_id?: string },
  ) =>
    request<{ deployment: Deployment }>(`/api/devices/${id}/build`, {
      method: 'POST',
      body: JSON.stringify(body),
    }),
}

export function logsStreamUrl(deploymentId: number): string {
  const base = getApiBase() || window.location.origin
  const token = getToken()
  const url = new URL(`${base}/api/deployments/${deploymentId}/logs/stream`)
  if (token) url.searchParams.set('access_token', token)
  return url.toString()
}
