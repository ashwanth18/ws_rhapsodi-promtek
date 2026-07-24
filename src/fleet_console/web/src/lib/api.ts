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
  action: 'provision' | 'deploy' | string
  robot_type?: string | null
  site_id?: string | null
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
    body: { robot_type: 'niryo' | 'jaka'; site_id: string; image_tag: string },
  ) =>
    request<{ deployment: Deployment }>(`/api/devices/${id}/provision`, {
      method: 'POST',
      body: JSON.stringify(body),
    }),
  deploy: (id: string, body: { image_tag: string }) =>
    request<{ deployment: Deployment }>(`/api/devices/${id}/deploy`, {
      method: 'POST',
      body: JSON.stringify(body),
    }),
}

export function logsStreamUrl(deploymentId: number): string {
  const base = getApiBase() || window.location.origin
  const token = getToken()
  const url = new URL(`${base}/api/deployments/${deploymentId}/logs/stream`)
  // EventSource cannot set Authorization headers; pass token as query if set.
  if (token) url.searchParams.set('access_token', token)
  return url.toString()
}
