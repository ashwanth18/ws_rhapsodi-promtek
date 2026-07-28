import { useEffect, useState } from 'react'
import { Link, useNavigate } from 'react-router-dom'
import { Copy, LayoutDashboard, Plus, Trash2 } from 'lucide-react'
import { api, type CustomDashboard } from '../lib/api'
import { Button, DataTable, SectionHeader, StatusBadge } from '../components/ui'

export default function DashboardsPage() {
  const navigate = useNavigate()
  const [dashboards, setDashboards] = useState<CustomDashboard[]>([])
  const [error, setError] = useState<string | null>(null)
  const [loading, setLoading] = useState(true)
  const [creating, setCreating] = useState(false)
  const [name, setName] = useState('New dashboard')
  const [scope, setScope] = useState<'fleet' | 'device_template'>('fleet')

  const load = async () => {
    setLoading(true)
    setError(null)
    try {
      const payload = await api.listDashboards()
      setDashboards(payload.dashboards)
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err))
    } finally {
      setLoading(false)
    }
  }

  useEffect(() => {
    void load()
  }, [])

  const create = async () => {
    setCreating(true)
    setError(null)
    try {
      const payload = await api.createDashboard({
        name: name.trim() || 'New dashboard',
        scope,
        panels: [],
      })
      navigate(`/dashboards/${payload.dashboard.id}`)
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err))
    } finally {
      setCreating(false)
    }
  }

  const duplicate = async (id: number) => {
    try {
      const payload = await api.duplicateDashboard(id)
      setDashboards((prev) => [payload.dashboard, ...prev])
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err))
    }
  }

  const remove = async (id: number) => {
    if (!window.confirm('Delete this dashboard?')) return
    try {
      await api.deleteDashboard(id)
      setDashboards((prev) => prev.filter((d) => d.id !== id))
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err))
    }
  }

  return (
    <div>
      <SectionHeader
        title="Dashboards"
        description="Grafana-style custom dashboards. Build panels from PromQL or LogQL, drag to arrange, and save."
        action={
          <div className="flex flex-wrap items-end gap-2">
            <label className="text-xs text-[var(--text-muted)]">
              Name
              <input
                className="mt-1 block rounded border border-[var(--border)] bg-[var(--surface-2)] px-2 py-1.5 text-sm"
                value={name}
                onChange={(e) => setName(e.target.value)}
              />
            </label>
            <label className="text-xs text-[var(--text-muted)]">
              Scope
              <select
                className="mt-1 block rounded border border-[var(--border)] bg-[var(--surface-2)] px-2 py-1.5 text-sm"
                value={scope}
                onChange={(e) =>
                  setScope(e.target.value as 'fleet' | 'device_template')
                }
              >
                <option value="fleet">Fleet</option>
                <option value="device_template">Device template</option>
              </select>
            </label>
            <Button onClick={create} disabled={creating}>
              <Plus className="h-4 w-4" />
              {creating ? 'Creating…' : 'New dashboard'}
            </Button>
          </div>
        }
      />

      {error ? (
        <div className="mb-4 rounded border border-[var(--status-bad-fg)]/40 bg-[var(--status-bad-bg)] px-3 py-2 text-sm text-[var(--status-bad-fg)]">
          {error}
        </div>
      ) : null}

      <DataTable
        rows={dashboards}
        rowKey={(d) => String(d.id)}
        emptyMessage={
          loading
            ? 'Loading…'
            : 'No dashboards yet — create one to start building panels.'
        }
        onRowClick={(d) => navigate(`/dashboards/${d.id}`)}
        columns={[
          {
            key: 'name',
            header: 'Dashboard',
            render: (d) => (
              <div className="flex items-center gap-2">
                <LayoutDashboard className="h-4 w-4 text-[var(--accent)]" />
                <div>
                  <div className="font-medium">{d.name}</div>
                  <div className="text-xs text-[var(--text-muted)]">
                    {d.panels?.length || 0} panels
                  </div>
                </div>
              </div>
            ),
          },
          {
            key: 'scope',
            header: 'Scope',
            render: (d) => (
              <StatusBadge
                label={d.scope}
                tone={d.scope === 'device_template' ? 'info' : 'neutral'}
              />
            ),
          },
          {
            key: 'updated',
            header: 'Updated',
            render: (d) => (
              <span className="text-xs text-[var(--text-muted)]">
                {d.updated_at
                  ? new Date(d.updated_at).toLocaleString()
                  : '—'}
              </span>
            ),
          },
          {
            key: 'actions',
            header: '',
            render: (d) => (
              <div
                className="flex gap-1"
                onClick={(e) => e.stopPropagation()}
              >
                <Link
                  to={`/dashboards/${d.id}`}
                  className="rounded px-2 py-1 text-xs text-[var(--accent)] hover:bg-white/5"
                >
                  Open
                </Link>
                <button
                  type="button"
                  className="rounded p-1 text-[var(--text-muted)] hover:bg-white/5"
                  title="Duplicate"
                  onClick={() => void duplicate(d.id)}
                >
                  <Copy className="h-3.5 w-3.5" />
                </button>
                <button
                  type="button"
                  className="rounded p-1 text-rose-300 hover:bg-white/5"
                  title="Delete"
                  onClick={() => void remove(d.id)}
                >
                  <Trash2 className="h-3.5 w-3.5" />
                </button>
              </div>
            ),
          },
        ]}
      />
    </div>
  )
}
