import { useCallback, useEffect, useMemo, useState } from 'react'
import { Link } from 'react-router-dom'
import {
  ArrowLeft,
  GripVertical,
  Pencil,
  Plus,
  RefreshCw,
  Save,
  Trash2,
} from 'lucide-react'
import GridLayout, { WidthProvider, type Layout } from 'react-grid-layout/legacy'
import 'react-grid-layout/css/styles.css'
import 'react-resizable/css/styles.css'
import {
  api,
  type CustomDashboard,
  type DashboardPanel,
} from '../../lib/api'
import { Button, SectionHeader, StatusBadge } from '../ui'
import Panel from './Panel'
import PanelEditor from './PanelEditor'

const ReactGridLayout = WidthProvider(GridLayout)

const SINCE_OPTS = [
  { value: 900, label: '15m' },
  { value: 3600, label: '1h' },
  { value: 21600, label: '6h' },
  { value: 86400, label: '24h' },
]

export default function DashboardView({
  dashboardId,
  deviceId,
  embed,
}: {
  dashboardId: number
  deviceId?: string
  embed?: boolean
}) {
  const [dashboard, setDashboard] = useState<CustomDashboard | null>(null)
  const [panels, setPanels] = useState<DashboardPanel[]>([])
  const [name, setName] = useState('')
  const [editing, setEditing] = useState(false)
  const [editor, setEditor] = useState<Partial<DashboardPanel> | null>(null)
  const [error, setError] = useState<string | null>(null)
  const [saving, setSaving] = useState(false)
  const [saved, setSaved] = useState(false)
  const [sinceSeconds, setSinceSeconds] = useState(3600)
  const [refreshKey, setRefreshKey] = useState(0)

  const load = useCallback(async () => {
    try {
      const payload = await api.getDashboard(dashboardId)
      setDashboard(payload.dashboard)
      setName(payload.dashboard.name)
      setPanels(payload.dashboard.panels || [])
      setError(null)
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err))
    }
  }, [dashboardId])

  useEffect(() => {
    void load()
  }, [load])

  const layout: Layout = useMemo(
    () =>
      panels.map((p) => ({
        i: p.id,
        x: p.x,
        y: p.y,
        w: p.w,
        h: p.h,
        minW: 2,
        minH: 2,
      })),
    [panels],
  )

  const onLayoutChange = (next: Layout) => {
    if (!editing) return
    setPanels((prev) =>
      prev.map((p) => {
        const item = next.find((n) => n.i === p.id)
        if (!item) return p
        return { ...p, x: item.x, y: item.y, w: item.w, h: item.h }
      }),
    )
    setSaved(false)
  }

  const save = async () => {
    setSaving(true)
    setError(null)
    try {
      const payload = await api.updateDashboard(dashboardId, {
        name: name.trim() || dashboard?.name,
        panels,
      })
      setDashboard(payload.dashboard)
      setPanels(payload.dashboard.panels || [])
      setSaved(true)
      setEditing(false)
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err))
    } finally {
      setSaving(false)
    }
  }

  const removePanel = (id: string) => {
    setPanels((prev) => prev.filter((p) => p.id !== id))
    setSaved(false)
  }

  const upsertPanel = (panel: DashboardPanel) => {
    setPanels((prev) => {
      const idx = prev.findIndex((p) => p.id === panel.id)
      if (idx >= 0) {
        const copy = [...prev]
        copy[idx] = { ...copy[idx], ...panel }
        return copy
      }
      const maxY = prev.reduce((acc, p) => Math.max(acc, p.y + p.h), 0)
      return [
        ...prev,
        {
          ...panel,
          x: 0,
          y: maxY,
          w: panel.w || 6,
          h: panel.h || 4,
        },
      ]
    })
    setEditor(null)
    setSaved(false)
    setEditing(true)
  }

  if (!dashboard && !error) {
    return (
      <div className="text-sm text-[var(--text-muted)]">Loading dashboard…</div>
    )
  }

  return (
    <div>
      {!embed ? (
        <SectionHeader
          title={
            <div className="flex flex-wrap items-center gap-2">
              <Link
                to="/dashboards"
                className="inline-flex items-center gap-1 text-sm text-[var(--text-muted)] hover:text-[var(--accent)]"
              >
                <ArrowLeft className="h-4 w-4" />
                Dashboards
              </Link>
              <span className="text-[var(--text-muted)]">/</span>
              {editing ? (
                <input
                  className="rounded border border-[var(--border)] bg-[var(--surface-2)] px-2 py-1 text-lg font-semibold"
                  value={name}
                  onChange={(e) => {
                    setName(e.target.value)
                    setSaved(false)
                  }}
                />
              ) : (
                <span>{dashboard?.name}</span>
              )}
              <StatusBadge
                label={dashboard?.scope || 'fleet'}
                tone={
                  dashboard?.scope === 'device_template' ? 'info' : 'neutral'
                }
              />
            </div>
          }
          description={
            deviceId
              ? `Scoped to device ${deviceId} ($device_id substituted in queries).`
              : 'Fleet dashboard — use $device_id in queries for device templates.'
          }
          action={
            <div className="flex flex-wrap gap-2">
              <select
                className="rounded border border-[var(--border)] bg-[var(--surface-2)] px-2 py-1 text-xs"
                value={sinceSeconds}
                onChange={(e) => setSinceSeconds(Number(e.target.value))}
              >
                {SINCE_OPTS.map((o) => (
                  <option key={o.value} value={o.value}>
                    {o.label}
                  </option>
                ))}
              </select>
              <Button
                variant="ghost"
                size="sm"
                onClick={() => setRefreshKey((k) => k + 1)}
              >
                <RefreshCw className="h-3.5 w-3.5" />
              </Button>
              <Button
                variant="outline"
                size="sm"
                onClick={() => setEditing((v) => !v)}
              >
                <Pencil className="h-3.5 w-3.5" />
                {editing ? 'Done arranging' : 'Edit layout'}
              </Button>
              <Button
                variant="outline"
                size="sm"
                onClick={() =>
                  setEditor({
                    type: 'timeseries',
                    datasource: 'prometheus',
                    title: 'New panel',
                  })
                }
              >
                <Plus className="h-3.5 w-3.5" />
                Add panel
              </Button>
              <Button size="sm" onClick={save} disabled={saving}>
                <Save className="h-3.5 w-3.5" />
                {saving ? 'Saving…' : saved ? 'Saved' : 'Save'}
              </Button>
            </div>
          }
        />
      ) : (
        <div className="mb-3 flex flex-wrap items-center justify-between gap-2">
          <div className="font-display text-base font-semibold">
            {dashboard?.name}
          </div>
          <div className="flex gap-2">
            <Link
              to={`/dashboards/${dashboardId}${deviceId ? `?device=${encodeURIComponent(deviceId)}` : ''}`}
              className="text-xs text-[var(--accent)] hover:underline"
            >
              Open full editor
            </Link>
            <Button
              size="sm"
              variant="ghost"
              onClick={() => setRefreshKey((k) => k + 1)}
            >
              <RefreshCw className="h-3.5 w-3.5" />
            </Button>
          </div>
        </div>
      )}

      {error ? (
        <div className="mb-4 rounded border border-[var(--status-bad-fg)]/40 bg-[var(--status-bad-bg)] px-3 py-2 text-sm text-[var(--status-bad-fg)]">
          {error}
        </div>
      ) : null}

      {panels.length === 0 ? (
        <div className="rounded-[var(--radius-md)] border border-dashed border-[var(--border)] px-4 py-16 text-center text-sm text-[var(--text-muted)]">
          No panels yet.{' '}
          <button
            type="button"
            className="text-[var(--accent)] hover:underline"
            onClick={() =>
              setEditor({
                type: 'timeseries',
                datasource: 'prometheus',
                title: 'CPU %',
              })
            }
          >
            Add your first panel
          </button>
        </div>
      ) : (
        <ReactGridLayout
          className="layout"
          layout={layout}
          cols={12}
          rowHeight={48}
          margin={[12, 12] as [number, number]}
          containerPadding={[0, 0] as [number, number]}
          onLayoutChange={onLayoutChange}
          isDraggable={editing && !embed}
          isResizable={editing && !embed}
          draggableHandle=".panel-drag-handle"
        >
          {panels.map((panel) => (
            <div key={panel.id} className="relative">
              {editing && !embed ? (
                <div className="absolute right-2 top-2 z-10 flex gap-1">
                  <button
                    type="button"
                    className="panel-drag-handle cursor-grab rounded bg-black/40 p-1 text-slate-300 hover:bg-black/60 active:cursor-grabbing"
                    title="Drag"
                  >
                    <GripVertical className="h-3 w-3" />
                  </button>
                  <button
                    type="button"
                    className="rounded bg-black/40 p-1 text-slate-300 hover:bg-black/60"
                    title="Edit"
                    onClick={() => setEditor(panel)}
                  >
                    <Pencil className="h-3 w-3" />
                  </button>
                  <button
                    type="button"
                    className="rounded bg-black/40 p-1 text-rose-300 hover:bg-black/60"
                    title="Remove"
                    onClick={() => removePanel(panel.id)}
                  >
                    <Trash2 className="h-3 w-3" />
                  </button>
                </div>
              ) : null}
              <Panel
                panel={panel}
                deviceId={deviceId}
                sinceSeconds={sinceSeconds}
                refreshKey={refreshKey}
                className="h-full"
              />
            </div>
          ))}
        </ReactGridLayout>
      )}

      {editor ? (
        <PanelEditor
          initial={editor}
          deviceId={deviceId}
          onCancel={() => setEditor(null)}
          onSave={upsertPanel}
        />
      ) : null}
    </div>
  )
}
