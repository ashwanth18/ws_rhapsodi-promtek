import { useEffect, useState } from 'react'
import { useNavigate } from 'react-router-dom'
import { RefreshCw } from 'lucide-react'
import { api, type Deployment } from '../lib/api'
import {
  Button,
  DataTable,
  SectionHeader,
  StatusBadge,
  deployTone,
} from '../components/ui'

export default function DeploymentsPage() {
  const navigate = useNavigate()
  const [rows, setRows] = useState<Deployment[]>([])
  const [status, setStatus] = useState('')
  const [error, setError] = useState<string | null>(null)

  const load = async () => {
    setError(null)
    try {
      const payload = await api.listDeployments({
        status: status || undefined,
      })
      setRows(payload.deployments)
    } catch (err) {
      setError(err instanceof Error ? err.message : String(err))
    }
  }

  useEffect(() => {
    load()
    const id = window.setInterval(load, 10000)
    return () => window.clearInterval(id)
  }, [status])

  return (
    <div>
      <SectionHeader
        title="Deployment activity"
        description="Fleet-wide provision and update history."
        action={
          <div className="flex items-center gap-2">
            <select
              className="rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-1)] px-3 py-2 text-sm"
              value={status}
              onChange={(e) => setStatus(e.target.value)}
            >
              <option value="">All statuses</option>
              <option value="running">running</option>
              <option value="success">success</option>
              <option value="failed">failed</option>
              <option value="rolled_back">rolled_back</option>
            </select>
            <Button variant="outline" onClick={load}>
              <RefreshCw className="h-4 w-4" />
              Refresh
            </Button>
          </div>
        }
      />

      {error ? (
        <div className="mb-4 rounded-[var(--radius-md)] border border-[var(--status-bad-fg)]/40 bg-[var(--status-bad-bg)] px-4 py-3 text-sm text-[var(--status-bad-fg)]">
          {error}
        </div>
      ) : null}

      <DataTable
        rows={rows}
        rowKey={(d) => d.id}
        emptyMessage="No deployments recorded yet."
        onRowClick={(d) => navigate(`/devices/${d.device_id}`)}
        columns={[
          {
            key: 'id',
            header: 'ID',
            render: (d) => <span className="text-xs">#{d.id}</span>,
          },
          {
            key: 'device',
            header: 'Device',
            render: (d) => <span className="font-medium">{d.device_id}</span>,
          },
          {
            key: 'action',
            header: 'Action',
            render: (d) => d.action,
          },
          {
            key: 'type',
            header: 'Robot type',
            render: (d) => d.robot_type || '—',
          },
          {
            key: 'tag',
            header: 'Version',
            render: (d) => <code className="text-xs text-[var(--accent)]">{d.image_tag}</code>,
          },
          {
            key: 'status',
            header: 'Status',
            render: (d) => (
              <StatusBadge
                label={d.status}
                tone={deployTone(d.status)}
                pulse={d.status === 'running'}
              />
            ),
          },
          {
            key: 'started',
            header: 'Started',
            render: (d) => (
              <span className="text-xs text-[var(--text-muted)]">
                {d.started_at ? new Date(d.started_at).toLocaleString() : '—'}
              </span>
            ),
          },
          {
            key: 'error',
            header: 'Error',
            render: (d) => (
              <span className="line-clamp-1 max-w-[220px] text-xs text-[var(--text-muted)]">
                {d.error_message || '—'}
              </span>
            ),
          },
        ]}
      />
    </div>
  )
}
