import { useParams, useSearchParams } from 'react-router-dom'
import DashboardView from '../components/dashboard/DashboardView'

export default function DashboardDetailPage() {
  const { dashboardId = '' } = useParams()
  const [params] = useSearchParams()
  const deviceId = params.get('device') || undefined
  const id = Number(dashboardId)
  if (!id || Number.isNaN(id)) {
    return (
      <div className="text-sm text-[var(--status-bad-fg)]">
        Invalid dashboard id
      </div>
    )
  }
  return <DashboardView dashboardId={id} deviceId={deviceId} />
}
