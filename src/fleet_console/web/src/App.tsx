import { Navigate, Route, Routes } from 'react-router-dom'
import Layout from './pages/Layout'
import DevicesPage from './pages/DevicesPage'
import DeviceDetailPage from './pages/DeviceDetailPage'
import DeploymentsPage from './pages/DeploymentsPage'
import ReleasesPage from './pages/ReleasesPage'
import SettingsPage from './pages/SettingsPage'
import MonitoringPage from './pages/MonitoringPage'
import DashboardsPage from './pages/DashboardsPage'
import DashboardDetailPage from './pages/DashboardDetailPage'

export default function App() {
  return (
    <Routes>
      <Route element={<Layout />}>
        <Route index element={<DevicesPage />} />
        <Route path="devices/:deviceId" element={<DeviceDetailPage />} />
        <Route path="monitoring" element={<MonitoringPage />} />
        <Route path="dashboards" element={<DashboardsPage />} />
        <Route path="dashboards/:dashboardId" element={<DashboardDetailPage />} />
        <Route path="releases" element={<ReleasesPage />} />
        <Route path="deployments" element={<DeploymentsPage />} />
        <Route path="settings" element={<SettingsPage />} />
        <Route path="*" element={<Navigate to="/" replace />} />
      </Route>
    </Routes>
  )
}
