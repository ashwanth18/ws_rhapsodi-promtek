import { Navigate, Route, Routes } from 'react-router-dom'
import Layout from './pages/Layout'
import DevicesPage from './pages/DevicesPage'
import DeviceDetailPage from './pages/DeviceDetailPage'
import DeploymentsPage from './pages/DeploymentsPage'

export default function App() {
  return (
    <Routes>
      <Route element={<Layout />}>
        <Route index element={<DevicesPage />} />
        <Route path="devices/:deviceId" element={<DeviceDetailPage />} />
        <Route path="deployments" element={<DeploymentsPage />} />
        <Route path="*" element={<Navigate to="/" replace />} />
      </Route>
    </Routes>
  )
}
