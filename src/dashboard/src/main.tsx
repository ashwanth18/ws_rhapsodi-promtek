import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import App from './App'
import './index.css'
import { createBrowserRouter, RouterProvider } from 'react-router-dom'
import DashboardPage from './pages/DashboardPage'
import LogsPage from './pages/LogsPage'
import ControlsPage from './pages/ControlsPage'
import WebhookWeightmentsPage from './pages/WebhookWeightmentsPage'
import WebhookWeightmentDetailPage from './pages/WebhookWeightmentDetailPage'
import StockLocationPage from './pages/StockLocationPage'
import { RosProvider } from './ros/RosContext'
import { RuntimeConfigProvider } from './config/RuntimeConfig'

const router = createBrowserRouter([
  { path: '/', element: <DashboardPage /> },
  { path: '/logs', element: <LogsPage /> },
  { path: '/webhook-weightments', element: <WebhookWeightmentsPage /> },
  { path: '/webhook-weightments/:eventId', element: <WebhookWeightmentDetailPage /> },
  { path: '/stock-location', element: <StockLocationPage /> },
  { path: '/controls', element: <ControlsPage /> },
])

createRoot(document.getElementById('root') as HTMLElement).render(
  <StrictMode>
    <RuntimeConfigProvider>
      {/* Provide a single shared ROS connection to all routes */}
      <RosProvider>
        <RouterProvider router={router} />
      </RosProvider>
    </RuntimeConfigProvider>
  </StrictMode>,
)
