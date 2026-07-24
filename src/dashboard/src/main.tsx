import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import App from './App'
import './index.css'
import { createBrowserRouter, RouterProvider } from 'react-router-dom'
import DashboardPage from './pages/DashboardPage'
import LogsPage from './pages/LogsPage'
import ControlsPage from './pages/ControlsPage'
import BatchListPage from './pages/BatchListPage'
import BatchDetailPage from './pages/BatchDetailPage'
import StockLocationPage from './pages/StockLocationPage'
import { RosProvider } from './ros/RosContext'
import { RuntimeConfigProvider } from './config/RuntimeConfig'
import { ThemeProvider } from './theme/ThemeContext'

const router = createBrowserRouter([
  { path: '/', element: <DashboardPage /> },
  { path: '/logs', element: <LogsPage /> },
  { path: '/batches', element: <BatchListPage /> },
  { path: '/batches/:eventId', element: <BatchDetailPage /> },
  { path: '/webhook-weightments', element: <BatchListPage /> },
  { path: '/webhook-weightments/:eventId', element: <BatchDetailPage /> },
  { path: '/stock-location', element: <StockLocationPage /> },
  { path: '/controls', element: <ControlsPage /> },
])

createRoot(document.getElementById('root') as HTMLElement).render(
  <StrictMode>
    <ThemeProvider>
      <RuntimeConfigProvider>
        {/* Provide a single shared ROS connection to all routes */}
        <RosProvider>
          <RouterProvider router={router} />
        </RosProvider>
      </RuntimeConfigProvider>
    </ThemeProvider>
  </StrictMode>,
)
