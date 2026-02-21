import { StrictMode } from 'react'
import { createRoot } from 'react-dom/client'
import App from './App'
import './index.css'
import { createBrowserRouter, RouterProvider } from 'react-router-dom'
import DashboardPage from './pages/DashboardPage'
import LogsPage from './pages/LogsPage'
import ControlsPage from './pages/ControlsPage'
import TrainingPage from './pages/TrainingPage'
import { RosProvider } from './ros/RosContext'
import { RuntimeConfigProvider } from './config/RuntimeConfig'

const router = createBrowserRouter([
  { path: '/', element: <DashboardPage /> },
  { path: '/logs', element: <LogsPage /> },
  { path: '/controls', element: <ControlsPage /> },
  { path: '/training', element: <TrainingPage /> },
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
