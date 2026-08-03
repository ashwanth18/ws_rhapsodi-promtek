import { NavLink, Outlet } from 'react-router-dom'
import {
  Activity,
  HeartPulse,
  KeyRound,
  LayoutDashboard,
  Package,
  Server,
  Settings,
} from 'lucide-react'
import { useState } from 'react'
import { getToken, setToken } from '../lib/api'
import { APP_VERSION } from '../lib/version'
import { Button } from '../components/ui'
import { cn } from '../lib/cn'

const NAV = [
  { to: '/', label: 'Devices', icon: Server, end: true },
  { to: '/monitoring', label: 'Monitoring', icon: HeartPulse, end: false },
  { to: '/dashboards', label: 'Dashboards', icon: LayoutDashboard, end: false },
  { to: '/releases', label: 'Releases', icon: Package, end: false },
  { to: '/deployments', label: 'Deployments', icon: Activity, end: false },
  { to: '/settings', label: 'Settings', icon: Settings, end: false },
]

export default function Layout() {
  const [tokenDraft, setTokenDraft] = useState(getToken())
  const [showSession, setShowSession] = useState(false)

  return (
    <div className="flex min-h-full">
      <aside className="flex w-60 shrink-0 flex-col border-r border-[var(--border)] bg-[var(--surface-1)]">
        <div className="border-b border-[var(--border)] px-4 py-5">
          <div className="font-display text-lg font-semibold tracking-tight">
            Rhapsodi Fleet
          </div>
          <div className="mt-1 text-xs text-[var(--text-muted)]">
            Fleet management · deploy · observe
          </div>
          <p className="mt-2 text-[11px] leading-snug text-[var(--text-faint)]">
            Control plane for the fleet. Open each device’s robot dashboard for cell operator UI.
          </p>
        </div>
        <nav className="flex flex-1 flex-col gap-1 p-3">
          {NAV.map((item) => (
            <NavLink
              key={item.to}
              to={item.to}
              end={item.end}
              className={({ isActive }) =>
                cn(
                  'flex items-center gap-2 rounded-[var(--radius-sm)] px-3 py-2 text-sm transition',
                  isActive
                    ? 'bg-[var(--accent)]/15 text-[var(--accent)]'
                    : 'text-[var(--text-secondary)] hover:bg-white/5',
                )
              }
            >
              <item.icon className="h-4 w-4" />
              {item.label}
            </NavLink>
          ))}
        </nav>
        <div className="border-t border-[var(--border)] p-3">
          <Button
            variant="ghost"
            className="w-full justify-start"
            onClick={() => setShowSession((v) => !v)}
          >
            <KeyRound className="h-4 w-4" />
            Session token
          </Button>
          {showSession ? (
            <div className="mt-2 space-y-2">
              <input
                className="w-full rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-2 py-1.5 text-xs"
                value={tokenDraft}
                onChange={(e) => setTokenDraft(e.target.value)}
                placeholder="Bearer for this browser"
                type="password"
                autoComplete="off"
              />
              <Button
                size="sm"
                className="w-full"
                onClick={() => {
                  setToken(tokenDraft.trim())
                  setShowSession(false)
                }}
              >
                Use in browser
              </Button>
            </div>
          ) : null}
          <div className="mt-3 px-1 text-[11px] text-[var(--text-faint)]">
            Fleet Console v{APP_VERSION}
          </div>
        </div>
      </aside>
      <main className="flex-1 overflow-auto p-6 lg:p-8">
        <Outlet />
      </main>
    </div>
  )
}
