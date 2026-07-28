import { ReactNode, useCallback, useEffect, useRef, useState } from 'react'
import { Link, Outlet, useLocation } from 'react-router-dom'
import {
  Activity,
  ChevronLeft,
  ChevronRight,
  History,
  Layers,
  LayoutDashboard,
  MapPin,
  Settings,
  SlidersHorizontal,
  SunMoon,
} from 'lucide-react'
import { Toaster, toast } from 'sonner'
import Button from '../components/ui/button'
import StatusBadge from '../components/ui/StatusBadge'
import SettingsDrawer from '../components/SettingsDrawer'
import { useRuntimeConfig } from '../config/RuntimeConfig'
import { ThemePreference, useTheme } from '../theme/ThemeContext'
import { useConnectionStatus } from '../hooks/useConnectionStatus'
import { useShellTelemetry } from '../hooks/useShellTelemetry'

const SIDEBAR_OPEN_STORAGE_KEY = 'rhapsodi.sidebarOpen'

type ActiveRobotRun = {
  run_id: number
  event_id: string | null
  batch_id: string | null
  weightment_id: number
  status: string | null
  ingredient_id: string | null
  ingredient_name: string | null
  target_weight_kg: number | null
  location_code: string | null
  location_id: number | null
  started_at: string | null
  start_time: string | null
}

type NavItem = {
  to: string
  label: string
  icon: ReactNode
  group: 'operate' | 'analyze' | 'system'
}

const NAV_ITEMS: NavItem[] = [
  { to: '/', label: 'Operations', icon: <LayoutDashboard className="h-4 w-4" />, group: 'operate' },
  { to: '/batches', label: 'Batches', icon: <Layers className="h-4 w-4" />, group: 'operate' },
  { to: '/logs', label: 'Run History', icon: <History className="h-4 w-4" />, group: 'analyze' },
  { to: '/stock-location', label: 'Locations', icon: <MapPin className="h-4 w-4" />, group: 'system' },
  { to: '/controls', label: 'Controls', icon: <SlidersHorizontal className="h-4 w-4" />, group: 'system' },
]

function robotStateTone(state: string): 'good' | 'warn' | 'bad' | 'idle' {
  if (state === 'running' || state === 'starting') return 'warn'
  if (state === 'succeeded') return 'good'
  if (state === 'failed' || state === 'mes_send_failed') return 'bad'
  return 'idle'
}

function robotStateLabel(state: string) {
  if (state === 'running') return 'Running'
  if (state === 'starting') return 'Starting'
  if (state === 'awaiting_processing') return 'Processing'
  if (state === 'succeeded') return 'Succeeded'
  if (state === 'failed') return 'Error'
  if (state === 'mes_send_failed') return 'MES Error'
  return 'Idle'
}

export default function SidebarLayout({ children }: { children?: ReactNode }) {
  const [open, setOpenState] = useState(() => {
    const stored = localStorage.getItem(SIDEBAR_OPEN_STORAGE_KEY)
    return stored !== 'false'
  })
  const [settingsOpen, setSettingsOpen] = useState(false)
  const { pathname } = useLocation()
  const { apiBase } = useRuntimeConfig()
  const { themePreference, setThemePreference } = useTheme()
  const { runState, weightStale } = useShellTelemetry()
  const { apiStatus, rosStatus, hostName } = useConnectionStatus(weightStale)
  const [activeRun, setActiveRun] = useState<ActiveRobotRun | null>(null)
  const prevRosStatus = useRef(rosStatus)

  const setOpen = (value: boolean) => {
    setOpenState(value)
    localStorage.setItem(SIDEBAR_OPEN_STORAGE_KEY, String(value))
  }

  const loadActiveRun = useCallback(async () => {
    const res = await fetch(`${apiBase}/robot_weightment_runs/active`)
    if (!res.ok) throw new Error('Failed to load active robot run')
    const json = await res.json()
    setActiveRun(json.active || null)
  }, [apiBase])

  useEffect(() => {
    void loadActiveRun()
    const stream = new EventSource(`${apiBase}/robot_weightment_runs/active/stream`)
    stream.onmessage = (event) => {
      try {
        const json = JSON.parse(event.data)
        setActiveRun(json.active || null)
      } catch {
        // ignore malformed payloads
      }
    }
    stream.onerror = () => void loadActiveRun()
    return () => stream.close()
  }, [apiBase, loadActiveRun])

  useEffect(() => {
    if (prevRosStatus.current === 'connected' && rosStatus === 'disconnected') {
      toast.error('rosbridge disconnected')
    }
    prevRosStatus.current = rosStatus
  }, [rosStatus])

  const nextTheme: ThemePreference =
    themePreference === 'auto' ? 'dark' : themePreference === 'dark' ? 'light' : 'auto'

  const grouped = {
    operate: NAV_ITEMS.filter((i) => i.group === 'operate'),
    analyze: NAV_ITEMS.filter((i) => i.group === 'analyze'),
    system: NAV_ITEMS.filter((i) => i.group === 'system'),
  }

  const renderNavGroup = (title: string, items: NavItem[]) => (
    <div className="space-y-1">
      {open && (
        <div className="px-3 pb-1 text-[10px] font-semibold uppercase tracking-[0.14em] text-[var(--text-faint)]">
          {title}
        </div>
      )}
      {items.map((item) => {
        const active = pathname === item.to || (item.to !== '/' && pathname.startsWith(item.to))
        return (
          <Link
            key={item.to}
            to={item.to}
            title={item.label}
            className={`flex items-center rounded-[var(--radius-sm)] text-sm transition-colors ${
              open ? 'gap-3 px-3 py-2.5' : 'mx-auto h-10 w-10 justify-center'
            } ${
              active
                ? 'bg-[var(--accent-muted)] text-[var(--accent)]'
                : 'text-[var(--text-muted)] hover:bg-[var(--hover-surface)] hover:text-[var(--text-primary)]'
            }`}
          >
            {item.icon}
            {open && <span>{item.label}</span>}
          </Link>
        )
      })}
    </div>
  )

  const effectiveState = activeRun?.status || runState
  const linksUnhealthy =
    apiStatus === 'disconnected' ||
    rosStatus === 'disconnected' ||
    apiStatus === 'connecting' ||
    rosStatus === 'connecting'
  const onControls = pathname === '/controls' || pathname.startsWith('/controls/')

  return (
    <div className="flex min-h-screen bg-[var(--bg)] text-[var(--text-primary)]">
      <Toaster position="top-right" richColors closeButton />
      <aside
        className={`flex flex-col border-r border-[var(--border)] bg-[var(--surface-1)] transition-all duration-200 ${
          open ? 'w-56' : 'w-[68px]'
        }`}
      >
        <div className={`flex h-14 items-center ${open ? 'px-4' : 'justify-center'}`}>
          {open ? (
            <div className="font-display text-sm font-bold tracking-tight">Rhapsodi Ops</div>
          ) : (
            <Activity className="h-5 w-5 text-[var(--accent)]" />
          )}
        </div>
        <nav className="flex-1 space-y-5 overflow-y-auto px-2 py-3">
          {renderNavGroup('Operate', grouped.operate)}
          {renderNavGroup('Analyze', grouped.analyze)}
          {renderNavGroup('System', grouped.system)}
        </nav>
        <div className="border-t border-[var(--border)] p-2">
          <Button
            variant="ghost"
            size="sm"
            onClick={() => setOpen(!open)}
            className={open ? 'w-full justify-start' : 'mx-auto'}
            aria-label={open ? 'Collapse sidebar' : 'Expand sidebar'}
          >
            {open ? <ChevronLeft className="h-4 w-4" /> : <ChevronRight className="h-4 w-4" />}
            {open && <span>Collapse</span>}
          </Button>
        </div>
      </aside>

      <div className="flex min-w-0 flex-1 flex-col">
        <header className="sticky top-0 z-20 border-b border-[var(--border)] bg-[var(--surface-1)]/95 backdrop-blur-sm">
          <div className="flex flex-wrap items-center justify-between gap-3 px-5 py-3">
            <div className="flex flex-wrap items-center gap-2">
              <StatusBadge
                label={robotStateLabel(effectiveState)}
                tone={robotStateTone(effectiveState)}
                pulse={effectiveState === 'running' || effectiveState === 'starting'}
              />
              {/* Link health lives on Controls (Signal Deck); sticky only nudges when something is down. */}
              {linksUnhealthy && !onControls ? (
                <Link to="/controls" title="Open Cell Signal Deck">
                  <StatusBadge label="Links — open Controls" tone="warn" pulse />
                </Link>
              ) : null}
            </div>
            <div className="flex flex-wrap items-center gap-2">
              {activeRun && (
                <Link
                  to={
                    activeRun.event_id
                      ? `/batches/${encodeURIComponent(activeRun.event_id)}?weightmentId=${activeRun.weightment_id}`
                      : '/batches'
                  }
                  className="hidden rounded-[var(--radius-sm)] border border-[var(--border)] bg-[var(--surface-2)] px-3 py-1.5 text-xs text-[var(--text-secondary)] hover:border-[var(--accent)] md:inline-flex"
                >
                  Batch {activeRun.batch_id ?? '—'} · Wt {activeRun.weightment_id} ·{' '}
                  {activeRun.ingredient_name ?? activeRun.ingredient_id ?? '—'}
                </Link>
              )}
              <Button
                variant="ghost"
                size="sm"
                onClick={() => setThemePreference(nextTheme)}
                title={`Theme: ${themePreference}`}
              >
                <SunMoon className="h-4 w-4" />
              </Button>
              <Button variant="outline" size="sm" onClick={() => setSettingsOpen(true)}>
                <Settings className="h-4 w-4" />
                {open && <span className="hidden sm:inline">Settings</span>}
              </Button>
            </div>
          </div>
        </header>
        <main className="flex-1">{children ?? <Outlet />}</main>
      </div>

      <SettingsDrawer
        open={settingsOpen}
        onClose={() => setSettingsOpen(false)}
        apiStatus={apiStatus}
        rosStatus={rosStatus}
        hostName={hostName}
      />
    </div>
  )
}
