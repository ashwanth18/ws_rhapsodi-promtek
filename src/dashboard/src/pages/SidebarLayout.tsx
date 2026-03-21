import { ReactNode, useCallback, useEffect, useState } from 'react'
import { Link, useLocation } from 'react-router-dom'
import GlassCard from '../components/GlassCard'
import Button from '../components/ui/button'
import ConnectionPanel from '../components/ConnectionPanel'
import { useRuntimeConfig } from '../config/RuntimeConfig'
import { ThemePreference, useTheme } from '../theme/ThemeContext'

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
  location_id: number | null
  location_code: string | null
  requested_at: string | null
  started_at: string | null
  finished_at: string | null
  start_time: string | null
  end_time: string | null
  actual_weight_kg: number | null
  energy_kwh: number | null
}

function SidebarToggleIcon({ open }: { open: boolean }) {
  return (
    <svg className="h-4 w-4" viewBox="0 0 24 24" fill="none" aria-hidden="true">
      <rect x="3" y="4" width="18" height="16" rx="2" stroke="currentColor" strokeWidth="2" />
      <path d="M9 4V20" stroke="currentColor" strokeWidth="2" />
      {open ? (
        <path
          d="M15 9L12 12L15 15"
          stroke="currentColor"
          strokeWidth="2"
          strokeLinecap="round"
          strokeLinejoin="round"
        />
      ) : (
        <path
          d="M12 9L15 12L12 15"
          stroke="currentColor"
          strokeWidth="2"
          strokeLinecap="round"
          strokeLinejoin="round"
        />
      )}
    </svg>
  )
}

function ThemeIcon({ preference }: { preference: ThemePreference }) {
  if (preference === 'light') {
    return (
      <svg className="h-4 w-4" viewBox="0 0 24 24" fill="none" aria-hidden="true">
        <circle cx="12" cy="12" r="4" stroke="currentColor" strokeWidth="2" />
        <path
          d="M12 2V5M12 19V22M4.93 4.93L7.05 7.05M16.95 16.95L19.07 19.07M2 12H5M19 12H22M4.93 19.07L7.05 16.95M16.95 7.05L19.07 4.93"
          stroke="currentColor"
          strokeWidth="2"
          strokeLinecap="round"
        />
      </svg>
    )
  }
  if (preference === 'dark') {
    return (
      <svg className="h-4 w-4" viewBox="0 0 24 24" fill="none" aria-hidden="true">
        <path
          d="M21 12.8A9 9 0 1111.2 3 7 7 0 0021 12.8z"
          stroke="currentColor"
          strokeWidth="2"
          strokeLinecap="round"
          strokeLinejoin="round"
        />
      </svg>
    )
  }
  return (
    <svg className="h-4 w-4" viewBox="0 0 24 24" fill="none" aria-hidden="true">
      <circle cx="12" cy="12" r="8.5" fill="#0f172a" stroke="currentColor" strokeWidth="1.5" />
      <path
        d="M12 3.5A8.5 8.5 0 0112 20.5C14.49 20.5 16.5 18.49 16.5 16C16.5 13.51 14.49 11.5 12 11.5C9.51 11.5 7.5 9.49 7.5 7C7.5 4.51 9.51 3.5 12 3.5Z"
        fill="#ffffff"
      />
      <circle cx="12" cy="7.5" r="1.4" fill="#0f172a" />
      <circle cx="12" cy="16.5" r="1.4" fill="#ffffff" stroke="#0f172a" strokeWidth="0.6" />
    </svg>
  )
}

function ToolbarExpandIcon({ expanded }: { expanded: boolean }) {
  return expanded ? (
    <svg className="h-4 w-4" viewBox="0 0 24 24" fill="none" aria-hidden="true">
      <path
        d="M8 4H4V8M16 4H20V8M8 20H4V16M16 20H20V16"
        stroke="currentColor"
        strokeWidth="2"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
    </svg>
  ) : (
    <svg className="h-4 w-4" viewBox="0 0 24 24" fill="none" aria-hidden="true">
      <path
        d="M9 4H4V9M15 4H20V9M9 20H4V15M15 20H20V15"
        stroke="currentColor"
        strokeWidth="2"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
    </svg>
  )
}

function DashboardIcon() {
  return (
    <svg className="h-4 w-4" viewBox="0 0 24 24" fill="none" aria-hidden="true">
      <path
        d="M4 13H10V20H4V13ZM14 4H20V10H14V4ZM14 13H20V20H14V13ZM4 4H10V10H4V4Z"
        stroke="currentColor"
        strokeWidth="2"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
    </svg>
  )
}

function HistoryIcon() {
  return (
    <svg className="h-4 w-4" viewBox="0 0 24 24" fill="none" aria-hidden="true">
      <path
        d="M3 12A9 9 0 1012 3a8.97 8.97 0 00-6.36 2.64L3 8M3 3v5h5"
        stroke="currentColor"
        strokeWidth="2"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
      <path
        d="M12 7V12L15 15"
        stroke="currentColor"
        strokeWidth="2"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
    </svg>
  )
}

function BatchIcon() {
  return (
    <svg className="h-4 w-4" viewBox="0 0 24 24" fill="none" aria-hidden="true">
      <path
        d="M12 3L20 7L12 11L4 7L12 3Z"
        stroke="currentColor"
        strokeWidth="2"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
      <path
        d="M4 12L12 16L20 12"
        stroke="currentColor"
        strokeWidth="2"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
      <path
        d="M4 17L12 21L20 17"
        stroke="currentColor"
        strokeWidth="2"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
    </svg>
  )
}

function LocationIcon() {
  return (
    <svg className="h-4 w-4" viewBox="0 0 24 24" fill="none" aria-hidden="true">
      <path
        d="M12 21S19 15.5 19 10A7 7 0 105 10c0 5.5 7 11 7 11Z"
        stroke="currentColor"
        strokeWidth="2"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
      <circle cx="12" cy="10" r="2.5" stroke="currentColor" strokeWidth="2" />
    </svg>
  )
}

function ControlsIcon() {
  return (
    <svg className="h-4 w-4" viewBox="0 0 24 24" fill="none" aria-hidden="true">
      <path
        d="M4 7H10M14 7H20M8 7C8 8.1 8.9 9 10 9C11.1 9 12 8.1 12 7C12 5.9 11.1 5 10 5C8.9 5 8 5.9 8 7ZM4 17H8M12 17H20M10 17C10 18.1 10.9 19 12 19C13.1 19 14 18.1 14 17C14 15.9 13.1 15 12 15C10.9 15 10 15.9 10 17Z"
        stroke="currentColor"
        strokeWidth="2"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
    </svg>
  )
}

export default function SidebarLayout({ children }: { children: ReactNode }) {
  const [open, setOpenState] = useState<boolean>(() => {
    const stored = localStorage.getItem(SIDEBAR_OPEN_STORAGE_KEY)
    if (stored === 'true') return true
    if (stored === 'false') return false
    return true
  })
  const [toolbarExpanded, setToolbarExpanded] = useState(true)
  const { pathname } = useLocation()
  const { apiBase } = useRuntimeConfig()
  const { themePreference, setThemePreference } = useTheme()
  const [activeRun, setActiveRun] = useState<ActiveRobotRun | null>(null)
  const NavLink = ({
    to,
    label,
    icon,
  }: {
    to: string
    label: string
    icon: ReactNode
  }) => (
    <Link
      to={to}
      title={open ? undefined : label}
      className={`block rounded-md text-sm ${
        pathname === to
          ? 'bg-[var(--surface-strong)] text-[var(--text-primary)]'
          : 'text-[var(--text-muted)] hover:bg-[var(--hover-surface)] hover:text-[var(--text-primary)]'
      } ${open ? 'px-3 py-2' : 'mx-auto flex h-10 w-10 items-center justify-center'}`}
    >
      <span className={`flex items-center ${open ? 'gap-2' : 'justify-center'}`}>
        {icon}
        <span
          className={`overflow-hidden whitespace-nowrap transition-all duration-300 ease-in-out ${
            open ? 'max-w-[160px] translate-x-0 opacity-100' : 'max-w-0 translate-x-1 opacity-0'
          }`}
        >
          {label}
        </span>
      </span>
    </Link>
  )
  const SidebarAction = ({
    onClick,
    icon,
    label,
    title,
  }: {
    onClick: () => void
    icon: ReactNode
    label: string
    title?: string
  }) => (
    <button
      type="button"
      onClick={onClick}
      title={title}
      className={`block w-full rounded-md text-sm transition-colors ${
        open
          ? 'px-3 py-2 text-left text-[var(--text-muted)] hover:bg-[var(--hover-surface)] hover:text-[var(--text-primary)]'
          : 'flex h-10 w-10 items-center justify-center text-[var(--text-muted)] hover:bg-[var(--hover-surface)] hover:text-[var(--text-primary)]'
      }`}
    >
      <span className={open ? 'flex items-center gap-2' : 'flex items-center justify-center'}>
        {icon}
        <span
          className={`overflow-hidden whitespace-nowrap transition-all duration-300 ease-in-out ${
            open ? 'max-w-[160px] translate-x-0 opacity-100' : 'max-w-0 translate-x-1 opacity-0'
          }`}
        >
          {label}
        </span>
      </span>
    </button>
  )
  const loadActiveRun = useCallback(async () => {
    const res = await fetch(`${apiBase}/robot_weightment_runs/active`)
    if (!res.ok) {
      throw new Error('Failed to load active robot run')
    }
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
        // Ignore malformed event payloads and wait for the next update.
      }
    }
    stream.onerror = () => {
      void loadActiveRun()
    }
    return () => {
      stream.close()
    }
  }, [apiBase, loadActiveRun])

  const liveStatusLabel =
    activeRun?.status === 'starting'
      ? 'Starting Weighment'
      : activeRun?.status === 'awaiting_processing'
        ? 'Finishing And Processing'
        : 'Live Weighing In Progress'
  const nextThemePreference: ThemePreference =
    themePreference === 'auto'
      ? 'dark'
      : themePreference === 'dark'
        ? 'light'
        : 'auto'
  const themeLabel =
    themePreference === 'auto'
      ? 'Auto Theme'
      : themePreference === 'dark'
        ? 'Dark Theme'
        : 'Light Theme'
  const setOpen = (value: boolean) => {
    setOpenState(value)
    localStorage.setItem(SIDEBAR_OPEN_STORAGE_KEY, String(value))
  }

  useEffect(() => {
    if (!activeRun) {
      setToolbarExpanded(true)
    }
  }, [activeRun])

  return (
    <div className="flex min-h-screen bg-[var(--bg)] text-[var(--text-primary)]">
      <aside
        className={`flex flex-col border-r border-[var(--border)] bg-[var(--surface-elevated)] backdrop-blur-md transition-all duration-200 ${
          open ? 'w-56' : 'w-16'
        }`}
      >
        <div
          className={`flex h-14 w-full items-center transition-all duration-300 ease-in-out ${
            open ? 'justify-between px-3' : 'justify-center px-2'
          }`}
        >
          <span
            className={`overflow-hidden whitespace-nowrap font-bold tracking-tight transition-all duration-300 ease-in-out ${
              open
                ? 'max-w-[140px] translate-x-0 opacity-100'
                : 'max-w-0 -translate-x-2 opacity-0'
            }`}
            style={{ fontFamily: 'Space Grotesk' }}
          >
            Rhapsodi Ops
          </span>
        </div>
        <div
          className={`px-2 pb-2 transition-all duration-300 ease-in-out ${
            open ? 'space-y-1' : 'flex flex-col items-center gap-2'
          }`}
        >
          <SidebarAction
            onClick={() => setOpen(!open)}
            icon={<SidebarToggleIcon open={open} />}
            label={open ? 'Collapse Sidebar' : 'Expand Sidebar'}
            title={open ? 'Collapse sidebar' : 'Expand sidebar'}
          />
          <SidebarAction
            onClick={() => setThemePreference(nextThemePreference)}
            icon={<ThemeIcon preference={themePreference} />}
            label={themeLabel}
            title={`${themeLabel} · click to switch to ${nextThemePreference}`}
          />
        </div>
        <nav
          className={`px-2 py-2 transition-all duration-300 ease-in-out ${
            open ? 'space-y-1' : 'space-y-2'
          }`}
        >
          <NavLink to="/" label="Operations Dashboard" icon={<DashboardIcon />} />
          <NavLink to="/logs" label="Run History" icon={<HistoryIcon />} />
          <NavLink to="/batches" label="Batches" icon={<BatchIcon />} />
          <NavLink to="/stock-location" label="Location Allocations" icon={<LocationIcon />} />
          <NavLink to="/controls" label="Controls & Sensors" icon={<ControlsIcon />} />
        </nav>
      </aside>
      <main className="flex-1 pb-20">
        <div className="px-6 pt-6">
          <ConnectionPanel />
        </div>
        {children}
        {activeRun && (
          <div className={`fixed bottom-3 right-3 z-30 ${open ? 'left-64' : 'left-20'}`}>
            <GlassCard className="border border-sky-400/25 px-4 py-3 shadow-xl shadow-sky-950/30 backdrop-blur">
              <div className="flex items-start justify-between gap-3">
                <div className="min-w-0">
                  <div className="text-[11px] font-semibold uppercase tracking-[0.18em] text-sky-300">
                    {liveStatusLabel}
                  </div>
                  {toolbarExpanded && (
                    <div className="mt-1 text-sm font-medium text-[var(--text-primary)]">
                      Batch {activeRun.batch_id ?? '—'} · Weightment {activeRun.weightment_id}
                    </div>
                  )}
                </div>
                <Button
                  size="sm"
                  variant="ghost"
                  onClick={() => setToolbarExpanded((current) => !current)}
                  aria-label={toolbarExpanded ? 'Collapse live weighing toolbar' : 'Expand live weighing toolbar'}
                  className="gap-2"
                >
                  <ToolbarExpandIcon expanded={toolbarExpanded} />
                  {toolbarExpanded ? 'Compact' : 'Expand'}
                </Button>
              </div>
              <div
                className={`overflow-hidden transition-all duration-300 ease-in-out ${
                  toolbarExpanded ? 'max-h-40 opacity-100 pt-3' : 'max-h-0 opacity-0'
                }`}
              >
                <div className="flex flex-col gap-3 md:flex-row md:items-center md:justify-between">
                  <div className="min-w-0">
                    <div className="text-xs text-[var(--text-muted)]">
                      Ingredient {activeRun.ingredient_name ?? activeRun.ingredient_id ?? '—'} ·
                      {' '}target {activeRun.target_weight_kg != null ? activeRun.target_weight_kg.toFixed(3) : '—'} kg
                      {' '}· loc {activeRun.location_code ?? activeRun.location_id ?? '—'}
                    </div>
                    <div className="mt-1 text-[11px] text-[var(--text-faint)]">
                      Started
                      {' '}
                      {activeRun.started_at ?? activeRun.start_time ? (
                        new Date(
                          activeRun.started_at ?? activeRun.start_time ?? ''
                        ).toLocaleTimeString([], {
                          hour: '2-digit',
                          minute: '2-digit',
                          second: '2-digit',
                          hour12: false,
                        })
                      ) : (
                        '—'
                      )}
                    </div>
                  </div>
                  <div className="flex flex-wrap items-center gap-2 self-start md:self-center">
                    <Link to="/">
                      <Button size="sm">Operations</Button>
                    </Link>
                    <Link
                      to={
                        activeRun.event_id
                          ? `/batches/${encodeURIComponent(activeRun.event_id)}?weightmentId=${activeRun.weightment_id}`
                          : '/batches'
                      }
                    >
                      <Button size="sm" variant="ghost">
                        Weighing Details
                      </Button>
                    </Link>
                  </div>
                </div>
              </div>
            </GlassCard>
          </div>
        )}
      </main>
    </div>
  )
}


