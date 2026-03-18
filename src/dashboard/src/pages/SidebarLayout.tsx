import { ReactNode, useCallback, useEffect, useState } from 'react'
import { Link, useLocation } from 'react-router-dom'
import GlassCard from '../components/GlassCard'
import Button from '../components/ui/button'
import ConnectionPanel from '../components/ConnectionPanel'
import { useRuntimeConfig } from '../config/RuntimeConfig'

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

function ChevronIcon({ open }: { open: boolean }) {
  return (
    <svg
      className={`h-4 w-4 transition-transform duration-300 ease-in-out ${
        open ? 'rotate-0' : '-rotate-90'
      }`}
      viewBox="0 0 24 24"
      fill="none"
      xmlns="http://www.w3.org/2000/svg"
      aria-hidden="true"
    >
      <path
        d="M6 9L12 15L18 9"
        stroke="currentColor"
        strokeWidth="2"
        strokeLinecap="round"
        strokeLinejoin="round"
      />
    </svg>
  )
}

export default function SidebarLayout({ children }: { children: ReactNode }) {
  const [open, setOpen] = useState(true)
  const [toolbarExpanded, setToolbarExpanded] = useState(true)
  const { pathname } = useLocation()
  const { apiBase } = useRuntimeConfig()
  const [activeRun, setActiveRun] = useState<ActiveRobotRun | null>(null)
  const NavLink = ({ to, label }: { to: string; label: string }) => (
    <Link
      to={to}
      className={`block px-3 py-2 rounded-md text-sm ${pathname === to ? 'bg-white/10 text-white' : 'text-white/70 hover:text-white hover:bg-white/5'}`}
    >
      {label}
    </Link>
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

  useEffect(() => {
    if (!activeRun) {
      setToolbarExpanded(true)
    }
  }, [activeRun])

  return (
    <div className="min-h-screen flex" style={{ background: 'linear-gradient(180deg, #0a0b0d 0%, #050607 65%)' }}>
      <aside className={`transition-all duration-200 border-r border-slate-800 ${open ? 'w-56' : 'w-14'} bg-[rgba(10,11,13,0.65)] backdrop-blur-md flex flex-col`}> 
        <div className={`h-14 w-full flex items-center ${open ? 'justify-between px-3' : 'justify-center px-0'}`}>
          <span className={`font-bold tracking-tight ${open ? 'opacity-100' : 'opacity-0'} transition-opacity`} style={{ fontFamily: 'Space Grotesk' }}>Rhapsodi Ops</span>
          <Button
            variant="ghost"
            size="sm"
            onClick={() => setOpen(!open)}
            aria-label="Toggle sidebar"
            className="w-8 h-8 rounded-md text-white/80 hover:bg-white/10"
          >
            ☰
          </Button>
        </div>
        {open && (
          <nav className="px-2 py-2 space-y-1">
            <NavLink to="/" label="Operations Dashboard" />
            <NavLink to="/logs" label="Run History" />
            <NavLink to="/webhook-weightments" label="Webhook Weightments" />
            <NavLink to="/stock-location" label="Stock Location" />
            <NavLink to="/controls" label="Controls & Sensors" />
          </nav>
        )}
      </aside>
      <main className="flex-1 pb-20">
        <div className="px-6 pt-6">
          <ConnectionPanel />
        </div>
        {children}
        {activeRun && (
          <div className={`fixed bottom-3 right-3 z-30 ${open ? 'left-64' : 'left-20'}`}>
            <GlassCard className="border border-sky-400/25 bg-slate-950/92 px-4 py-3 shadow-xl shadow-sky-950/30 backdrop-blur">
              <div className="flex items-start justify-between gap-3">
                <div className="min-w-0">
                  <div className="text-[11px] font-semibold uppercase tracking-[0.18em] text-sky-300">
                    {liveStatusLabel}
                  </div>
                  {toolbarExpanded && (
                    <div className="mt-1 text-sm font-medium text-white">
                      Batch {activeRun.batch_id ?? '—'} · Weightment {activeRun.weightment_id}
                    </div>
                  )}
                </div>
                <Button
                  size="sm"
                  variant="ghost"
                  onClick={() => setToolbarExpanded((current) => !current)}
                  aria-label={toolbarExpanded ? 'Collapse live weighing toolbar' : 'Expand live weighing toolbar'}
                >
                  <ChevronIcon open={toolbarExpanded} />
                </Button>
              </div>
              <div
                className={`overflow-hidden transition-all duration-300 ease-in-out ${
                  toolbarExpanded ? 'max-h-40 opacity-100 pt-3' : 'max-h-0 opacity-0'
                }`}
              >
                <div className="flex flex-col gap-3 md:flex-row md:items-center md:justify-between">
                  <div className="min-w-0">
                    <div className="text-xs text-white/65">
                      Ingredient {activeRun.ingredient_name ?? activeRun.ingredient_id ?? '—'} ·
                      {' '}target {activeRun.target_weight_kg != null ? activeRun.target_weight_kg.toFixed(3) : '—'} kg
                      {' '}· loc {activeRun.location_code ?? activeRun.location_id ?? '—'}
                    </div>
                    <div className="mt-1 text-[11px] text-white/45">
                      Started
                      {' '}
                      {activeRun.started_at ?? activeRun.start_time ? (
                        new Date(activeRun.started_at ?? activeRun.start_time ?? '').toLocaleTimeString()
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
                          ? `/webhook-weightments/${encodeURIComponent(activeRun.event_id)}?weightmentId=${activeRun.weightment_id}`
                          : '/webhook-weightments'
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


