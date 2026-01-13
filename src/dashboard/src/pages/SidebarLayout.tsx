import { ReactNode, useState } from 'react'
import { Link, useLocation } from 'react-router-dom'
import Button from '../components/ui/button'

export default function SidebarLayout({ children }: { children: ReactNode }) {
  const [open, setOpen] = useState(true)
  const { pathname } = useLocation()
  const NavLink = ({ to, label }: { to: string; label: string }) => (
    <Link
      to={to}
      className={`block px-3 py-2 rounded-md text-sm ${pathname === to ? 'bg-white/10 text-white' : 'text-white/70 hover:text-white hover:bg-white/5'}`}
    >
      {label}
    </Link>
  )
  return (
    <div className="min-h-screen flex" style={{ background: 'linear-gradient(180deg, #0a0b0d 0%, #050607 65%)' }}>
      <aside className={`transition-all duration-200 border-r border-slate-800 ${open ? 'w-56' : 'w-14'} bg-[rgba(10,11,13,0.65)] backdrop-blur-md flex flex-col`}> 
        <div className={`h-14 w-full flex items-center ${open ? 'justify-between px-3' : 'justify-center px-0'}`}>
          <span className={`font-bold tracking-tight ${open ? 'opacity-100' : 'opacity-0'} transition-opacity`} style={{ fontFamily: 'Space Grotesk' }}>Robot UI</span>
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
            <NavLink to="/" label="Dashboard" />
            <NavLink to="/logs" label="Historical Logs" />
            <NavLink to="/controls" label="Controls & Sensors" />
          </nav>
        )}
      </aside>
      <main className="flex-1">{children}</main>
    </div>
  )
}


