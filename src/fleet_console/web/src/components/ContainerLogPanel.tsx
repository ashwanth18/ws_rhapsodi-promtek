import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import {
  ArrowDown,
  Loader2,
  Pause,
  Play,
  RefreshCw,
  Search,
  Terminal,
} from 'lucide-react'
import { api, type LogLine } from '../lib/api'
import { Button } from './ui'
import { cn } from '../lib/cn'

type LineKind = 'error' | 'warn' | 'ok' | 'info' | 'plain'

function classifyLine(line: string): LineKind {
  if (/fatal:|ERROR|FAILED|failed=|Traceback|Exception|CRITICAL/i.test(line))
    return 'error'
  if (/WARNING|WARN|rolled.?back/i.test(line)) return 'warn'
  if (/Health OK|success|INFO|Started/i.test(line)) return 'ok'
  if (/^\s*(DEBUG|TRACE)/i.test(line)) return 'info'
  return 'plain'
}

const LINE_CLASS: Record<LineKind, string> = {
  error: 'text-rose-300',
  warn: 'text-amber-300',
  ok: 'text-emerald-300/90',
  info: 'text-slate-300',
  plain: 'text-slate-400',
}

function formatTs(ts: number): string {
  try {
    return new Date(ts * 1000).toLocaleTimeString()
  } catch {
    return ''
  }
}

const SINCE_OPTS = [
  { value: '15m', label: '15m' },
  { value: '1h', label: '1h' },
  { value: '6h', label: '6h' },
  { value: '24h', label: '24h' },
]

export default function ContainerLogPanel({
  deviceId,
  defaultContainer,
  compact,
}: {
  deviceId: string
  defaultContainer?: string
  compact?: boolean
}) {
  const [containers, setContainers] = useState<string[]>([])
  const [container, setContainer] = useState(defaultContainer || '')
  const [since, setSince] = useState('15m')
  const [filter, setFilter] = useState('')
  const [q, setQ] = useState('')
  const [lines, setLines] = useState<LogLine[]>([])
  const [loading, setLoading] = useState(false)
  const [error, setError] = useState<string | null>(null)
  const [live, setLive] = useState(true)
  const [autoScroll, setAutoScroll] = useState(true)
  const scrollerRef = useRef<HTMLDivElement | null>(null)
  const bottomRef = useRef<HTMLDivElement | null>(null)
  const lastTsRef = useRef<number | null>(null)

  const loadContainers = useCallback(async () => {
    try {
      const payload = await api.deviceLogContainers(deviceId)
      setContainers(payload.containers)
      if (!container && payload.containers.length) {
        // Prefer common services if present.
        const prefer = [
          'condor-agent',
          'condor_agent',
          'backend',
          'orchestrator',
          'data_collection',
          'dashboard',
        ]
        const hit =
          prefer.find((p) =>
            payload.containers.some((c) => c === p || c.includes(p)),
          ) || payload.containers[0]
        setContainer(hit)
      }
    } catch {
      /* Loki may not be up yet */
    }
  }, [deviceId, container])

  const fetchLogs = useCallback(
    async (opts?: { append?: boolean }) => {
      if (!deviceId) return
      const append = Boolean(opts?.append)
      // Only show spinner for full window reloads — live appends stay quiet.
      if (!append) setLoading(true)
      setError(null)
      try {
        const start =
          append && lastTsRef.current != null
            ? lastTsRef.current + 0.001
            : undefined
        const payload = await api.deviceLogs(deviceId, {
          container: container || undefined,
          q: q || undefined,
          since: start == null ? since : undefined,
          start,
          limit: 500,
          direction: 'forward',
        })
        if (append) {
          setLines((prev) => {
            const next = [...prev, ...payload.lines]
            // Cap buffer
            return next.length > 2000 ? next.slice(-2000) : next
          })
        } else {
          setLines(payload.lines)
        }
        if (payload.lines.length) {
          lastTsRef.current = payload.lines[payload.lines.length - 1].ts
        }
      } catch (err) {
        setError(err instanceof Error ? err.message : String(err))
      } finally {
        if (!append) setLoading(false)
      }
    },
    [deviceId, container, q, since],
  )

  useEffect(() => {
    void loadContainers()
  }, [loadContainers])

  useEffect(() => {
    lastTsRef.current = null
    void fetchLogs({ append: false })
  }, [deviceId, container, since, q]) // eslint-disable-line react-hooks/exhaustive-deps

  useEffect(() => {
    if (!live) return
    const id = window.setInterval(() => {
      void fetchLogs({ append: true })
    }, 3000)
    return () => window.clearInterval(id)
  }, [live, fetchLogs])

  useEffect(() => {
    if (!autoScroll) return
    const el = scrollerRef.current
    if (el) el.scrollTop = el.scrollHeight
    else bottomRef.current?.scrollIntoView({ behavior: 'auto' })
  }, [lines, autoScroll])

  const filtered = useMemo(() => {
    const needle = filter.trim().toLowerCase()
    if (!needle) return lines
    return lines.filter(
      (l) =>
        l.text.toLowerCase().includes(needle) ||
        l.container.toLowerCase().includes(needle),
    )
  }, [lines, filter])

  return (
    <div className="overflow-hidden rounded-[var(--radius-md)] border border-[var(--border)] bg-[#04060a]">
      <div className="flex flex-wrap items-center gap-2 border-b border-[var(--border)] bg-[#0d121a] px-3 py-2">
        <Terminal className="h-4 w-4 text-sky-300" />
        <select
          className="rounded border border-[var(--border)] bg-[var(--surface-2)] px-2 py-1 font-mono text-xs"
          value={container}
          onChange={(e) => setContainer(e.target.value)}
        >
          <option value="">All containers</option>
          {containers.map((c) => (
            <option key={c} value={c}>
              {c}
            </option>
          ))}
        </select>
        <select
          className="rounded border border-[var(--border)] bg-[var(--surface-2)] px-2 py-1 text-xs"
          value={since}
          onChange={(e) => {
            lastTsRef.current = null
            setSince(e.target.value)
          }}
          title="Time window (reloads logs; live mode keeps appending)"
        >
          {SINCE_OPTS.map((o) => (
            <option key={o.value} value={o.value}>
              {o.label}
            </option>
          ))}
        </select>
        <form
          className="flex min-w-[10rem] flex-1 items-center gap-1"
          onSubmit={(e) => {
            e.preventDefault()
            lastTsRef.current = null
            void fetchLogs({ append: false })
          }}
        >
          <Search className="h-3.5 w-3.5 text-slate-500" />
          <input
            className="w-full bg-transparent text-xs text-slate-300 outline-none placeholder:text-slate-600"
            placeholder="Loki line filter…"
            value={q}
            onChange={(e) => setQ(e.target.value)}
          />
        </form>
        <Button
          size="sm"
          variant="ghost"
          title={live ? 'Pause live tail' : 'Resume live tail'}
          onClick={() => setLive((v) => !v)}
        >
          {live ? <Pause className="h-3.5 w-3.5" /> : <Play className="h-3.5 w-3.5" />}
        </Button>
        <Button
          size="sm"
          variant="ghost"
          title="Jump to latest"
          onClick={() => {
            setAutoScroll(true)
            bottomRef.current?.scrollIntoView({ behavior: 'smooth' })
          }}
        >
          <ArrowDown className="h-3.5 w-3.5" />
        </Button>
        <Button
          size="sm"
          variant="ghost"
          title="Refresh"
          onClick={() => {
            lastTsRef.current = null
            void fetchLogs({ append: false })
          }}
        >
          {loading ? (
            <Loader2 className="h-3.5 w-3.5 animate-spin" />
          ) : (
            <RefreshCw className="h-3.5 w-3.5" />
          )}
        </Button>
      </div>

      <div className="flex items-center gap-2 border-b border-[var(--border)] bg-[#070a10] px-3 py-1.5">
        <Search className="h-3.5 w-3.5 text-slate-500" />
        <input
          className="w-full bg-transparent text-xs text-slate-300 outline-none placeholder:text-slate-600"
          placeholder="Filter displayed lines…"
          value={filter}
          onChange={(e) => setFilter(e.target.value)}
        />
        <span className="shrink-0 font-mono text-[10px] text-slate-600">
          {filtered.length}/{lines.length}
        </span>
      </div>

      {error ? (
        <div className="px-3 py-2 text-xs text-rose-300">
          {error}
          <span className="ml-2 text-slate-500">
            (Is Loki up? Check Settings → Loki URL)
          </span>
        </div>
      ) : null}

      <div
        ref={scrollerRef}
        onScroll={() => {
          const el = scrollerRef.current
          if (!el) return
          const nearBottom = el.scrollHeight - el.scrollTop - el.clientHeight < 48
          if (nearBottom && !autoScroll) setAutoScroll(true)
          if (!nearBottom && autoScroll) setAutoScroll(false)
        }}
        className={cn(
          'overflow-auto overscroll-contain bg-[#030508] font-mono text-[11px] leading-[1.55]',
          compact ? 'max-h-[280px]' : 'max-h-[440px]',
        )}
      >
        {filtered.length === 0 ? (
          <div className="px-4 py-10 text-center text-xs text-slate-600">
            {loading
              ? 'Loading logs…'
              : 'No log lines yet. Promtail ships container stdout/stderr to Loki.'}
          </div>
        ) : (
          <div className="min-w-full py-1">
            {filtered.map((line, i) => {
              const kind = classifyLine(line.text)
              return (
                <div
                  key={`${line.ts}-${i}-${line.text.slice(0, 24)}`}
                  className="group flex hover:bg-white/[0.03]"
                >
                  <span className="sticky left-0 w-16 shrink-0 select-none border-r border-white/[0.04] bg-[#030508] px-1 text-right text-[10px] text-slate-600">
                    {formatTs(line.ts)}
                  </span>
                  {!container ? (
                    <span className="w-28 shrink-0 truncate px-2 text-[10px] text-sky-500/80">
                      {line.container || '—'}
                    </span>
                  ) : null}
                  <span
                    className={cn(
                      'whitespace-pre-wrap break-all px-3 py-0.5',
                      LINE_CLASS[kind],
                    )}
                  >
                    {line.text || ' '}
                  </span>
                </div>
              )
            })}
            <div ref={bottomRef} />
          </div>
        )}
      </div>

      <div className="flex items-center justify-between border-t border-[var(--border)] bg-[#070a10] px-3 py-1.5 text-[10px] text-slate-500">
        <span>
          {live ? 'Live · polling Loki every 3s' : 'Paused'}
          {autoScroll ? '' : ' · scroll locked'}
        </span>
        <span className="font-mono">{lines.length} lines</span>
      </div>
    </div>
  )
}
