import { useEffect, useMemo, useRef, useState } from 'react'
import {
  ArrowDown,
  CheckCircle2,
  Circle,
  Copy,
  Download,
  Loader2,
  Pause,
  Play,
  RefreshCw,
  Search,
  Square,
  Terminal,
  XCircle,
} from 'lucide-react'
import { api, logsStreamUrl, type Deployment } from '../lib/api'
import { Button, StatusBadge, deployTone } from './ui'
import { cn } from '../lib/cn'

type StageId =
  | 'queued'
  | 'connect'
  | 'checkout'
  | 'configure'
  | 'pull'
  | 'start'
  | 'health'
  | 'agent'
  | 'done'

type StageDef = { id: StageId; label: string; patterns: RegExp[] }

const STAGES: StageDef[] = [
  {
    id: 'queued',
    label: 'Queued',
    patterns: [/PLAY \[/, /Starting/, /Desired state/, /workflow_dispatch/i],
  },
  {
    id: 'connect',
    label: 'Connect',
    patterns: [/Gathering Facts/i, /TASK \[.*[Ff]act/, /SSH/, /ok: \[.*\]/],
  },
  {
    id: 'checkout',
    label: 'Checkout',
    patterns: [/deploy bundle/i, /git (clone|fetch|checkout)/i, /Checkout deploy/i],
  },
  {
    id: 'configure',
    label: 'Configure',
    patterns: [
      /device\.yaml/i,
      /robot-prod\.env/i,
      /Apply.*profile/i,
      /fleet-agent/i,
      /Template/i,
      /POSTGRES_PASSWORD/,
    ],
  },
  {
    id: 'pull',
    label: 'Pull images',
    patterns: [/docker (compose )?pull/i, /Pulling/i, /Downloaded newer image/i],
  },
  {
    id: 'start',
    label: 'Start stack',
    patterns: [
      /compose .* up/i,
      /Creating/i,
      /Started/i,
      /Recreate stack/i,
      /systemd/i,
      /enable fleet-agent/i,
    ],
  },
  {
    id: 'health',
    label: 'Health check',
    patterns: [/Wait for backend health/i, /Health OK/i, /\/health/, /uri/i],
  },
  {
    id: 'agent',
    label: 'Agent',
    patterns: [
      /fleet-agent/i,
      /reconcile/i,
      /Applying release/i,
      /Already at desired/i,
      /Rolled back/i,
    ],
  },
  { id: 'done', label: 'Complete', patterns: [/PLAY RECAP/i, /failed=0/, /Provisioned /] },
]

function detectStageIndex(lines: string[]): number {
  let best = 0
  for (let i = 0; i < lines.length; i++) {
    const line = lines[i]
    for (let s = 0; s < STAGES.length; s++) {
      if (STAGES[s].patterns.some((p) => p.test(line))) {
        best = Math.max(best, s)
      }
    }
  }
  return best
}

function progressFor(status: string, stageIndex: number): number {
  if (status === 'success' || status === 'converged') return 100
  if (status === 'idle') return 0
  if (status === 'connecting') return 4
  const base = ((stageIndex + 1) / STAGES.length) * 92
  if (status === 'failed' || status === 'rolled_back') return Math.max(12, Math.min(92, base))
  return Math.max(6, Math.min(96, base))
}

type LineKind = 'error' | 'warn' | 'ok' | 'changed' | 'skip' | 'task' | 'play' | 'info' | 'plain'

function classifyLine(line: string): LineKind {
  if (/fatal:|ERROR|FAILED|failed=|Traceback|Exception/i.test(line)) return 'error'
  if (/WARNING|WARN|changed=|rolled.?back/i.test(line) && !/failed=/i.test(line)) return 'warn'
  if (/^ok:|changed=0.*failed=0|Health OK|success/i.test(line)) return 'ok'
  if (/^changed:|CHANGED/i.test(line)) return 'changed'
  if (/^skipping:|SKIPPING/i.test(line)) return 'skip'
  if (/^TASK \[|^TASK /i.test(line)) return 'task'
  if (/^PLAY \[|^PLAY /i.test(line)) return 'play'
  if (/^=> |^\*\*|INFO|Starting|Dispatch/i.test(line)) return 'info'
  return 'plain'
}

const LINE_CLASS: Record<LineKind, string> = {
  error: 'text-rose-300',
  warn: 'text-amber-300',
  ok: 'text-emerald-300/90',
  changed: 'text-sky-300',
  skip: 'text-slate-500',
  task: 'text-violet-300 font-medium',
  play: 'text-cyan-300 font-semibold',
  info: 'text-slate-300',
  plain: 'text-slate-400',
}

function formatElapsed(ms: number): string {
  const s = Math.floor(ms / 1000)
  const m = Math.floor(s / 60)
  const rem = s % 60
  if (m <= 0) return `${rem}s`
  return `${m}m ${rem.toString().padStart(2, '0')}s`
}

export default function LogViewer({
  deploymentId,
  deployment,
  onStatus,
}: {
  deploymentId: number | null
  deployment?: Deployment | null
  onStatus?: (status: string) => void
}) {
  const [lines, setLines] = useState<string[]>([])
  const [status, setStatus] = useState<string>('idle')
  const [filter, setFilter] = useState('')
  const [autoScroll, setAutoScroll] = useState(true)
  const [copied, setCopied] = useState(false)
  const [cancelling, setCancelling] = useState(false)
  const [startedAt, setStartedAt] = useState<number | null>(null)
  const [now, setNow] = useState(Date.now())
  const scrollerRef = useRef<HTMLDivElement | null>(null)
  const bottomRef = useRef<HTMLDivElement | null>(null)
  // Keep callback out of the stream effect deps — an inline onStatus from the
  // parent was reconnecting EventSource on every device poll / re-render.
  const onStatusRef = useRef(onStatus)
  onStatusRef.current = onStatus
  const pendingRef = useRef<string[]>([])
  const flushTimerRef = useRef<number | null>(null)

  useEffect(() => {
    if (!deploymentId) {
      setLines([])
      setStatus('idle')
      setStartedAt(null)
      return
    }
    let cancelled = false
    pendingRef.current = []
    if (flushTimerRef.current != null) {
      window.clearTimeout(flushTimerRef.current)
      flushTimerRef.current = null
    }
    setLines([])
    setStatus('connecting')
    setStartedAt(Date.now())
    setAutoScroll(true)

    const flushPending = () => {
      flushTimerRef.current = null
      if (cancelled || pendingRef.current.length === 0) return
      const chunk = pendingRef.current
      pendingRef.current = []
      setLines((prev) => [...prev, ...chunk])
      setStatus((s) => (s === 'connecting' ? 'running' : s))
    }

    api
      .getLogs(deploymentId)
      .then((payload) => {
        if (cancelled) return
        // Prefer snapshot only if we have not already streamed past it.
        setLines((prev) => {
          if (prev.length > 0) return prev
          return payload.log ? payload.log.split('\n') : []
        })
        setStatus(payload.status)
      })
      .catch(() => undefined)

    const es = new EventSource(logsStreamUrl(deploymentId))
    es.onmessage = (ev) => {
      if (cancelled) return
      pendingRef.current.push(ev.data)
      // Batch rapid SSE lines (~10fps) so the terminal does not thrash.
      if (flushTimerRef.current == null) {
        flushTimerRef.current = window.setTimeout(flushPending, 100)
      }
    }
    es.addEventListener('done', (ev) => {
      if (flushTimerRef.current != null) {
        window.clearTimeout(flushTimerRef.current)
        flushPending()
      }
      const next = (ev as MessageEvent).data || 'done'
      setStatus(next)
      onStatusRef.current?.(next)
      es.close()
    })
    es.onerror = () => undefined
    return () => {
      cancelled = true
      if (flushTimerRef.current != null) {
        window.clearTimeout(flushTimerRef.current)
        flushTimerRef.current = null
      }
      es.close()
    }
  }, [deploymentId])

  useEffect(() => {
    if (!deploymentId) return
    if (status === 'running' || status === 'connecting') {
      const id = window.setInterval(() => setNow(Date.now()), 1000)
      return () => window.clearInterval(id)
    }
  }, [deploymentId, status])

  useEffect(() => {
    if (!autoScroll) return
    const el = scrollerRef.current
    if (el) {
      // Instant jump — smooth scroll on every batch feels like a full refresh.
      el.scrollTop = el.scrollHeight
    } else {
      bottomRef.current?.scrollIntoView({ behavior: 'auto' })
    }
  }, [lines, autoScroll])

  const onScroll = () => {
    const el = scrollerRef.current
    if (!el) return
    const nearBottom = el.scrollHeight - el.scrollTop - el.clientHeight < 48
    if (nearBottom && !autoScroll) setAutoScroll(true)
    if (!nearBottom && autoScroll) setAutoScroll(false)
  }

  const stageIndex = useMemo(() => detectStageIndex(lines), [lines])
  const pct = progressFor(status, stageIndex)
  const running = status === 'running' || status === 'connecting'
  const elapsedMs =
    startedAt != null
      ? Math.max(0, (running ? now : Date.now()) - startedAt)
      : 0

  const filtered = useMemo(() => {
    const q = filter.trim().toLowerCase()
    if (!q) return lines.map((text, i) => ({ text, i }))
    return lines
      .map((text, i) => ({ text, i }))
      .filter((row) => row.text.toLowerCase().includes(q))
  }, [lines, filter])

  const errorCount = useMemo(
    () => lines.filter((l) => classifyLine(l) === 'error').length,
    [lines],
  )

  const copyLogs = async () => {
    await navigator.clipboard.writeText(lines.join('\n'))
    setCopied(true)
    window.setTimeout(() => setCopied(false), 1500)
  }

  const cancelJob = async () => {
    if (!deploymentId || cancelling) return
    setCancelling(true)
    try {
      const resp = await api.cancelDeployment(deploymentId)
      setStatus(resp.deployment.status)
      onStatusRef.current?.(resp.deployment.status)
      setLines((prev) => [...prev, '', '# cancelled by operator'])
    } catch (err) {
      setLines((prev) => [
        ...prev,
        '',
        `# cancel failed: ${err instanceof Error ? err.message : String(err)}`,
      ])
    } finally {
      setCancelling(false)
    }
  }

  const downloadLogs = () => {
    const blob = new Blob([lines.join('\n')], { type: 'text/plain;charset=utf-8' })
    const url = URL.createObjectURL(blob)
    const a = document.createElement('a')
    a.href = url
    a.download = `deployment-${deploymentId || 'log'}.log`
    a.click()
    URL.revokeObjectURL(url)
  }

  if (!deploymentId) {
    return (
      <div className="rounded-[var(--radius-md)] border border-dashed border-[var(--border)] bg-[var(--surface-1)] px-4 py-12 text-center">
        <Terminal className="mx-auto mb-3 h-8 w-8 text-[var(--text-muted)] opacity-50" />
        <p className="text-sm text-[var(--text-muted)]">
          Select a job or start flash-install / CI build to stream logs here.
        </p>
      </div>
    )
  }

  const action = deployment?.action || 'job'
  const barColor =
    status === 'failed'
      ? 'bg-[var(--status-bad-fg)]'
      : status === 'rolled_back'
        ? 'bg-[var(--status-warn-fg)]'
        : status === 'success' || status === 'converged'
          ? 'bg-[var(--status-good-fg)]'
          : 'bg-[var(--accent)]'

  return (
    <div className="overflow-hidden rounded-[var(--radius-md)] border border-[var(--border)] bg-[#04060a] shadow-[inset_0_1px_0_rgba(255,255,255,0.04)]">
      {/* Header */}
      <div className="border-b border-[var(--border)] bg-gradient-to-b from-[#0d121a] to-[#080b10] px-3 py-2.5">
        <div className="flex flex-wrap items-center justify-between gap-2">
          <div className="flex min-w-0 items-center gap-2.5">
            <div className="flex h-8 w-8 items-center justify-center rounded-lg bg-sky-500/10 text-sky-300">
              {running ? (
                <Loader2 className="h-4 w-4 animate-spin" />
              ) : status === 'failed' ? (
                <XCircle className="h-4 w-4 text-rose-400" />
              ) : status === 'success' || status === 'converged' ? (
                <CheckCircle2 className="h-4 w-4 text-emerald-400" />
              ) : (
                <Terminal className="h-4 w-4" />
              )}
            </div>
            <div className="min-w-0">
              <div className="flex flex-wrap items-center gap-2">
                <span className="font-mono text-xs text-slate-300">
                  #{deploymentId}
                </span>
                <span className="rounded bg-white/5 px-1.5 py-0.5 font-mono text-[10px] uppercase tracking-wide text-slate-400">
                  {action}
                </span>
                <StatusBadge
                  label={status}
                  tone={deployTone(status)}
                  pulse={running}
                />
              </div>
              <div className="mt-0.5 truncate text-[11px] text-slate-500">
                {STAGES[stageIndex]?.label || 'Starting'}
                {deployment?.image_tag ? ` · ${deployment.image_tag}` : ''}
                {deployment?.profile_id ? ` · ${deployment.profile_id}` : ''}
                {' · '}
                {formatElapsed(elapsedMs)}
                {errorCount > 0 ? ` · ${errorCount} error line${errorCount === 1 ? '' : 's'}` : ''}
              </div>
            </div>
          </div>

          <div className="flex flex-wrap items-center gap-1">
            {running ? (
              <Button
                size="sm"
                variant="danger"
                title="Cancel running job"
                disabled={cancelling}
                onClick={cancelJob}
              >
                <Square className="h-3.5 w-3.5 fill-current" />
                {cancelling ? 'Cancelling…' : 'Cancel'}
              </Button>
            ) : null}
            <Button
              size="sm"
              variant="ghost"
              title={autoScroll ? 'Pause auto-scroll' : 'Resume auto-scroll'}
              onClick={() => setAutoScroll((v) => !v)}
            >
              {autoScroll ? <Pause className="h-3.5 w-3.5" /> : <Play className="h-3.5 w-3.5" />}
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
            <Button size="sm" variant="ghost" title="Copy logs" onClick={copyLogs}>
              <Copy className="h-3.5 w-3.5" />
              {copied ? 'Copied' : null}
            </Button>
            <Button size="sm" variant="ghost" title="Download .log" onClick={downloadLogs}>
              <Download className="h-3.5 w-3.5" />
            </Button>
            <Button
              size="sm"
              variant="ghost"
              title="Refresh"
              onClick={() =>
                api.getLogs(deploymentId).then((p) => {
                  setLines(p.log ? p.log.split('\n') : [])
                  setStatus(p.status)
                })
              }
            >
              <RefreshCw className="h-3.5 w-3.5" />
            </Button>
          </div>
        </div>

        {/* Progress bar */}
        <div className="mt-3">
          <div className="mb-1.5 flex items-center justify-between text-[10px] uppercase tracking-wide text-slate-500">
            <span>Progress</span>
            <span className="font-mono text-slate-400">{Math.round(pct)}%</span>
          </div>
          <div className="relative h-1.5 overflow-hidden rounded-full bg-white/5">
            <div
              className={cn(
                'h-full rounded-full transition-[width] duration-500 ease-out',
                barColor,
                running && 'animate-pulse',
              )}
              style={{ width: `${pct}%` }}
            />
            {running ? (
              <div
                className="pointer-events-none absolute inset-y-0 w-24 animate-[shimmer_1.4s_ease_infinite] bg-gradient-to-r from-transparent via-white/25 to-transparent"
                style={{ left: `calc(${pct}% - 3rem)` }}
              />
            ) : null}
          </div>
        </div>

        {/* Stage rail */}
        <div className="mt-3 flex gap-1 overflow-x-auto pb-0.5">
          {STAGES.map((stage, idx) => {
            const done =
              idx < stageIndex ||
              status === 'success' ||
              status === 'converged' ||
              (idx === stageIndex && !running && status !== 'failed' && status !== 'idle')
            const current = idx === stageIndex && running
            const failedHere =
              (status === 'failed' || status === 'rolled_back') && idx === stageIndex
            return (
              <div
                key={stage.id}
                className={cn(
                  'flex min-w-[4.5rem] flex-1 flex-col items-center gap-1 rounded-md px-1 py-1',
                  current && 'bg-sky-500/10',
                  failedHere && 'bg-rose-500/10',
                )}
                title={stage.label}
              >
                <div className="flex items-center gap-1">
                  {failedHere ? (
                    <XCircle className="h-3 w-3 text-rose-400" />
                  ) : done ? (
                    <CheckCircle2 className="h-3 w-3 text-emerald-400" />
                  ) : current ? (
                    <Loader2 className="h-3 w-3 animate-spin text-sky-400" />
                  ) : (
                    <Circle className="h-3 w-3 text-slate-600" />
                  )}
                </div>
                <span
                  className={cn(
                    'truncate text-[9px] font-medium',
                    current
                      ? 'text-sky-300'
                      : done
                        ? 'text-slate-400'
                        : failedHere
                          ? 'text-rose-300'
                          : 'text-slate-600',
                  )}
                >
                  {stage.label}
                </span>
              </div>
            )
          })}
        </div>
      </div>

      {/* Filter bar */}
      <div className="flex items-center gap-2 border-b border-[var(--border)] bg-[#070a10] px-3 py-1.5">
        <Search className="h-3.5 w-3.5 text-slate-500" />
        <input
          className="w-full bg-transparent text-xs text-slate-300 outline-none placeholder:text-slate-600"
          placeholder="Filter log lines…"
          value={filter}
          onChange={(e) => setFilter(e.target.value)}
        />
        <span className="shrink-0 font-mono text-[10px] text-slate-600">
          {filtered.length}/{lines.length}
        </span>
      </div>

      {/* Terminal body */}
      <div
        ref={scrollerRef}
        onScroll={onScroll}
        className="max-h-[440px] overflow-auto overscroll-contain bg-[#030508] font-mono text-[11px] leading-[1.55]"
      >
        {filtered.length === 0 ? (
          <div className="px-4 py-10 text-center text-xs text-slate-600">
            {running ? 'Waiting for log output…' : 'No log lines match this filter.'}
          </div>
        ) : (
          <div className="min-w-full py-1">
            {filtered.map(({ text, i }) => {
              const kind = classifyLine(text)
              return (
                <div
                  key={`${i}-${text.slice(0, 24)}`}
                  className="group flex hover:bg-white/[0.03]"
                >
                  <span className="sticky left-0 w-12 shrink-0 select-none border-r border-white/[0.04] bg-[#030508] px-2 text-right text-[10px] text-slate-600 group-hover:bg-[#0a0e14]">
                    {i + 1}
                  </span>
                  <span
                    className={cn(
                      'whitespace-pre-wrap break-all px-3 py-0.5',
                      LINE_CLASS[kind],
                    )}
                  >
                    {text || ' '}
                  </span>
                </div>
              )
            })}
            <div ref={bottomRef} />
          </div>
        )}
      </div>

      {/* Footer */}
      <div className="flex items-center justify-between border-t border-[var(--border)] bg-[#070a10] px-3 py-1.5 text-[10px] text-slate-500">
        <span>
          {autoScroll ? 'Live · following output' : 'Paused · scroll locked'}
        </span>
        <span className="font-mono">
          {lines.length} lines · SSE stream
        </span>
      </div>

      <style>{`
        @keyframes shimmer {
          0% { transform: translateX(-40%); opacity: 0; }
          40% { opacity: 1; }
          100% { transform: translateX(120%); opacity: 0; }
        }
      `}</style>
    </div>
  )
}
