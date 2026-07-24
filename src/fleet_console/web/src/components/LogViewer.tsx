import { useEffect, useRef, useState } from 'react'
import { api, logsStreamUrl } from '../lib/api'
import { Button } from './ui'

export default function LogViewer({
  deploymentId,
  onStatus,
}: {
  deploymentId: number | null
  onStatus?: (status: string) => void
}) {
  const [lines, setLines] = useState<string[]>([])
  const [status, setStatus] = useState<string>('idle')
  const bottomRef = useRef<HTMLDivElement | null>(null)

  useEffect(() => {
    if (!deploymentId) {
      setLines([])
      setStatus('idle')
      return
    }
    let cancelled = false
    setLines([])
    setStatus('connecting')

    api
      .getLogs(deploymentId)
      .then((payload) => {
        if (cancelled) return
        setLines(payload.log ? payload.log.split('\n') : [])
        setStatus(payload.status)
        onStatus?.(payload.status)
      })
      .catch(() => {
        /* stream will fill */
      })

    const es = new EventSource(logsStreamUrl(deploymentId))
    es.onmessage = (ev) => {
      setLines((prev) => [...prev, ev.data])
      setStatus((s) => (s === 'connecting' ? 'running' : s))
    }
    es.addEventListener('done', (ev) => {
      const next = (ev as MessageEvent).data || 'done'
      setStatus(next)
      onStatus?.(next)
      es.close()
    })
    es.onerror = () => {
      // Browser will retry; keep open unless finished.
    }
    return () => {
      cancelled = true
      es.close()
    }
  }, [deploymentId, onStatus])

  useEffect(() => {
    bottomRef.current?.scrollIntoView({ behavior: 'smooth' })
  }, [lines])

  if (!deploymentId) {
    return (
      <div className="rounded-[var(--radius-md)] border border-dashed border-[var(--border)] px-4 py-10 text-center text-sm text-[var(--text-muted)]">
        Start a provision or deploy to stream Ansible logs here.
      </div>
    )
  }

  return (
    <div className="overflow-hidden rounded-[var(--radius-md)] border border-[var(--border)] bg-[#05070a]">
      <div className="flex items-center justify-between border-b border-[var(--border)] px-3 py-2">
        <div className="text-xs text-[var(--text-muted)]">
          Deployment #{deploymentId} · {status}
        </div>
        <Button
          size="sm"
          variant="ghost"
          onClick={() =>
            api.getLogs(deploymentId).then((p) => setLines(p.log.split('\n')))
          }
        >
          Refresh
        </Button>
      </div>
      <pre className="max-h-[420px] overflow-auto p-3 text-xs leading-5 text-emerald-200/90">
        {lines.join('\n')}
        <div ref={bottomRef} />
      </pre>
    </div>
  )
}
