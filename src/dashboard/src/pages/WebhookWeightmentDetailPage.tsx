import { useCallback, useEffect, useRef, useState } from 'react'
import { Link, useParams, useSearchParams } from 'react-router-dom'

import DateTimeText from '../components/DateTimeText'
import GlassCard from '../components/GlassCard'
import Button from '../components/ui/button'
import { useRuntimeConfig } from '../config/RuntimeConfig'
import SidebarLayout from './SidebarLayout'

type Summary = {
  event_id: string
  sent_utc: string | null
  batch_id: string | null
  completed: boolean
  batch_auto_run_enabled: boolean
  batch_run_in_progress: boolean
  next_weightment_id: number | null
  active_weightment_id?: number | null
  active_run_id?: number | null
  last_completed_weightment_id?: number | null
}

type Row = {
  weightment_id: number
  target_weight_kg: number | null
  actual_weight_kg: number | null
  completed: boolean
  stock_item_id: string | null
  ingredient_name: string | null
  location_id: number | null
  location_code: string | null
  start_time: string | null
  end_time: string | null
  energy_kwh: number | null
  robot_run_id: number | null
  robot_status: string | null
  robot_error: string | null
  robot_requested_at: string | null
  robot_started_at: string | null
  robot_finished_at: string | null
  robot_trace_run_id: string | null
  robot_processed_run_db_id: number | null
  robot_processed_id: number | null
  robot_live_completion_weight_kg: number | null
  robot_processed_final_weight_kg: number | null
  robot_processed_start_time: string | null
  robot_processed_end_time: string | null
  robot_processed_overshoot_g: number | null
  robot_processed_scoop_duration_s: number | null
  robot_processed_pour_duration_s: number | null
  robot_processed_settle_time_s: number | null
  robot_phase_events: Array<{ t_ns: number; phase: string }> | null
  robot_mcap_path: string | null
  robot_parquet_path: string | null
  robot_mes_weighment_sent: boolean
  robot_mes_batch_end_sent: boolean
}

type LiveStatusRow = Pick<
  Row,
  | 'weightment_id'
  | 'completed'
  | 'actual_weight_kg'
  | 'start_time'
  | 'end_time'
  | 'energy_kwh'
  | 'robot_run_id'
  | 'robot_status'
  | 'robot_error'
  | 'robot_requested_at'
  | 'robot_started_at'
  | 'robot_finished_at'
  | 'robot_trace_run_id'
  | 'robot_processed_id'
  | 'robot_mes_weighment_sent'
  | 'robot_mes_batch_end_sent'
>

function formatNumber(value: number | null, digits = 3): string {
  return value != null ? value.toFixed(digits) : '—'
}

function ChevronIcon({ open }: { open: boolean }) {
  return (
    <svg
      className={`h-4 w-4 transition-transform duration-300 ease-in-out ${
        open ? 'rotate-180' : 'rotate-0'
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

export default function WebhookWeightmentDetailPage() {
  const { apiBase } = useRuntimeConfig()
  const { eventId = '' } = useParams()
  const [searchParams] = useSearchParams()
  const decodedEventId = decodeURIComponent(eventId)
  const focusedWeightmentId = Number.parseInt(
    searchParams.get('weightmentId') || '',
    10
  )
  const [summary, setSummary] = useState<Summary | null>(null)
  const [rows, setRows] = useState<Row[]>([])
  const [loading, setLoading] = useState(false)
  const [runningId, setRunningId] = useState<number | null>(null)
  const [runningBatch, setRunningBatch] = useState(false)
  const [sendingId, setSendingId] = useState<number | null>(null)
  const [openRobotDetailIds, setOpenRobotDetailIds] = useState<number[]>([])
  const [robotDetailsInitialized, setRobotDetailsInitialized] = useState(false)
  const lastAutoScrolledWeightmentIdRef = useRef<number | null>(null)
  const incompleteRowsMissingLocation = rows.filter(
    (row) => !row.completed && row.location_id == null
  )
  const robotDetailRows = rows.filter(
    (row) =>
      row.robot_status ||
      row.robot_error ||
      row.robot_trace_run_id ||
      row.robot_processed_id != null ||
      row.robot_mcap_path ||
      row.robot_parquet_path
  )
  const hasActiveRobotRun =
    summary?.batch_auto_run_enabled ||
    rows.some((row) =>
      ['starting', 'running', 'awaiting_processing'].includes(row.robot_status ?? '')
    )

  const loadRows = useCallback(async () => {
    try {
      setLoading(true)
      const res = await fetch(
        `${apiBase}/webhook_weightments/${encodeURIComponent(decodedEventId)}`
      )
      const json = await res.json()
      setSummary(json.summary || null)
      setRows(json.rows || [])
    } finally {
      setLoading(false)
    }
  }, [apiBase, decodedEventId])

  const loadLiveStatus = useCallback(async () => {
    const res = await fetch(
      `${apiBase}/webhook_events/${encodeURIComponent(decodedEventId)}/live_status`
    )
    if (!res.ok) {
      throw new Error(`Failed to load live status for event ${decodedEventId}`)
    }
    const json = await res.json()
    if (json.summary) {
      setSummary((current) => (current ? { ...current, ...json.summary } : json.summary))
    }
    const liveRows = Array.isArray(json.rows) ? (json.rows as LiveStatusRow[]) : []
    if (!liveRows.length) {
      return
    }
    const liveRowMap = new Map(liveRows.map((row) => [row.weightment_id, row]))
    setRows((current) =>
      current.map((row) => {
        const liveRow = liveRowMap.get(row.weightment_id)
        return liveRow ? { ...row, ...liveRow } : row
      })
    )
  }, [apiBase, decodedEventId])

  useEffect(() => {
    if (decodedEventId) {
      setRobotDetailsInitialized(false)
      setOpenRobotDetailIds([])
      void loadRows()
    }
  }, [decodedEventId, loadRows])

  useEffect(() => {
    if (!decodedEventId || !hasActiveRobotRun) {
      return
    }
    void loadLiveStatus()
    const intervalId = window.setInterval(() => {
      void loadLiveStatus()
    }, 2000)
    return () => window.clearInterval(intervalId)
  }, [decodedEventId, hasActiveRobotRun, loadLiveStatus])

  useEffect(() => {
    if (!Number.isFinite(focusedWeightmentId)) {
      lastAutoScrolledWeightmentIdRef.current = null
      return
    }
    if (lastAutoScrolledWeightmentIdRef.current === focusedWeightmentId) {
      return
    }
    const target = document.getElementById(`weightment-row-${focusedWeightmentId}`)
    if (!target) {
      return
    }
    const timer = window.setTimeout(() => {
      lastAutoScrolledWeightmentIdRef.current = focusedWeightmentId
      target.scrollIntoView({ behavior: 'smooth', block: 'center' })
    }, 100)
    return () => window.clearTimeout(timer)
  }, [focusedWeightmentId, rows])

  useEffect(() => {
    setOpenRobotDetailIds((current) => {
      const validIds = new Set(robotDetailRows.map((row) => row.weightment_id))
      if (!robotDetailsInitialized) {
        return []
      }
      const kept = current.filter((id) => validIds.has(id))
      if (kept.length !== current.length) {
        return kept
      }
      return current
    })
    if (!robotDetailsInitialized && robotDetailRows.length > 0) {
      setRobotDetailsInitialized(true)
    }
  }, [robotDetailRows, robotDetailsInitialized])

  const runRobot = async (weightmentId: number) => {
    try {
      setRunningId(weightmentId)
      const res = await fetch(`${apiBase}/webhook_weightments/${weightmentId}/run_robot`, {
        method: 'POST',
      })
      if (!res.ok) {
        const text = await res.text()
        throw new Error(text || `Failed to start robot run for weightment ${weightmentId}`)
      }
      await loadRows()
    } catch (error) {
      const message =
        error instanceof Error
          ? error.message
          : `Failed to start robot run for weightment ${weightmentId}`
      window.alert(message)
    } finally {
      setRunningId(null)
    }
  }

  const runFullBatch = async () => {
    try {
      setRunningBatch(true)
      const res = await fetch(`${apiBase}/webhook_events/${encodeURIComponent(decodedEventId)}/run_all`, {
        method: 'POST',
      })
      if (!res.ok) {
        const text = await res.text()
        throw new Error(text || `Failed to start full batch for event ${decodedEventId}`)
      }
      await loadRows()
    } catch (error) {
      const message =
        error instanceof Error
          ? error.message
          : `Failed to start full batch for event ${decodedEventId}`
      window.alert(message)
    } finally {
      setRunningBatch(false)
    }
  }

  const sendWeightment = async (weightmentId: number) => {
    try {
      setSendingId(weightmentId)
      const res = await fetch(`${apiBase}/webhook_weightments/${weightmentId}/send`, {
        method: 'POST',
      })
      if (!res.ok) {
        const text = await res.text()
        throw new Error(text || `Failed to send weightment ${weightmentId}`)
      }
      await loadRows()
    } catch (error) {
      const message =
        error instanceof Error ? error.message : `Failed to send weightment ${weightmentId}`
      window.alert(message)
    } finally {
      setSendingId(null)
    }
  }

  const robotStatusChip = (row: Row) => {
    if (!row.robot_status) {
      return <span className="text-xs text-white/50">—</span>
    }
    const label =
      row.robot_status === 'starting'
        ? 'Starting'
        : row.robot_status === 'running'
          ? 'Running'
          : row.robot_status === 'awaiting_processing'
            ? 'Awaiting Processing'
          : row.robot_status === 'succeeded'
            ? 'Succeeded'
            : row.robot_status === 'mes_send_failed'
              ? 'MES Send Failed'
              : row.robot_status === 'failed'
                ? 'Failed'
                : row.robot_status
    const classes =
      row.robot_status === 'succeeded'
        ? 'bg-emerald-500/20 text-emerald-300'
        : row.robot_status === 'starting' ||
            row.robot_status === 'running'
          ? 'bg-sky-500/20 text-sky-300'
          : row.robot_status === 'awaiting_processing'
            ? 'bg-amber-500/20 text-amber-300'
          : 'bg-rose-500/20 text-rose-300'
    return (
      <span className={`inline-flex rounded-full px-2 py-1 text-xs font-semibold ${classes}`}>
        {label}
      </span>
    )
  }

  const toggleRobotDetail = (weightmentId: number) => {
    setOpenRobotDetailIds((current) =>
      current.includes(weightmentId)
        ? current.filter((id) => id !== weightmentId)
        : [...current, weightmentId]
    )
  }

  return (
    <SidebarLayout>
      <div className="px-6 py-6">
        <div className="mb-4 flex items-end justify-between gap-4">
          <div>
            <div className="mb-2">
              <Link to="/webhook-weightments" className="text-sm text-sky-300 hover:text-sky-200">
                ← Back to webhook batches
              </Link>
            </div>
            <h1 className="text-2xl font-bold" style={{ fontFamily: 'Space Grotesk' }}>
              Batch Weightments
            </h1>
            <p className="text-white/70">
              Batch `{summary?.batch_id ?? '—'}`
            </p>
          </div>
          <div className="flex items-center gap-3">
            <span
              className={`inline-flex rounded-full px-3 py-1 text-xs font-semibold ${
                summary?.completed
                  ? 'bg-emerald-500/20 text-emerald-300'
                  : 'bg-rose-500/20 text-rose-300'
              }`}
            >
              {summary?.completed ? 'Complete' : 'Not Complete'}
            </span>
            {!summary?.completed && (
              <div className="flex flex-col items-end gap-1">
                <Button
                  onClick={() => runFullBatch()}
                  disabled={
                    loading ||
                    runningBatch ||
                    summary?.batch_run_in_progress ||
                    summary?.batch_auto_run_enabled ||
                    incompleteRowsMissingLocation.length > 0
                  }
                >
                  {runningBatch
                    ? 'Starting Batch...'
                    : summary?.batch_auto_run_enabled
                      ? 'Batch Auto-Run Enabled'
                      : 'Run Full Batch'}
                </Button>
                {incompleteRowsMissingLocation.length > 0 && (
                  <span className="text-xs text-rose-300">
                    Full batch requires locations for every incomplete weightment
                  </span>
                )}
              </div>
            )}
            <Button onClick={() => loadRows()} disabled={loading}>
              {loading ? 'Refreshing…' : 'Refresh'}
            </Button>
          </div>
        </div>

        <GlassCard className="mb-6">
          <div className="grid grid-cols-1 gap-3 text-sm text-white/80 md:grid-cols-3">
            <div>
              <div className="text-white/50">Sent UTC</div>
              <DateTimeText value={summary?.sent_utc ?? null} />
            </div>
            <div>
              <div className="text-white/50">Batch ID</div>
              <div>{summary?.batch_id ?? '—'}</div>
            </div>
            <div>
              <div className="text-white/50">Batch Auto-Run</div>
              <div>
                {summary?.batch_auto_run_enabled
                  ? summary?.batch_run_in_progress
                    ? `Enabled, running weightment ${summary?.next_weightment_id ?? '—'}`
                    : `Enabled, next weightment ${summary?.next_weightment_id ?? '—'}`
                  : 'Disabled'}
              </div>
            </div>
          </div>
        </GlassCard>

        <GlassCard>
          <div className="mb-3 text-xs text-white/60">Showing {rows.length} weightments</div>
          <div className="overflow-auto">
            <table className="w-full text-sm">
              <thead className="text-left text-white/70">
                <tr>
                  <th>Weightment ID</th>
                  <th className="text-right">Target Weight (kg)</th>
                  <th className="text-right">Actual Weight (kg)</th>
                  <th>Completed</th>
                  <th>Stock Item ID</th>
                  <th>Ingredient Name</th>
                  <th>Location ID</th>
                  <th>Start Time</th>
                  <th>End Time</th>
                  <th className="text-right">Energy (kWh)</th>
                  <th />
                </tr>
              </thead>
              <tbody>
                {loading ? (
                  <tr>
                    <td className="py-6 text-center text-white/70" colSpan={11}>
                      Loading…
                    </td>
                  </tr>
                ) : rows.length === 0 ? (
                  <tr>
                    <td className="py-6 text-center text-white/70" colSpan={11}>
                      No weightments found for this event.
                    </td>
                  </tr>
                ) : (
                  rows.map((row) => (
                    <tr
                      id={`weightment-row-${row.weightment_id}`}
                      key={row.weightment_id}
                      className={`border-t border-slate-800 ${
                        Number.isFinite(focusedWeightmentId) &&
                        row.weightment_id === focusedWeightmentId
                          ? 'bg-sky-500/10 ring-1 ring-sky-400/40'
                          : ''
                      }`}
                    >
                      <td className="py-3 pr-4">{row.weightment_id}</td>
                      <td className="py-3 pr-4 text-right">
                        {row.target_weight_kg != null ? row.target_weight_kg.toFixed(3) : '—'}
                      </td>
                      <td className="py-3 pr-4 text-right">
                        {row.actual_weight_kg != null ? row.actual_weight_kg.toFixed(3) : '—'}
                      </td>
                      <td className="py-3 pr-4">
                        <span
                          className={`inline-flex rounded-full px-2 py-1 text-xs font-semibold ${
                            row.completed
                              ? 'bg-emerald-500/20 text-emerald-300'
                              : 'bg-rose-500/20 text-rose-300'
                          }`}
                        >
                          {row.completed ? 'Complete' : 'Not Complete'}
                        </span>
                      </td>
                      <td className="py-3 pr-4">{row.stock_item_id ?? '—'}</td>
                      <td className="py-3 pr-4">{row.ingredient_name ?? '—'}</td>
                      <td className="py-3 pr-4">{row.location_id ?? '—'}</td>
                      <td className="py-3 pr-4">
                        <DateTimeText value={row.start_time} />
                      </td>
                      <td className="py-3 pr-4">
                        <DateTimeText value={row.end_time} />
                      </td>
                      <td className="py-3 pr-4 text-right">
                        {row.energy_kwh != null ? row.energy_kwh.toFixed(3) : '—'}
                      </td>
                      <td className="py-3 text-right">
                        {!row.completed && (
                          <div className="flex flex-col items-end gap-2">
                            <Button
                              size="sm"
                              onClick={() => runRobot(row.weightment_id)}
                              disabled={
                                runningId === row.weightment_id ||
                                row.location_id == null ||
                                row.robot_status === 'starting' ||
                                row.robot_status === 'running' ||
                                row.robot_status === 'awaiting_processing'
                              }
                            >
                              {runningId === row.weightment_id
                                ? 'Starting…'
                                : row.robot_status === 'starting'
                                  ? 'Starting…'
                                  : row.robot_status === 'running'
                                    ? 'Running…'
                                    : row.robot_status === 'awaiting_processing'
                                      ? 'Awaiting Trace…'
                                    : 'Run Robot'}
                            </Button>
                            <Button
                              size="sm"
                              variant="ghost"
                              onClick={() => sendWeightment(row.weightment_id)}
                              disabled={
                                sendingId === row.weightment_id ||
                                row.location_id == null ||
                                row.robot_status === 'starting' ||
                                row.robot_status === 'running'
                              }
                            >
                              {sendingId === row.weightment_id
                                ? 'Sending…'
                                : 'Send To MES'}
                            </Button>
                            {row.location_id == null && (
                              <span className="text-xs text-rose-300">
                                No matching location id found
                              </span>
                            )}
                          </div>
                        )}
                      </td>
                    </tr>
                  ))
                )}
              </tbody>
            </table>
          </div>
        </GlassCard>

        {robotDetailRows.length > 0 && (
          <div className="mt-6 space-y-4">
            <div className="text-sm text-white/70">Robot Run Details</div>
            {robotDetailRows.map((row) => (
              <GlassCard key={`robot-details-${row.weightment_id}`}>
                <div className="flex flex-col gap-4">
                  <div
                    id={`robot-detail-${row.weightment_id}`}
                    className="flex flex-wrap items-center justify-between gap-3"
                  >
                    <div>
                      <div className="text-sm font-semibold text-white">
                        Weightment {row.weightment_id}
                      </div>
                      <div className="text-xs text-white/50">
                        Ingredient {row.ingredient_name ?? row.stock_item_id ?? '—'}
                      </div>
                    </div>
                    <div className="flex items-center gap-2">
                      {robotStatusChip(row)}
                      <Button
                        size="sm"
                        variant="ghost"
                        onClick={() => toggleRobotDetail(row.weightment_id)}
                        aria-label={
                          openRobotDetailIds.includes(row.weightment_id)
                            ? `Collapse robot details for weightment ${row.weightment_id}`
                            : `Expand robot details for weightment ${row.weightment_id}`
                        }
                      >
                        <ChevronIcon open={openRobotDetailIds.includes(row.weightment_id)} />
                      </Button>
                    </div>
                  </div>

                  <div
                    className={`overflow-hidden transition-all duration-300 ease-in-out ${
                      openRobotDetailIds.includes(row.weightment_id)
                        ? 'max-h-[1000px] opacity-100'
                        : 'max-h-0 opacity-0'
                    }`}
                  >
                    <div className="flex flex-col gap-4 pt-1">
                      <div className="grid grid-cols-1 gap-3 text-sm text-white/80 md:grid-cols-3">
                        <div>
                          <div className="text-white/50">Requested</div>
                          <DateTimeText value={row.robot_requested_at} />
                        </div>
                        <div>
                          <div className="text-white/50">Started</div>
                          <DateTimeText value={row.robot_started_at} />
                        </div>
                        <div>
                          <div className="text-white/50">Finished</div>
                          <DateTimeText value={row.robot_finished_at} />
                        </div>
                        <div>
                          <div className="text-white/50">Trace Run ID</div>
                          <div>{row.robot_trace_run_id ?? '—'}</div>
                        </div>
                        <div>
                          <div className="text-white/50">Processed Log ID</div>
                          <div>{row.robot_processed_id ?? '—'}</div>
                        </div>
                        <div>
                          <div className="text-white/50">Weights</div>
                          <div>
                            Live {formatNumber(row.robot_live_completion_weight_kg)} kg,
                            processed {formatNumber(row.robot_processed_final_weight_kg)} kg
                          </div>
                        </div>
                        <div>
                          <div className="text-white/50">Processed Start</div>
                          <DateTimeText value={row.robot_processed_start_time} />
                        </div>
                        <div>
                          <div className="text-white/50">Processed End</div>
                          <DateTimeText value={row.robot_processed_end_time} />
                        </div>
                        <div>
                          <div className="text-white/50">MCAP / Parquet</div>
                          <div className="break-all text-xs">
                            {row.robot_mcap_path ?? '—'}
                            <br />
                            {row.robot_parquet_path ?? '—'}
                          </div>
                        </div>
                      </div>

                      {(row.robot_processed_pour_duration_s != null ||
                        row.robot_processed_scoop_duration_s != null ||
                        row.robot_processed_settle_time_s != null ||
                        row.robot_processed_overshoot_g != null) && (
                        <div className="text-sm text-white/70">
                          Processed metrics: scoop {formatNumber(row.robot_processed_scoop_duration_s, 2)}s,
                          pour {formatNumber(row.robot_processed_pour_duration_s, 2)}s, settle{' '}
                          {formatNumber(row.robot_processed_settle_time_s, 2)}s, overshoot{' '}
                          {formatNumber(row.robot_processed_overshoot_g, 1)} g
                        </div>
                      )}

                      {row.robot_status === 'awaiting_processing' && (
                        <div className="text-sm text-amber-300">
                          Waiting for processed MCAP before MES send
                        </div>
                      )}

                      {row.robot_error && (
                        <div className="text-sm text-rose-300">{row.robot_error}</div>
                      )}
                    </div>
                  </div>
                </div>
              </GlassCard>
            ))}
          </div>
        )}

      </div>
    </SidebarLayout>
  )
}
