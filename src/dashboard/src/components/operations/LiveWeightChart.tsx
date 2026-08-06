import { useEffect, useMemo, useState } from 'react'
import {
  Area,
  AreaChart,
  CartesianGrid,
  ReferenceLine,
  ResponsiveContainer,
  Tooltip,
  XAxis,
  YAxis,
} from 'recharts'
import GlassCard from '../GlassCard'

type Point = { t: number; weight: number }

type Props = {
  weight: number | null
  targetWeightG: number | null
  targetToleranceG: number | null
  windowMs?: number
  /** Absolute-scale reference line (e.g. postScoop + target during lightsout pour). */
  referenceWeightG?: number | null
  referenceToleranceG?: number | null
}

export default function LiveWeightChart({
  weight,
  targetWeightG,
  targetToleranceG,
  windowMs = 60000,
  referenceWeightG,
  referenceToleranceG,
}: Props) {
  const refY =
    typeof referenceWeightG === 'number' ? referenceWeightG : targetWeightG
  const refTol =
    typeof referenceToleranceG === 'number'
      ? referenceToleranceG
      : targetToleranceG
  const [history, setHistory] = useState<Point[]>([])

  useEffect(() => {
    if (typeof weight !== 'number' || !Number.isFinite(weight)) return
    const now = Date.now()
    setHistory((prev) => {
      const next = [...prev, { t: now, weight }]
      return next.filter((p) => now - p.t <= windowMs)
    })
  }, [weight, windowMs])

  const data = useMemo(
    () =>
      history.map((p) => ({
        time: new Date(p.t).toLocaleTimeString([], {
          hour: '2-digit',
          minute: '2-digit',
          second: '2-digit',
          hour12: false,
        }),
        weight: p.weight,
      })),
    [history]
  )

  return (
    <GlassCard className="h-full">
      <div className="mb-3 flex items-center justify-between">
        <h3 className="font-display text-sm font-semibold uppercase tracking-wider text-[var(--text-faint)]">
          Weight Curve
        </h3>
        <span className="text-xs text-[var(--text-muted)]">Last 60s</span>
      </div>
      <div className="h-56 w-full">
        {data.length < 2 ? (
          <div className="flex h-full items-center justify-center text-sm text-[var(--text-muted)]">
            Waiting for weight samples…
          </div>
        ) : (
          <ResponsiveContainer width="100%" height="100%">
            <AreaChart data={data} margin={{ top: 8, right: 8, left: 0, bottom: 0 }}>
              <defs>
                <linearGradient id="weightFill" x1="0" y1="0" x2="0" y2="1">
                  <stop offset="0%" stopColor="var(--accent)" stopOpacity={0.35} />
                  <stop offset="100%" stopColor="var(--accent)" stopOpacity={0.02} />
                </linearGradient>
              </defs>
              <CartesianGrid stroke="var(--chart-grid)" strokeDasharray="3 3" />
              <XAxis dataKey="time" tick={{ fill: 'var(--chart-axis)', fontSize: 10 }} />
              <YAxis tick={{ fill: 'var(--chart-axis)', fontSize: 10 }} width={42} />
              <Tooltip
                contentStyle={{
                  background: 'var(--surface-2)',
                  border: '1px solid var(--border)',
                  borderRadius: 8,
                  color: 'var(--text-primary)',
                }}
              />
              {typeof refY === 'number' && (
                <ReferenceLine
                  y={refY}
                  stroke="var(--status-good-fg)"
                  strokeDasharray="4 4"
                />
              )}
              {typeof refY === 'number' && typeof refTol === 'number' && (
                  <>
                    <ReferenceLine
                      y={refY + refTol}
                      stroke="var(--status-warn-fg)"
                      strokeDasharray="2 6"
                    />
                    <ReferenceLine
                      y={refY - refTol}
                      stroke="var(--status-warn-fg)"
                      strokeDasharray="2 6"
                    />
                  </>
                )}
              <Area
                type="monotone"
                dataKey="weight"
                stroke="var(--accent)"
                fill="url(#weightFill)"
                strokeWidth={2}
                dot={false}
                isAnimationActive={false}
              />
            </AreaChart>
          </ResponsiveContainer>
        )}
      </div>
    </GlassCard>
  )
}
