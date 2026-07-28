import {
  Button,
  ColorPicker,
  Input,
  InputNumber,
  Space,
  Tag,
  Typography,
} from 'antd'
import type { Color } from 'antd/es/color-picker'
import type { ThresholdStep } from '../../lib/api'
import { DEFAULT_THRESHOLDS } from '../../lib/echartsTheme'

const { Text } = Typography

export type ThresholdPreset = {
  id: string
  label: string
  unit: string
  min: number
  max: number
  thresholds: ThresholdStep[]
}

export const THRESHOLD_PRESETS: ThresholdPreset[] = [
  {
    id: 'cpu',
    label: 'CPU health',
    unit: 'percent',
    min: 0,
    max: 100,
    thresholds: [
      { upTo: 70, color: '#34d399', label: 'OK' },
      { upTo: 90, color: '#fbbf24', label: 'High' },
      { upTo: 100, color: '#fb7185', label: 'Critical' },
    ],
  },
  {
    id: 'memory',
    label: 'Memory',
    unit: 'percent',
    min: 0,
    max: 100,
    thresholds: [
      { upTo: 75, color: '#34d399', label: 'OK' },
      { upTo: 90, color: '#fbbf24', label: 'High' },
      { upTo: 100, color: '#fb7185', label: 'Critical' },
    ],
  },
  {
    id: 'disk',
    label: 'Disk',
    unit: 'percent',
    min: 0,
    max: 100,
    thresholds: [
      { upTo: 80, color: '#34d399', label: 'OK' },
      { upTo: 90, color: '#fbbf24', label: 'High' },
      { upTo: 100, color: '#fb7185', label: 'Critical' },
    ],
  },
]

function BandPreview({
  thresholds,
  min,
  max,
}: {
  thresholds: ThresholdStep[]
  min: number
  max: number
}) {
  const span = Math.max(max - min, 1e-9)
  const sorted = [...thresholds].sort((a, b) => a.upTo - b.upTo)
  let prev = min
  const segments = sorted.map((s) => {
    const from = prev
    const to = Math.max(from, Math.min(max, s.upTo))
    prev = to
    return { ...s, from, to, width: ((to - from) / span) * 100 }
  })
  return (
    <div
      style={{
        display: 'flex',
        height: 14,
        borderRadius: 999,
        overflow: 'hidden',
        border: '1px solid #1e293b',
        background: '#0d121a',
      }}
    >
      {segments.map((s, i) => (
        <div
          key={i}
          title={`${s.label || ''} ≤ ${s.upTo}`}
          style={{
            width: `${Math.max(s.width, 0)}%`,
            background: s.color,
            minWidth: s.width > 0 ? 4 : 0,
          }}
        />
      ))}
    </div>
  )
}

function toHex(c: Color | string): string {
  if (typeof c === 'string') return c
  return c.toHexString()
}

export default function ThresholdEditor({
  value,
  min = 0,
  max = 100,
  onChange,
  onPreset,
  showPresets = true,
}: {
  value?: ThresholdStep[]
  min?: number
  max?: number
  onChange: (next: ThresholdStep[]) => void
  onPreset?: (preset: ThresholdPreset) => void
  showPresets?: boolean
}) {
  const steps =
    value && value.length
      ? value
      : DEFAULT_THRESHOLDS.map((t) => ({
          ...t,
          upTo: t.upTo === 100 ? max : t.upTo,
        }))

  const update = (i: number, patch: Partial<ThresholdStep>) => {
    const next = steps.map((s, j) => (j === i ? { ...s, ...patch } : s))
    onChange(next)
  }

  const move = (i: number, dir: -1 | 1) => {
    const j = i + dir
    if (j < 0 || j >= steps.length) return
    const next = [...steps]
    ;[next[i], next[j]] = [next[j], next[i]]
    onChange(next)
  }

  const invalid = steps.some(
    (s, i) => i > 0 && s.upTo <= steps[i - 1].upTo,
  )

  return (
    <div>
      <Text type="secondary" style={{ display: 'block', marginBottom: 8 }}>
        Stages color the scale from min → max. Each row ends at “Up to”.
      </Text>

      {showPresets ? (
        <Space wrap style={{ marginBottom: 12 }}>
          {THRESHOLD_PRESETS.map((p) => (
            <Tag
              key={p.id}
              style={{ cursor: 'pointer', userSelect: 'none' }}
              color="processing"
              onClick={() => {
                onChange(p.thresholds.map((t) => ({ ...t })))
                onPreset?.(p)
              }}
            >
              {p.label}
            </Tag>
          ))}
        </Space>
      ) : null}

      <BandPreview thresholds={steps} min={min} max={max} />
      <div
        style={{
          display: 'flex',
          justifyContent: 'space-between',
          marginTop: 4,
          marginBottom: 12,
          fontFamily: 'ui-monospace, monospace',
          fontSize: 10,
          color: '#64748b',
        }}
      >
        <span>{min}</span>
        <span>{max}</span>
      </div>

      <Space direction="vertical" style={{ width: '100%' }} size={8}>
        {steps.map((s, i) => (
          <Space.Compact key={i} style={{ width: '100%' }}>
            <ColorPicker
              value={s.color}
              size="small"
              onChange={(c) => update(i, { color: toHex(c) })}
            />
            <InputNumber
              style={{ width: 100 }}
              value={s.upTo}
              min={min}
              max={max}
              onChange={(v) => update(i, { upTo: Number(v) || 0 })}
              addonBefore="≤"
            />
            <Input
              style={{ flex: 1, minWidth: 80 }}
              placeholder="Label"
              value={s.label || ''}
              onChange={(e) => update(i, { label: e.target.value })}
            />
            <Button size="small" onClick={() => move(i, -1)} disabled={i === 0}>
              ↑
            </Button>
            <Button
              size="small"
              onClick={() => move(i, 1)}
              disabled={i === steps.length - 1}
            >
              ↓
            </Button>
            <Button
              danger
              size="small"
              disabled={steps.length <= 1}
              onClick={() => onChange(steps.filter((_, j) => j !== i))}
            >
              ×
            </Button>
          </Space.Compact>
        ))}
      </Space>

      <Button
        type="dashed"
        block
        style={{ marginTop: 10 }}
        onClick={() =>
          onChange([
            ...steps,
            {
              upTo: max,
              color: '#a78bfa',
              label: 'Stage',
            },
          ])
        }
      >
        + Add stage
      </Button>

      {invalid ? (
        <Text type="danger" style={{ display: 'block', marginTop: 8 }}>
          Each “Up to” must be greater than the previous stage.
        </Text>
      ) : null}
    </div>
  )
}
