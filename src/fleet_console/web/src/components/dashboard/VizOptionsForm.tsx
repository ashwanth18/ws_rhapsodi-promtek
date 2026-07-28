import { Form, InputNumber, Slider, Switch, Typography } from 'antd'
import type { PanelOptions, PanelType } from '../../lib/api'
import ThresholdEditor, { type ThresholdPreset } from './ThresholdEditor'

const { Text } = Typography

export default function VizOptionsForm({
  type,
  options,
  min,
  max,
  onChange,
  onPreset,
}: {
  type: PanelType
  options: PanelOptions
  min?: number | null
  max?: number | null
  onChange: (next: PanelOptions) => void
  onPreset?: (preset: ThresholdPreset) => void
}) {
  const patch = (p: Partial<PanelOptions>) => onChange({ ...options, ...p })

  if (type === 'logs' || type === 'table') {
    return (
      <Text type="secondary">
        No extra style options for {type} panels.
      </Text>
    )
  }

  return (
    <Form layout="vertical" size="middle">
      {(type === 'timeseries' || type === 'area' || type === 'bar') && (
        <>
          <Form.Item label="Show legend">
            <Switch
              checked={options.showLegend !== false}
              onChange={(v) => patch({ showLegend: v })}
            />
          </Form.Item>
          {type !== 'bar' ? (
            <Form.Item label="Smooth lines">
              <Switch
                checked={options.smooth !== false}
                onChange={(v) => patch({ smooth: v })}
              />
            </Form.Item>
          ) : null}
          <Form.Item label="Stack series">
            <Switch
              checked={Boolean(options.stack)}
              onChange={(v) => patch({ stack: v })}
            />
          </Form.Item>
          {type === 'area' ? (
            <Form.Item label={`Fill opacity (${Math.round((options.fillOpacity ?? 0.25) * 100)}%)`}>
              <Slider
                min={0}
                max={1}
                step={0.05}
                value={options.fillOpacity ?? 0.25}
                onChange={(v) => patch({ fillOpacity: v })}
              />
            </Form.Item>
          ) : null}
        </>
      )}

      {type === 'pie' ? (
        <>
          <Form.Item label="Donut">
            <Switch
              checked={options.donut !== false}
              onChange={(v) => patch({ donut: v })}
            />
          </Form.Item>
          <Form.Item label="Show legend">
            <Switch
              checked={options.showLegend !== false}
              onChange={(v) => patch({ showLegend: v })}
            />
          </Form.Item>
        </>
      ) : null}

      {type === 'gauge' ? (
        <>
          <Form.Item label="Show pointer">
            <Switch
              checked={options.showPointer !== false}
              onChange={(v) => patch({ showPointer: v })}
            />
          </Form.Item>
          <Form.Item label="Stages">
            <ThresholdEditor
              value={options.thresholds}
              min={min ?? 0}
              max={max ?? 100}
              onChange={(thresholds) => patch({ thresholds })}
              onPreset={onPreset}
            />
          </Form.Item>
        </>
      ) : null}

      {type === 'stat' ? (
        <>
          <Form.Item label="Decimals">
            <InputNumber
              min={0}
              max={6}
              value={options.decimals ?? 1}
              onChange={(v) => patch({ decimals: v ?? 1 })}
            />
          </Form.Item>
          <Form.Item label="Threshold colors">
            <ThresholdEditor
              value={options.thresholds}
              min={min ?? 0}
              max={max ?? 100}
              onChange={(thresholds) => patch({ thresholds })}
              onPreset={onPreset}
            />
          </Form.Item>
        </>
      ) : null}
    </Form>
  )
}
