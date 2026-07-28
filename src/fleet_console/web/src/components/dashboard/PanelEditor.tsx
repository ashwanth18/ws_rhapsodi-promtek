import { useEffect, useMemo, useState } from 'react'
import {
  Alert,
  AutoComplete,
  Button,
  Checkbox,
  Col,
  Collapse,
  ConfigProvider,
  Form,
  Input,
  InputNumber,
  Modal,
  Row,
  Select,
  Space,
  Tabs,
  theme,
  Typography,
} from 'antd'
import {
  Activity,
  AreaChart,
  BarChart3,
  Gauge,
  Hash,
  Logs,
  PieChart,
  Table2,
} from 'lucide-react'
import {
  api,
  type CatalogMetric,
  type DashboardPanel,
  type PanelBuilderState,
  type PanelDatasource,
  type PanelOptions,
  type PanelType,
  type PromBuilderFn,
} from '../../lib/api'
import { defaultOptionsForType } from '../../lib/echartsTheme'
import { gaugeMax, UNIT_OPTIONS } from '../../lib/formatUnits'
import { newId } from '../../lib/id'
import {
  compileQuery,
  defaultBuilder,
  PROM_FNS,
  RANGE_OPTIONS,
} from '../../lib/queryBuilder'
import Panel from './Panel'
import ThresholdEditor, { type ThresholdPreset } from './ThresholdEditor'
import VizOptionsForm from './VizOptionsForm'

const { TextArea } = Input
const { Text, Title } = Typography

const TYPES: {
  value: PanelType
  label: string
  hint: string
  icon: typeof Activity
}[] = [
  { value: 'timeseries', label: 'Time series', hint: 'Lines over time', icon: Activity },
  { value: 'area', label: 'Area', hint: 'Filled series', icon: AreaChart },
  { value: 'bar', label: 'Bar', hint: 'Categorical bars', icon: BarChart3 },
  { value: 'stat', label: 'Stat', hint: 'Big number', icon: Hash },
  { value: 'gauge', label: 'Gauge', hint: 'Stages & needle', icon: Gauge },
  { value: 'pie', label: 'Pie', hint: 'Share breakdown', icon: PieChart },
  { value: 'table', label: 'Table', hint: 'Raw vectors', icon: Table2 },
  { value: 'logs', label: 'Logs', hint: 'Log stream', icon: Logs },
]
const DS: PanelDatasource[] = ['prometheus', 'loki']
const PANEL_TYPES = new Set<string>(TYPES.map((t) => t.value))
const CATEGORY_ORDER = ['host', 'containers', 'logs', 'other']
const CATEGORY_LABELS: Record<string, string> = {
  host: 'Host',
  containers: 'Containers',
  logs: 'Logs',
  other: 'Other',
}

function defaultType(ds: PanelDatasource): PanelType {
  return ds === 'loki' ? 'logs' : 'timeseries'
}

function asPanelType(viz: string | undefined, ds: PanelDatasource): PanelType {
  if (viz && PANEL_TYPES.has(viz)) return viz as PanelType
  return defaultType(ds)
}

function suggestTitle(ds: PanelDatasource, builder: PanelBuilderState): string {
  if (ds === 'loki') {
    const c = builder.container?.trim()
    return c ? `Logs · ${c}` : 'Logs'
  }
  const m = builder.metric?.trim()
  if (!m) return 'New panel'
  if (builder.fn === 'cpu_pct_from_idle') return 'CPU %'
  if (builder.fn === 'mem_pct_used') return 'Memory %'
  if (builder.fn === 'disk_root_pct') return 'Disk %'
  if (builder.fn === 'temp_max') return 'Temperature'
  return m
}

function defaultGaugeMax(unit: string): number {
  return gaugeMax(unit === 'none' ? undefined : unit)
}

export default function PanelEditor({
  initial,
  deviceId,
  onSave,
  onCancel,
}: {
  initial?: Partial<DashboardPanel>
  deviceId?: string
  onSave: (panel: DashboardPanel) => void
  onCancel: () => void
}) {
  const initialDs = initial?.datasource || 'prometheus'
  const initialType = initial?.type || defaultType(initialDs)
  const [tab, setTab] = useState('data')
  const [title, setTitle] = useState(initial?.title || 'New panel')
  const [titleTouched, setTitleTouched] = useState(Boolean(initial?.title))
  const [type, setType] = useState<PanelType>(initialType)
  const [datasource, setDatasource] = useState<PanelDatasource>(initialDs)
  const [unit, setUnit] = useState<string>(initial?.unit || 'none')
  const [min, setMin] = useState<number | null>(
    initial?.min ?? (initialType === 'gauge' || initialType === 'stat' ? 0 : null),
  )
  const [max, setMax] = useState<number | null>(
    initial?.max ??
      (initialType === 'gauge' || initialType === 'stat'
        ? defaultGaugeMax(initial?.unit || 'percent')
        : null),
  )
  const [options, setOptions] = useState<PanelOptions>(
    () => initial?.options || defaultOptionsForType(initialType),
  )
  const [builder, setBuilder] = useState<PanelBuilderState>(
    () => initial?.builder || defaultBuilder(initialDs),
  )
  const [advanced, setAdvanced] = useState(
    () =>
      initial?.builder?.mode === 'advanced' ||
      Boolean(initial?.query && !initial?.builder),
  )
  const [query, setQuery] = useState(
    () =>
      initial?.query ||
      compileQuery(initialDs, initial?.builder || defaultBuilder(initialDs)),
  )
  const [catalogId, setCatalogId] = useState<string | undefined>(
    initial?.catalog_id,
  )
  const [catalog, setCatalog] = useState<CatalogMetric[]>([])
  const [catalogFilter, setCatalogFilter] = useState('')
  const [catalogError, setCatalogError] = useState<string | null>(null)
  const [customOpen, setCustomOpen] = useState(
    () => Boolean(initial?.query && !initial?.catalog_id),
  )

  const [metrics, setMetrics] = useState<string[]>([])
  const [labelNames, setLabelNames] = useState<string[]>([])
  const [labelValueCache, setLabelValueCache] = useState<
    Record<string, string[]>
  >({})
  const [hosts, setHosts] = useState<string[]>([])
  const [containers, setContainers] = useState<string[]>([])
  const [metaError, setMetaError] = useState<string | null>(null)

  useEffect(() => {
    let cancelled = false
    api
      .listMetricCatalog()
      .then((r) => {
        if (!cancelled) setCatalog(r.metrics || [])
      })
      .catch((e) => {
        if (!cancelled)
          setCatalogError(e instanceof Error ? e.message : String(e))
      })
    return () => {
      cancelled = true
    }
  }, [])

  useEffect(() => {
    if (!customOpen || datasource !== 'prometheus') return
    let cancelled = false
    setMetaError(null)
    api
      .metaPromMetrics()
      .then((r) => {
        if (!cancelled) setMetrics(r.metrics || [])
      })
      .catch((e) => {
        if (!cancelled)
          setMetaError(e instanceof Error ? e.message : String(e))
      })
    return () => {
      cancelled = true
    }
  }, [datasource, customOpen])

  useEffect(() => {
    if (!customOpen || datasource !== 'prometheus' || !builder.metric) return
    let cancelled = false
    api
      .metaPromLabels(builder.metric)
      .then((r) => {
        if (!cancelled) setLabelNames(r.labels || [])
      })
      .catch(() => {
        if (!cancelled) setLabelNames([])
      })
    return () => {
      cancelled = true
    }
  }, [datasource, builder.metric, customOpen])

  useEffect(() => {
    if (!customOpen || datasource !== 'loki') return
    let cancelled = false
    setMetaError(null)
    Promise.all([
      api.metaLokiLabelValues('host'),
      api.metaLokiLabelValues('compose_service'),
    ])
      .then(([h, c]) => {
        if (cancelled) return
        setHosts(h.values || [])
        setContainers(c.values || [])
      })
      .catch((e) => {
        if (!cancelled)
          setMetaError(e instanceof Error ? e.message : String(e))
      })
    return () => {
      cancelled = true
    }
  }, [datasource, customOpen])

  useEffect(() => {
    if (!customOpen || datasource !== 'loki') return
    const host = (builder.host || '').trim()
    const resolvedHost =
      !host || host === '$device_id' ? deviceId || '' : host
    if (!resolvedHost) return
    let cancelled = false
    const load = async () => {
      try {
        if (!host || host === '$device_id') {
          const r = await api.deviceLogContainers(resolvedHost)
          if (!cancelled && r.containers?.length) {
            setContainers(r.containers)
            return
          }
        }
        const match = `{host="${resolvedHost.replace(/"/g, '')}"}`
        const r = await api.metaLokiLabelValues('compose_service', match)
        if (!cancelled && r.values?.length) setContainers(r.values)
      } catch {
        /* keep previous list */
      }
    }
    void load()
    return () => {
      cancelled = true
    }
  }, [datasource, builder.host, deviceId, customOpen])

  // Custom guided builder owns the query only when not using a catalog entry
  useEffect(() => {
    if (catalogId || !customOpen) return
    if (advanced) return
    const next = compileQuery(datasource, builder)
    setQuery(next)
    if (!titleTouched) {
      setTitle(suggestTitle(datasource, builder))
    }
  }, [advanced, datasource, builder, titleTouched, catalogId, customOpen])

  const loadLabelValues = async (label: string) => {
    if (!label || labelValueCache[label]) return
    try {
      const r = await api.metaPromLabelValues({
        label,
        metric: builder.metric,
      })
      setLabelValueCache((prev) => ({ ...prev, [label]: r.values || [] }))
    } catch {
      setLabelValueCache((prev) => ({ ...prev, [label]: [] }))
    }
  }

  const preview: DashboardPanel = useMemo(
    () => ({
      id: initial?.id || 'preview',
      title,
      type,
      datasource,
      query,
      unit: unit && unit !== 'none' ? unit : undefined,
      min:
        (type === 'gauge' || type === 'stat') && min != null ? min : undefined,
      max:
        (type === 'gauge' || type === 'stat') && max != null ? max : undefined,
      options,
      catalog_id: catalogId,
      builder: catalogId
        ? undefined
        : { ...builder, mode: advanced ? 'advanced' : 'guided' },
      x: initial?.x ?? 0,
      y: initial?.y ?? 0,
      w: initial?.w ?? 6,
      h: initial?.h ?? 4,
    }),
    [
      initial,
      title,
      type,
      datasource,
      query,
      unit,
      min,
      max,
      options,
      catalogId,
      builder,
      advanced,
    ],
  )

  const updateBuilder = (patch: Partial<PanelBuilderState>) => {
    setCatalogId(undefined)
    setBuilder((prev) => ({ ...prev, ...patch }))
  }

  const matchers = builder.matchers || []

  const filteredCatalog = useMemo(() => {
    const q = catalogFilter.trim().toLowerCase()
    return catalog.filter((m) => {
      if (!q) return true
      return (
        m.label.toLowerCase().includes(q) ||
        m.id.toLowerCase().includes(q) ||
        (m.hint || '').toLowerCase().includes(q) ||
        m.category.toLowerCase().includes(q)
      )
    })
  }, [catalog, catalogFilter])

  const catalogByCategory = useMemo(() => {
    const groups = new Map<string, CatalogMetric[]>()
    for (const m of filteredCatalog) {
      const cat = m.category || 'other'
      if (!groups.has(cat)) groups.set(cat, [])
      groups.get(cat)!.push(m)
    }
    const keys = [
      ...CATEGORY_ORDER.filter((c) => groups.has(c)),
      ...[...groups.keys()].filter((c) => !CATEGORY_ORDER.includes(c)).sort(),
    ]
    return keys.map((k) => ({
      category: k,
      label: CATEGORY_LABELS[k] || k,
      items: groups.get(k) || [],
    }))
  }, [filteredCatalog])

  const applyCatalogEntry = (entry: CatalogMetric) => {
    const ds = entry.datasource
    const viz = asPanelType(entry.viz, ds)
    setCatalogId(entry.id)
    setCustomOpen(false)
    setAdvanced(false)
    setDatasource(ds)
    setQuery(entry.query)
    setType(viz)
    setUnit(entry.unit || 'none')
    setMin(entry.min ?? (viz === 'gauge' || viz === 'stat' ? 0 : null))
    setMax(
      entry.max ??
        (viz === 'gauge' || viz === 'stat'
          ? defaultGaugeMax(entry.unit || 'percent')
          : null),
    )
    setOptions(entry.options || defaultOptionsForType(viz))
    setTitle(entry.label)
    setTitleTouched(true)
    setTab('viz')
  }

  const onDatasourceChange = (next: PanelDatasource) => {
    setCatalogId(undefined)
    setDatasource(next)
    const b = defaultBuilder(next)
    setBuilder(b)
    setAdvanced(false)
    const t = defaultType(next)
    setType(t)
    setOptions(defaultOptionsForType(t))
    const nextUnit = next === 'loki' ? 'none' : 'percent'
    setUnit(nextUnit)
    setMin(t === 'gauge' || t === 'stat' ? 0 : null)
    setMax(t === 'gauge' || t === 'stat' ? defaultGaugeMax(nextUnit) : null)
    if (!titleTouched) setTitle(suggestTitle(next, b))
  }

  const onTypeChange = (next: PanelType) => {
    setType(next)
    setOptions((prev) => {
      const base = defaultOptionsForType(next)
      // Keep thresholds when switching between gauge ↔ stat
      if (
        (next === 'gauge' || next === 'stat') &&
        prev.thresholds?.length
      ) {
        return { ...base, thresholds: prev.thresholds }
      }
      return base
    })
    if (next === 'gauge' || next === 'stat') {
      setMin((m) => (m == null ? 0 : m))
      setMax((m) => (m == null ? defaultGaugeMax(unit) : m))
    }
  }

  const onUnitChange = (next: string) => {
    setUnit(next)
    if (
      (type === 'gauge' || type === 'stat') &&
      (max == null || max === defaultGaugeMax(unit))
    ) {
      setMax(defaultGaugeMax(next))
    }
  }

  const onPreset = (preset: ThresholdPreset) => {
    setUnit(preset.unit)
    setMin(preset.min)
    setMax(preset.max)
    setOptions((o) => ({
      ...o,
      thresholds: preset.thresholds.map((t) => ({ ...t })),
    }))
  }

  const dataTab = (
    <Form layout="vertical" size="middle">
      <Form.Item label="Title" required>
        <Input
          value={title}
          onChange={(e) => {
            setTitleTouched(true)
            setTitle(e.target.value)
          }}
        />
      </Form.Item>

      {catalogError ? (
        <Alert
          type="error"
          showIcon
          message={catalogError}
          style={{ marginBottom: 12 }}
        />
      ) : null}

      <Form.Item
        label="Pick a metric"
        extra="Curated formulas from config/metric_catalog.yaml — no PromQL required."
      >
        <Input
          allowClear
          placeholder="Search CPU, memory, disk, temp, logs…"
          value={catalogFilter}
          onChange={(e) => setCatalogFilter(e.target.value)}
          style={{ marginBottom: 10 }}
        />
        <div
          style={{
            maxHeight: 280,
            overflowY: 'auto',
            border: '1px solid #1e293b',
            borderRadius: 8,
            padding: 8,
            background: '#0a0e14',
          }}
        >
          {!catalog.length && !catalogError ? (
            <Text type="secondary">Loading catalog…</Text>
          ) : null}
          {catalogByCategory.map((group) => (
            <div key={group.category} style={{ marginBottom: 12 }}>
              <Text
                type="secondary"
                style={{
                  fontSize: 11,
                  textTransform: 'uppercase',
                  letterSpacing: 0.6,
                }}
              >
                {group.label}
              </Text>
              <div
                style={{
                  display: 'grid',
                  gridTemplateColumns: 'repeat(2, minmax(0, 1fr))',
                  gap: 6,
                  marginTop: 6,
                }}
              >
                {group.items.map((m) => {
                  const active = catalogId === m.id
                  return (
                    <button
                      key={m.id}
                      type="button"
                      onClick={() => applyCatalogEntry(m)}
                      style={{
                        textAlign: 'left',
                        padding: '8px 10px',
                        borderRadius: 8,
                        border: active
                          ? '1px solid #38bdf8'
                          : '1px solid #1e293b',
                        background: active
                          ? 'rgba(56,189,248,0.12)'
                          : '#111827',
                        cursor: 'pointer',
                        color: '#e2e8f0',
                      }}
                    >
                      <div style={{ fontSize: 12, fontWeight: 600 }}>
                        {m.label}
                      </div>
                      <div style={{ fontSize: 10, color: '#64748b' }}>
                        {m.hint || m.id} · {m.viz}
                      </div>
                    </button>
                  )
                })}
              </div>
            </div>
          ))}
          {catalog.length > 0 && !filteredCatalog.length ? (
            <Text type="secondary">No metrics match that search.</Text>
          ) : null}
        </div>
      </Form.Item>

      {catalogId ? (
        <Alert
          type="success"
          showIcon
          style={{ marginBottom: 12 }}
          message={`Using catalog: ${catalogId}`}
          description="Query and defaults are filled. Adjust Visualization / Style tabs as needed."
        />
      ) : null}

      <Collapse
        ghost
        activeKey={customOpen ? ['custom'] : []}
        onChange={(keys) => {
          const open = (keys as string[]).includes('custom')
          setCustomOpen(open)
          if (open) setCatalogId(undefined)
        }}
        items={[
          {
            key: 'custom',
            label: 'Custom query (advanced)',
            children: (
              <>
      <Form.Item label="Datasource">
        <Select
          value={datasource}
          onChange={onDatasourceChange}
          options={DS.map((d) => ({ value: d, label: d }))}
        />
      </Form.Item>

      {metaError ? (
        <Alert
          type="error"
          showIcon
          message={metaError}
          style={{ marginBottom: 12 }}
        />
      ) : null}

      {!advanced && datasource === 'prometheus' ? (
        <div
          style={{
            border: '1px solid #1e293b',
            borderRadius: 8,
            padding: 14,
            marginBottom: 12,
            background: 'linear-gradient(180deg,#111827 0%,#0d121a 100%)',
          }}
        >
          <Text
            strong
            style={{ display: 'block', marginBottom: 10, color: '#e2e8f0' }}
          >
            Query builder
          </Text>
          <Form.Item label="Metric" style={{ marginBottom: 12 }}>
            <Select
              showSearch
              placeholder="Search metrics…"
              value={builder.metric || undefined}
              onChange={(v) => updateBuilder({ metric: v })}
              optionFilterProp="label"
              options={metrics.map((m) => ({ value: m, label: m }))}
              listHeight={240}
            />
          </Form.Item>

          <Form.Item label="Label matchers" style={{ marginBottom: 8 }}>
            <Space direction="vertical" style={{ width: '100%' }}>
              {matchers.map((m, i) => (
                <Space.Compact key={i} style={{ width: '100%' }}>
                  <Select
                    style={{ width: '36%' }}
                    placeholder="label"
                    showSearch
                    value={m.label || undefined}
                    onChange={(label) => {
                      const next = [...matchers]
                      next[i] = { ...next[i], label }
                      updateBuilder({ matchers: next })
                      void loadLabelValues(label)
                    }}
                    onOpenChange={(open) => {
                      if (open && m.label) void loadLabelValues(m.label)
                    }}
                    options={(labelNames.includes(m.label) || !m.label
                      ? labelNames
                      : [m.label, ...labelNames]
                    ).map((l) => ({ value: l, label: l }))}
                  />
                  <AutoComplete
                    style={{ width: '52%' }}
                    placeholder="$device_id"
                    value={m.value}
                    onFocus={() => {
                      if (m.label) void loadLabelValues(m.label)
                    }}
                    onChange={(v) => {
                      const next = [...matchers]
                      next[i] = { ...next[i], value: v || '' }
                      updateBuilder({ matchers: next })
                    }}
                    options={[
                      { value: '$device_id' },
                      ...(labelValueCache[m.label] || []).map((v) => ({
                        value: v,
                      })),
                    ]}
                  />
                  <Button
                    danger
                    onClick={() =>
                      updateBuilder({
                        matchers: matchers.filter((_, j) => j !== i),
                      })
                    }
                  >
                    ×
                  </Button>
                </Space.Compact>
              ))}
              <Button
                type="dashed"
                size="small"
                onClick={() =>
                  updateBuilder({
                    matchers: [
                      ...matchers,
                      { label: 'instance', value: '$device_id' },
                    ],
                  })
                }
              >
                + matcher
              </Button>
            </Space>
          </Form.Item>

          <Row gutter={12}>
            <Col span={12}>
              <Form.Item label="Function" style={{ marginBottom: 12 }}>
                <Select
                  value={builder.fn || 'raw'}
                  onChange={(v) =>
                    updateBuilder({ fn: v as PromBuilderFn })
                  }
                  options={PROM_FNS.map((f) => ({
                    value: f.value,
                    label: f.label,
                  }))}
                />
              </Form.Item>
            </Col>
            <Col span={12}>
              <Form.Item label="Range window" style={{ marginBottom: 12 }}>
                <Select
                  value={builder.range || '5m'}
                  onChange={(v) => updateBuilder({ range: v })}
                  options={RANGE_OPTIONS.map((r) => ({
                    value: r,
                    label: `[${r}]`,
                  }))}
                />
              </Form.Item>
            </Col>
          </Row>

          <Form.Item label="Group by" style={{ marginBottom: 0 }}>
            <Select
              mode="tags"
              placeholder="instance, mode"
              value={builder.groupBy || []}
              onChange={(v) => updateBuilder({ groupBy: v })}
              options={labelNames.map((l) => ({ value: l, label: l }))}
              tokenSeparators={[',']}
            />
          </Form.Item>
        </div>
      ) : null}

      {!advanced && datasource === 'loki' ? (
        <div
          style={{
            border: '1px solid #1e293b',
            borderRadius: 8,
            padding: 14,
            marginBottom: 12,
            background: 'linear-gradient(180deg,#111827 0%,#0d121a 100%)',
          }}
        >
          <Form.Item label="Host" style={{ marginBottom: 12 }}>
            <Select
              showSearch
              value={builder.host || '$device_id'}
              onChange={(v) => updateBuilder({ host: v })}
              options={[
                { value: '$device_id', label: '$device_id (template)' },
                ...hosts.map((h) => ({ value: h, label: h })),
              ]}
            />
          </Form.Item>
          <Form.Item
            label="Container / compose service"
            style={{ marginBottom: 12 }}
          >
            <Select
              showSearch
              allowClear
              placeholder="All containers"
              value={builder.container || undefined}
              onChange={(v) => updateBuilder({ container: v || '' })}
              options={containers.map((c) => ({ value: c, label: c }))}
            />
          </Form.Item>
          <Form.Item label="Stream" style={{ marginBottom: 12 }}>
            <Select
              value={builder.stream || ''}
              onChange={(v) => updateBuilder({ stream: v })}
              options={[
                { value: '', label: 'Any' },
                { value: 'stdout', label: 'stdout' },
                { value: 'stderr', label: 'stderr' },
              ]}
            />
          </Form.Item>
          <Form.Item label="Line contains" style={{ marginBottom: 0 }}>
            <Input
              value={builder.lineFilter || ''}
              onChange={(e) =>
                updateBuilder({ lineFilter: e.target.value })
              }
              placeholder="error"
            />
          </Form.Item>
        </div>
      ) : null}

      <Form.Item style={{ marginBottom: 8 }}>
        <Checkbox
          checked={advanced}
          onChange={(e) => {
            setAdvanced(e.target.checked)
            if (e.target.checked) setCatalogId(undefined)
          }}
        >
          Edit raw query
        </Checkbox>
      </Form.Item>

      <Form.Item
        label={`Query (${datasource === 'prometheus' ? 'PromQL' : 'LogQL'})`}
        extra={
          !advanced
            ? 'Auto-generated from builder — enable Edit raw query to override'
            : undefined
        }
      >
        <TextArea
          rows={4}
          value={query}
          readOnly={!advanced}
          onChange={(e) => {
            if (advanced) {
              setCatalogId(undefined)
              setQuery(e.target.value)
            }
          }}
          style={{
            fontFamily: 'ui-monospace, SFMono-Regular, Menlo, monospace',
            fontSize: 12,
            opacity: advanced ? 1 : 0.85,
          }}
        />
      </Form.Item>
              </>
            ),
          },
        ]}
      />

      {!customOpen ? (
        <Form.Item label="Query" style={{ marginTop: 8 }}>
          <TextArea
            rows={3}
            value={query}
            readOnly
            style={{
              fontFamily: 'ui-monospace, SFMono-Regular, Menlo, monospace',
              fontSize: 11,
              opacity: 0.85,
            }}
          />
        </Form.Item>
      ) : null}
    </Form>
  )

  const vizTab = (
    <Form layout="vertical" size="middle">
      <Text type="secondary" style={{ display: 'block', marginBottom: 10 }}>
        Choose how this panel looks. Style options are on the next tab.
      </Text>
      <div
        style={{
          display: 'grid',
          gridTemplateColumns: 'repeat(4, minmax(0, 1fr))',
          gap: 8,
          marginBottom: 16,
        }}
      >
        {TYPES.filter((t) =>
          datasource === 'loki'
            ? t.value === 'logs' || t.value === 'table'
            : t.value !== 'logs',
        ).map((t) => {
          const Icon = t.icon
          const active = type === t.value
          return (
            <button
              key={t.value}
              type="button"
              onClick={() => onTypeChange(t.value)}
              style={{
                textAlign: 'left',
                padding: '10px 10px',
                borderRadius: 10,
                border: active
                  ? '1px solid #38bdf8'
                  : '1px solid #1e293b',
                background: active
                  ? 'rgba(56,189,248,0.12)'
                  : '#0d121a',
                cursor: 'pointer',
                color: '#e2e8f0',
              }}
            >
              <Icon
                size={16}
                style={{ color: active ? '#38bdf8' : '#94a3b8' }}
              />
              <div style={{ fontSize: 12, fontWeight: 600, marginTop: 6 }}>
                {t.label}
              </div>
              <div style={{ fontSize: 10, color: '#64748b' }}>{t.hint}</div>
            </button>
          )
        })}
      </div>

      {datasource === 'prometheus' ? (
        <Form.Item label="Unit">
          <Select
            showSearch
            value={unit}
            onChange={onUnitChange}
            optionFilterProp="label"
            options={UNIT_OPTIONS.map((u) => ({
              value: u.value,
              label: u.label,
            }))}
          />
        </Form.Item>
      ) : null}

      {type === 'gauge' || type === 'stat' ? (
        <Row gutter={12}>
          <Col span={12}>
            <Form.Item
              label="Min"
              validateStatus={
                max != null && min != null && max <= min ? 'error' : undefined
              }
              help={
                max != null && min != null && max <= min
                  ? 'Max must be greater than min'
                  : undefined
              }
            >
              <InputNumber
                style={{ width: '100%' }}
                value={min}
                onChange={(v) => setMin(v)}
              />
            </Form.Item>
          </Col>
          <Col span={12}>
            <Form.Item label="Max">
              <InputNumber
                style={{ width: '100%' }}
                value={max}
                onChange={(v) => setMax(v)}
              />
            </Form.Item>
          </Col>
        </Row>
      ) : null}

      {type === 'gauge' ? (
        <Form.Item
          label="Stages"
          extra="Colored bands on the gauge. Edit labels and “up to” values here or under Style."
        >
          <ThresholdEditor
            value={options.thresholds}
            min={min ?? 0}
            max={max ?? 100}
            onChange={(thresholds) =>
              setOptions((o) => ({ ...o, thresholds }))
            }
            onPreset={onPreset}
          />
        </Form.Item>
      ) : null}
    </Form>
  )

  return (
    <ConfigProvider
      theme={{
        algorithm: theme.darkAlgorithm,
        token: {
          colorPrimary: '#38bdf8',
          colorBgBase: '#0d121a',
          colorBgContainer: '#111827',
          colorBorder: '#1e293b',
          borderRadius: 8,
          fontSize: 13,
        },
      }}
    >
      <Modal
        open
        title={
          <div>
            <Title level={4} style={{ margin: 0 }}>
              {initial?.id ? 'Edit panel' : 'Add panel'}
            </Title>
            <Text type="secondary" style={{ fontSize: 12, fontWeight: 400 }}>
              Guided builder · use{' '}
              <code style={{ color: '#38bdf8' }}>$device_id</code> for device
              scope
            </Text>
          </div>
        }
        onCancel={onCancel}
        width={1100}
        destroyOnHidden
        styles={{
          body: { maxHeight: '72vh', overflow: 'hidden', paddingTop: 8 },
        }}
        footer={[
          <Button key="cancel" onClick={onCancel}>
            Cancel
          </Button>,
          <Button
            key="save"
            type="primary"
            disabled={!title.trim() || !query.trim()}
            onClick={() =>
              onSave({
                ...preview,
                id: initial?.id || newId(),
              })
            }
          >
            Save panel
          </Button>,
        ]}
      >
        <Row gutter={20} style={{ height: '100%' }}>
          <Col
            xs={24}
            md={14}
            style={{ maxHeight: '64vh', overflowY: 'auto', paddingRight: 4 }}
          >
            <Tabs
              activeKey={tab}
              onChange={setTab}
              items={[
                { key: 'data', label: 'Data', children: dataTab },
                { key: 'viz', label: 'Visualization', children: vizTab },
                {
                  key: 'style',
                  label: 'Style',
                  children: (
                    <VizOptionsForm
                      type={type}
                      options={options}
                      min={min}
                      max={max}
                      onChange={setOptions}
                      onPreset={onPreset}
                    />
                  ),
                },
              ]}
            />
          </Col>
          <Col xs={24} md={10}>
            <div
              style={{
                position: 'sticky',
                top: 0,
                border: '1px solid #1e293b',
                borderRadius: 12,
                padding: 10,
                background:
                  'radial-gradient(120% 80% at 50% 0%, rgba(56,189,248,0.08), transparent), #0a0e14',
              }}
            >
              <Text
                type="secondary"
                style={{ display: 'block', marginBottom: 8, fontSize: 11 }}
              >
                Live preview
              </Text>
              <Panel
                panel={preview}
                deviceId={deviceId}
                sinceSeconds={3600}
                className="h-80"
              />
            </div>
          </Col>
        </Row>
      </Modal>
    </ConfigProvider>
  )
}
