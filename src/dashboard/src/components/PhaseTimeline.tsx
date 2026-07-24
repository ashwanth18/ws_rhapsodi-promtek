type Props = {
  phases: string[]
  index: number // -1 idle; 0..n active/completed; >=n done
}

export default function PhaseTimeline({ phases, index }: Props) {
  const currentLabel =
    index < 0
      ? 'Idle'
      : index >= phases.length
        ? 'Done'
        : phases[index]

  return (
    <div className="space-y-3">
      <div className="flex items-center gap-2 text-sm">
        <span>Current:</span>
        <span
          className={`rounded-md px-2 py-0.5 text-xs ${
            index < 0
              ? 'bg-[var(--status-idle-bg)] text-[var(--status-idle-fg)]'
              : 'bg-[var(--status-warn-bg)] text-[var(--status-warn-fg)]'
          }`}
        >
          {currentLabel}
        </span>
      </div>
      <div className="overflow-hidden rounded-md border border-[var(--border)]">
        <div className="flex">
          {phases.map((p, i) => {
            const completed = index > i
            const active = index === i
            const bg = completed
              ? 'bg-[var(--status-good-bg)]'
              : active
                ? 'bg-[var(--status-warn-bg)]'
                : 'bg-[var(--surface-muted)]'
            const border = 'border-l border-[var(--border)]'
            return (
              <div
                key={p}
                className={`flex-1 py-2 text-center text-xs text-[var(--text-secondary)] ${bg} ${
                  i > 0 ? border : ''
                }`}
              >
                {p}
              </div>
            )
          })}
        </div>
      </div>
      <div className="text-sm">Progress</div>
      <div className="h-2 rounded-full bg-[var(--surface-strong)]">
        <div
          className={`h-full rounded-full ${
            index < 0
              ? 'bg-[var(--surface-strong)]'
              : index >= phases.length
                ? 'bg-[var(--status-good-fg)]'
                : 'bg-[var(--status-warn-fg)]'
          }`}
          style={{ width: `${Math.max(0, Math.min(index + 1, phases.length)) / phases.length * 100}%` }}
        />
      </div>
    </div>
  )
}













