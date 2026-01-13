type Props = {
  phases: string[]
  index: number // -1 idle; 0..n active/completed; >=n done
}

export default function PhaseTimeline({ phases, index }: Props) {
  return (
    <div className="space-y-3">
      <div className="flex items-center gap-2 text-sm">
        <span>Current:</span>
        <span className={`px-2 py-0.5 rounded-md text-xs ${index < 0 ? 'bg-white/10 text-white/70' : 'bg-amber-400/15 text-amber-400'}`}>
          {index < 0 ? 'Idle' : phases[Math.min(index, phases.length - 1)]}
        </span>
      </div>
      <div className="rounded-md border border-slate-800 overflow-hidden">
        <div className="flex">
          {phases.map((p, i) => {
            const completed = index > i
            const active = index === i
            const bg = completed ? 'bg-emerald-400/20' : active ? 'bg-amber-400/20' : 'bg-white/5'
            const border = 'border-l border-slate-900'
            return (
              <div key={p} className={`flex-1 ${bg} ${i > 0 ? border : ''} text-center text-xs py-2 text-white/80`}>{p}</div>
            )
          })}
        </div>
      </div>
      <div className="text-sm">Progress</div>
      <div className="h-2 rounded-full bg-white/10">
        <div
          className={`h-full rounded-full ${index < 0 ? 'bg-white/10' : index >= phases.length ? 'bg-emerald-400' : 'bg-amber-400'}`}
          style={{ width: `${Math.max(0, Math.min(index + 1, phases.length)) / phases.length * 100}%` }}
        />
      </div>
    </div>
  )
}













