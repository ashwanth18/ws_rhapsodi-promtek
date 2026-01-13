import { ReactNode } from 'react'

export default function GlassCard({ children, className = '' }: { children: ReactNode; className?: string }) {
  return (
    <div
      className={
        'rounded-[12px] p-4 border border-slate-800 bg-[rgba(10,11,13,0.65)] ' +
        'backdrop-blur-md shadow-[inset_0_1px_0_rgba(255,255,255,0.04),0_10px_30px_rgba(0,0,0,0.45)] ' +
        'transition-transform duration-200 hover:-translate-y-0.5 hover:shadow-[inset_0_1px_0_rgba(255,255,255,0.06),0_16px_40px_rgba(0,0,0,0.55)] ' +
        className
      }
    >
      {children}
    </div>
  )
}


