import { SelectHTMLAttributes } from 'react'
import { cn } from '../../lib/cn'

type Props = SelectHTMLAttributes<HTMLSelectElement>

export default function Select({ className = '', ...props }: Props) {
  return (
    <select
      className={cn(
        'bg-[#0a0b0d] border border-slate-800 rounded-md px-2 py-1 text-sm text-white/90',
        'focus:outline-none focus:ring-2 focus:ring-[#22d3ee] focus:border-[#22d3ee]',
        className
      )}
      {...props}
    />
  )}













