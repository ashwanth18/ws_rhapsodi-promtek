import { SelectHTMLAttributes } from 'react'
import { cn } from '../../lib/cn'

type Props = SelectHTMLAttributes<HTMLSelectElement>

export default function Select({ className = '', ...props }: Props) {
  return (
    <select
      className={cn(
        'rounded-md border border-[var(--border)] bg-[var(--surface)] px-2 py-1 text-sm text-[var(--text-primary)]',
        'focus:outline-none focus:ring-2 focus:ring-[#22d3ee] focus:border-[#22d3ee]',
        className
      )}
      {...props}
    />
  )}













