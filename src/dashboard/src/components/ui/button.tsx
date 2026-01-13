import { ButtonHTMLAttributes, forwardRef } from 'react'
import { cn } from '../../lib/cn'

type Props = ButtonHTMLAttributes<HTMLButtonElement> & {
  variant?: 'primary' | 'ghost'
  size?: 'sm' | 'md'
}

const Button = forwardRef<HTMLButtonElement, Props>(({ className = '', variant = 'primary', size = 'md', ...props }, ref) => {
  const base = 'inline-flex items-center justify-center rounded-md font-semibold transition-colors'
  const sizes = size === 'sm' ? 'text-sm px-3 py-1.5' : 'text-sm px-4 py-2'
  const variants =
    variant === 'primary'
      ? 'bg-[#38bdf8] text-black hover:bg-[#60a5fa] focus:ring-2 focus:ring-[#22d3ee]'
      : 'bg-transparent text-white/80 hover:bg-white/5'

  return <button ref={ref} className={cn(base, sizes, variants, className)} {...props} />
})

export default Button













