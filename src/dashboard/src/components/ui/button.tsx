import { ButtonHTMLAttributes, forwardRef } from 'react'
import { cn } from '../../lib/cn'

type Props = ButtonHTMLAttributes<HTMLButtonElement> & {
  variant?: 'primary' | 'ghost' | 'danger' | 'outline'
  size?: 'sm' | 'md' | 'lg'
}

const Button = forwardRef<HTMLButtonElement, Props>(
  ({ className = '', variant = 'primary', size = 'md', ...props }, ref) => {
    const base =
      'inline-flex items-center justify-center rounded-[var(--radius-sm)] font-medium transition-colors disabled:opacity-50 disabled:cursor-not-allowed'
    const sizes = {
      sm: 'text-xs px-3 py-1.5 gap-1.5',
      md: 'text-sm px-4 py-2 gap-2',
      lg: 'text-sm px-5 py-2.5 gap-2',
    }
    const variants = {
      primary:
        'bg-[var(--button-primary-bg)] text-[var(--button-primary-text)] hover:bg-[var(--button-primary-hover)]',
      ghost:
        'bg-transparent text-[var(--button-ghost-text)] hover:bg-[var(--hover-surface)]',
      danger:
        'bg-[var(--status-bad-bg)] text-[var(--status-bad-fg)] hover:opacity-90',
      outline:
        'border border-[var(--border-strong)] bg-transparent text-[var(--text-secondary)] hover:bg-[var(--hover-surface)]',
    }

    return (
      <button
        ref={ref}
        className={cn(base, sizes[size], variants[variant], className)}
        {...props}
      />
    )
  }
)

Button.displayName = 'Button'
export default Button
