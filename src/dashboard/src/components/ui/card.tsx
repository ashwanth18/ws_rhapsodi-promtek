import { HTMLAttributes } from 'react'
import { cn } from '../../lib/cn'

export function Card({ className = '', ...props }: HTMLAttributes<HTMLDivElement>) {
  return (
    <div
      className={cn(
        'rounded-xl border border-white/10 bg-white/5 shadow-lg backdrop-blur',
        className
      )}
      {...props}
    />
  )
}

export function CardHeader({ className = '', ...props }: HTMLAttributes<HTMLDivElement>) {
  return (
    <div className={cn('px-4 pt-4', className)} {...props} />
  )
}

export function CardTitle({ className = '', ...props }: HTMLAttributes<HTMLHeadingElement>) {
  return (
    <h3 className={cn('text-sm font-semibold text-white/90', className)} {...props} />
  )
}

export function CardContent({ className = '', ...props }: HTMLAttributes<HTMLDivElement>) {
  return (
    <div className={cn('px-4 pb-4 pt-2', className)} {...props} />
  )
}





