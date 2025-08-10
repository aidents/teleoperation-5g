import { cn } from "@/lib/utils"
import type { ReactNode } from "react"

export default function GlassCard({
  title,
  icon,
  headerRight,
  className,
  children,
}: {
  title?: string
  icon?: ReactNode
  headerRight?: ReactNode
  className?: string
  children: ReactNode
}) {
  return (
    <section
      className={cn(
        // Acrílico claro, neutro
        "rounded-3xl border border-neutral-300/80 bg-white/85 backdrop-blur-xl shadow-[0_20px_60px_rgba(0,0,0,0.25)]",
        className,
      )}
    >
      {title ? (
        <div className="flex items-center justify-between gap-3 border-b border-neutral-200/80 px-4 py-3 sm:px-6">
          <div className="flex items-center gap-2">
            {icon ? (
              <div className="inline-grid size-8 place-items-center rounded-2xl border border-neutral-300 bg-white/80">
                {icon}
              </div>
            ) : null}
            <h2 className="text-base sm:text-lg font-semibold tracking-tight text-neutral-900">{title}</h2>
          </div>
          {headerRight}
        </div>
      ) : null}
      {children}
    </section>
  )
}
