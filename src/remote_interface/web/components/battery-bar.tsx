"use client"

import { cn } from "@/lib/utils"

export default function BatteryBar({
  percent = 100,
  className,
}: {
  percent?: number
  className?: string
}) {
  const pct = Math.max(0, Math.min(100, percent))
  const isLow = pct < 20

  return (
    <div className={cn("h-3 w-full rounded-full border border-neutral-200 bg-neutral-100", className)}>
      <div
        className="h-full rounded-full transition-[width] duration-300 ease-out"
        style={{
          width: `${pct}%`,
          background: isLow
            ? "linear-gradient(90deg, rgb(239,68,68) 0%, rgb(220,38,38) 100%)"
            : "linear-gradient(90deg, rgb(23,23,23) 0%, rgb(64,64,64) 100%)",
          boxShadow: "0 0 24px rgba(0,0,0,0.06) inset",
        }}
      />
    </div>
  )
}
