"use client"

import { useMemo } from "react"
import { cn } from "@/lib/utils"

export default function Sparkline({
  values = [],
  width = 600,
  height = 80,
  tone = "neutral",
  className,
}: {
  values?: number[]
  width?: number
  height?: number
  tone?: "neutral" | "brand"
  className?: string
}) {
  const d = useMemo(() => {
    if (!values.length) return ""
    const max = Math.max(...values)
    const min = Math.min(...values)
    const span = Math.max(1, max - min)
    const step = width / (values.length - 1)
    return values
      .map((v, i) => {
        const x = i * step
        const y = height - ((v - min) / span) * height
        return `${x},${y}`
      })
      .join(" ")
  }, [values, width, height])

  const lineStart = tone === "neutral" ? "rgb(23,23,23)" : "rgb(168,85,247)"
  const lineEnd = tone === "neutral" ? "rgb(115,115,115)" : "rgb(244,63,94)"

  return (
    <div className={cn("w-full overflow-hidden", className)}>
      <svg
        viewBox={`0 0 ${width} ${height}`}
        width="100%"
        height={height}
        className="drop-shadow-[0_2px_8px_rgba(0,0,0,0.1)]"
      >
        <defs>
          <linearGradient id="sparkline" x1="0" x2="1" y1="0" y2="0">
            <stop offset="0%" stopColor={lineStart} stopOpacity="1" />
            <stop offset="100%" stopColor={lineEnd} stopOpacity="1" />
          </linearGradient>
          <linearGradient id="fill" x1="0" x2="0" y1="0" y2="1">
            <stop offset="0%" stopColor="rgba(0,0,0,0.08)" />
            <stop offset="100%" stopColor="rgba(0,0,0,0.01)" />
          </linearGradient>
        </defs>
        <polyline points={`0,${height} ${d} ${width},${height}`} fill="url(#fill)" stroke="none" />
        <polyline
          points={d}
          fill="none"
          stroke="url(#sparkline)"
          strokeWidth="3"
          strokeLinecap="round"
          strokeLinejoin="round"
        />
      </svg>
    </div>
  )
}
