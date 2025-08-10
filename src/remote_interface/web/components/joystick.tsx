"use client"

import type React from "react"
import { useEffect, useRef, useState } from "react"
import { cn } from "@/lib/utils"

type Props = {
  size?: number
  springBack?: boolean
  onChange?: (v: { x: number; y: number }) => void
  className?: string
  theme?: "light" | "dark"
}

export default function Joystick({ size = 220, springBack = true, onChange, className, theme = "light" }: Props) {
  const radius = size / 2
  const knobRadius = size * 0.16
  const ref = useRef<HTMLDivElement>(null)
  const [dragging, setDragging] = useState(false)
  const [pos, setPos] = useState({ x: 0, y: 0 })
  const scale = dragging ? 1.05 : 1

  useEffect(() => {
    onChange?.({ x: +(pos.x / (radius - knobRadius)).toFixed(2), y: +(pos.y / (radius - knobRadius)).toFixed(2) })
  }, [pos]) // eslint-disable-line

  function setFromEvent(e: PointerEvent | React.PointerEvent) {
    const el = ref.current
    if (!el) return
    const rect = el.getBoundingClientRect()
    const cx = rect.left + rect.width / 2
    const cy = rect.top + rect.height / 2
    const x = ("clientX" in e ? e.clientX : 0) - cx
    const y = ("clientY" in e ? e.clientY : 0) - cy
    const max = radius - knobRadius
    const len = Math.hypot(x, y)
    const scale = len > max ? max / len : 1
    setPos({ x: x * scale, y: y * scale })
  }

  function handleDown(e: React.PointerEvent) {
    setDragging(true)
    ;(e.target as Element).setPointerCapture?.(e.pointerId)
    setFromEvent(e)
  }
  function handleMove(e: React.PointerEvent) {
    if (!dragging) return
    setFromEvent(e)
  }
  function handleUp() {
    setDragging(false)
    if (springBack) setPos({ x: 0, y: 0 })
  }

  const ringBorder = "border-neutral-200"
  const ringBg = "bg-white/70"
  const textTone = theme === "dark" ? "text-neutral-300" : "text-neutral-500"

  return (
    <div className={cn("select-none", className)} style={{ width: size, height: size }}>
      <div
        ref={ref}
        onPointerDown={handleDown}
        onPointerMove={handleMove}
        onPointerUp={handleUp}
        onPointerCancel={handleUp}
        className={cn(
          "relative grid place-items-center rounded-full",
          ringBorder,
          ringBg,
          "border backdrop-blur-xl shadow-inner",
        )}
        style={{
          width: size,
          height: size,
          boxShadow: "inset 0 10px 30px rgba(0,0,0,0.12), inset 0 -6px 12px rgba(255,255,255,0.6)",
        }}
      >
        {/* Knob */}
        <div
          className={cn(
            "absolute rounded-full border border-neutral-400 bg-transparent",
            "shadow-[0_10px_24px_rgba(0,0,0,0.18)]",
          )}
          style={{
            width: knobRadius * 2,
            height: knobRadius * 2,
            transform: `translate(${pos.x}px, ${pos.y}px) scale(${scale})`,
          }}
        >
          {/* Sutil brillo superior */}
          <div
            className="absolute inset-0 rounded-full opacity-60 text-transparent"
            style={{
              background:
                "radial-gradient(50% 50% at 50% 38%, rgba(255,255,255,0.85) 0%, rgba(255,255,255,0.45) 45%, rgba(255,255,255,0.20) 100%)",
            }}
          />
          {/* Anillo de separación para que no se pierda con el fondo del aro */}
          <div className="pointer-events-none absolute inset-0 rounded-full ring-1 ring-black/10" />
          {/* Núcleo oscuro con contraste tipo fondo UI */}
          <div
            className="absolute left-1/2 top-1/2 -translate-x-1/2 -translate-y-1/2 rounded-full border border-white/30 shadow-[inset_0_6px_12px_rgba(255,255,255,0.18),0_8px_16px_rgba(0,0,0,0.25)] bg-black"
            style={{
              width: knobRadius * 1.24,
              height: knobRadius * 1.24,
              background: "linear-gradient(180deg, rgb(0 0 0) 0%, rgb(64 64 64) 100%)",
            }}
          />
        </div>
        {/* Directions */}
        <div className={cn("absolute left-1/2 top-2 -translate-x-1/2 text-xs", textTone)}>Adelante</div>
        <div className={cn("absolute left-1/2 bottom-2 -translate-x-1/2 text-xs", textTone)}>Atrás</div>
        <div className={cn("absolute top-1/2 left-2 -translate-y-1/2 text-xs", textTone)}>Izquierda</div>
        <div className={cn("absolute top-1/2 right-2 -translate-y-1/2 text-xs", textTone)}>Derecha</div>
      </div>
    </div>
  )
}
