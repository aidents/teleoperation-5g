"use client"

import React, { useEffect, useRef, useState, useCallback } from "react"
import { cn } from "@/lib/utils"
import { useROS2WebSocket } from "@/hooks/useROS2WebSocket"

interface JoystickControl {
  x: number;  // Roll (izquierda/derecha)
  y: number;  // Pitch (adelante/atrás)
  z: number;  // Throttle (altura)
  yaw: number; // Yaw (rotación)
}

interface ROS2JoystickProps {
  size?: number
  className?: string
  theme?: "light" | "dark"
  onControlChange?: (control: JoystickControl) => void
}

export default function ROS2Joystick({ 
  size = 220, 
  className, 
  theme = "light",
  onControlChange 
}: ROS2JoystickProps) {
  const { sendControl, isConnected, connectionStatus } = useROS2WebSocket()
  
  const [mainJoystick, setMainJoystick] = useState({ x: 0, y: 0 })
  const [throttle, setThrottle] = useState(0)
  const [yaw, setYaw] = useState(0)
  
  const mainRef = useRef<HTMLDivElement>(null)
  const throttleRef = useRef<HTMLDivElement>(null)
  const yawRef = useRef<HTMLDivElement>(null)
  
  const [dragging, setDragging] = useState<'main' | 'throttle' | 'yaw' | null>(null)
  
  const radius = size / 2
  const knobRadius = size * 0.16

  // Función para enviar comandos de control a ROS2
  const sendROS2Control = useCallback((control: JoystickControl) => {
    if (isConnected) {
      sendControl(control.x, control.y, control.z, control.yaw)
      onControlChange?.(control)
    }
  }, [isConnected, sendControl, onControlChange])

  // Efecto para enviar comandos cuando cambien los controles
  useEffect(() => {
    const control: JoystickControl = {
      x: mainJoystick.x,
      y: mainJoystick.y,
      z: throttle,
      yaw: yaw
    }
    sendROS2Control(control)
  }, [mainJoystick, throttle, yaw, sendROS2Control])

  // Función para calcular posición del joystick principal
  const setMainJoystickFromEvent = useCallback((e: PointerEvent | React.PointerEvent) => {
    const el = mainRef.current
    if (!el) return
    
    const rect = el.getBoundingClientRect()
    const cx = rect.left + rect.width / 2
    const cy = rect.top + rect.height / 2
    const x = ("clientX" in e ? e.clientX : 0) - cx
    const y = ("clientY" in e ? e.clientY : 0) - cy
    const max = radius - knobRadius
    const len = Math.hypot(x, y)
    const scale = len > max ? max / len : 1
    
    const normalizedX = +(x * scale / (radius - knobRadius)).toFixed(2)
    const normalizedY = +(y * scale / (radius - knobRadius)).toFixed(2)
    
    setMainJoystick({ x: normalizedX, y: normalizedY })
  }, [radius, knobRadius])

  // Función para calcular posición del throttle
  const setThrottleFromEvent = useCallback((e: PointerEvent | React.PointerEvent) => {
    const el = throttleRef.current
    if (!el) return
    
    const rect = el.getBoundingClientRect()
    const height = rect.height
    const y = ("clientY" in e ? e.clientY : 0) - rect.top
    const normalizedY = Math.max(0, Math.min(1, 1 - (y / height)))
    setThrottle(+(normalizedY * 2 - 1).toFixed(2)) // Rango de -1 a 1
  }, [])

  // Función para calcular posición del yaw
  const setYawFromEvent = useCallback((e: PointerEvent | React.PointerEvent) => {
    const el = yawRef.current
    if (!el) return
    
    const rect = el.getBoundingClientRect()
    const width = rect.width
    const x = ("clientX" in e ? e.clientX : 0) - rect.left
    const normalizedX = Math.max(0, Math.min(1, x / width))
    setYaw(+(normalizedX * 2 - 1).toFixed(2)) // Rango de -1 a 1
  }, [])

  // Event handlers para el joystick principal
  const handleMainDown = (e: React.PointerEvent) => {
    setDragging('main')
    ;(e.target as Element).setPointerCapture?.(e.pointerId)
    setMainJoystickFromEvent(e)
  }

  const handleMainMove = (e: React.PointerEvent) => {
    if (dragging === 'main') {
      setMainJoystickFromEvent(e)
    }
  }

  const handleMainUp = () => {
    setDragging(null)
    setMainJoystick({ x: 0, y: 0 })
  }

  // Event handlers para el throttle
  const handleThrottleDown = (e: React.PointerEvent) => {
    setDragging('throttle')
    ;(e.target as Element).setPointerCapture?.(e.pointerId)
    setThrottleFromEvent(e)
  }

  const handleThrottleMove = (e: React.PointerEvent) => {
    if (dragging === 'throttle') {
      setThrottleFromEvent(e)
    }
  }

  const handleThrottleUp = () => {
    setDragging(null)
  }

  // Event handlers para el yaw
  const handleYawDown = (e: React.PointerEvent) => {
    setDragging('yaw')
    ;(e.target as Element).setPointerCapture?.(e.pointerId)
    setYawFromEvent(e)
  }

  const handleYawMove = (e: React.PointerEvent) => {
    if (dragging === 'yaw') {
      setYawFromEvent(e)
    }
  }

  const handleYawUp = () => {
    setDragging(null)
    setYaw(0)
  }

  const ringBorder = "border-neutral-200"
  const ringBg = "bg-white/70"
  const textTone = theme === "dark" ? "text-neutral-300" : "text-neutral-500"

  return (
    <div className={cn("flex flex-col gap-4", className)}>
      {/* Estado de conexión */}
      <div className="flex items-center gap-2 text-sm">
        <div className={cn(
          "w-2 h-2 rounded-full",
          connectionStatus === 'connected' ? "bg-green-500" :
          connectionStatus === 'connecting' ? "bg-yellow-500" :
          connectionStatus === 'error' ? "bg-red-500" : "bg-gray-500"
        )} />
        <span className={textTone}>
          {connectionStatus === 'connected' ? 'Conectado a ROS2' :
           connectionStatus === 'connecting' ? 'Conectando...' :
           connectionStatus === 'error' ? 'Error de conexión' : 'Desconectado'}
        </span>
      </div>

      <div className="flex gap-6">
        {/* Joystick principal (Roll/Pitch) */}
        <div className="flex flex-col items-center">
          <div className={cn("select-none")} style={{ width: size, height: size }}>
            <div
              ref={mainRef}
              onPointerDown={handleMainDown}
              onPointerMove={handleMainMove}
              onPointerUp={handleMainUp}
              onPointerCancel={handleMainUp}
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
                  transform: `translate(${mainJoystick.x * (radius - knobRadius)}px, ${mainJoystick.y * (radius - knobRadius)}px)`,
                }}
              >
                <div className="absolute inset-0 rounded-full opacity-60 text-transparent"
                     style={{
                       background: "radial-gradient(50% 50% at 50% 38%, rgba(255,255,255,0.85) 0%, rgba(255,255,255,0.45) 45%, rgba(255,255,255,0.20) 100%)",
                     }} />
                <div className="pointer-events-none absolute inset-0 rounded-full ring-1 ring-black/10" />
                <div className="absolute left-1/2 top-1/2 -translate-x-1/2 -translate-y-1/2 rounded-full border border-white/30 shadow-[inset_0_6px_12px_rgba(255,255,255,0.18),0_8px_16px_rgba(0,0,0,0.25)] bg-black"
                     style={{
                       width: knobRadius * 1.24,
                       height: knobRadius * 1.24,
                       background: "linear-gradient(180deg, rgb(0 0 0) 0%, rgb(64 64 64) 100%)",
                     }} />
              </div>
              
              {/* Direcciones */}
              <div className={cn("absolute left-1/2 top-2 -translate-x-1/2 text-xs", textTone)}>Adelante</div>
              <div className={cn("absolute left-1/2 bottom-2 -translate-x-1/2 text-xs", textTone)}>Atrás</div>
              <div className={cn("absolute top-1/2 left-2 -translate-y-1/2 text-xs", textTone)}>Izquierda</div>
              <div className={cn("absolute top-1/2 right-2 -translate-y-1/2 text-xs", textTone)}>Derecha</div>
            </div>
          </div>
          <div className="mt-2 text-center">
            <div className="text-xs text-gray-500">Roll/Pitch</div>
            <div className="text-sm font-mono">X: {mainJoystick.x.toFixed(2)} Y: {mainJoystick.y.toFixed(2)}</div>
          </div>
        </div>

        {/* Controles verticales */}
        <div className="flex flex-col gap-4">
          {/* Throttle */}
          <div className="flex flex-col items-center">
            <div className={cn("select-none relative")} style={{ width: size * 0.3, height: size }}>
              <div
                ref={throttleRef}
                onPointerDown={handleThrottleDown}
                onPointerMove={handleThrottleMove}
                onPointerUp={handleThrottleUp}
                onPointerCancel={handleThrottleUp}
                className={cn(
                  "relative rounded-full",
                  ringBorder,
                  ringBg,
                  "border backdrop-blur-xl shadow-inner",
                )}
                style={{
                  width: size * 0.3,
                  height: size,
                  boxShadow: "inset 0 10px 30px rgba(0,0,0,0.12), inset 0 -6px 12px rgba(255,255,255,0.6)",
                }}
              >
                {/* Indicador de throttle */}
                <div
                  className="absolute left-1/2 w-full h-2 bg-blue-500 rounded-full transition-all duration-100"
                  style={{
                    bottom: `${(throttle + 1) * 50}%`,
                    transform: 'translateX(-50%)',
                  }}
                />
              </div>
              <div className="absolute -top-6 left-1/2 -translate-x-1/2 text-xs text-gray-500">Arriba</div>
              <div className="absolute -bottom-6 left-1/2 -translate-x-1/2 text-xs text-gray-500">Abajo</div>
            </div>
            <div className="mt-2 text-center">
              <div className="text-xs text-gray-500">Throttle</div>
              <div className="text-sm font-mono">{throttle.toFixed(2)}</div>
            </div>
          </div>

          {/* Yaw */}
          <div className="flex flex-col items-center">
            <div className={cn("select-none relative")} style={{ width: size, height: size * 0.3 }}>
              <div
                ref={yawRef}
                onPointerDown={handleYawDown}
                onPointerMove={handleYawMove}
                onPointerUp={handleYawUp}
                onPointerCancel={handleYawUp}
                className={cn(
                  "relative rounded-full",
                  ringBorder,
                  ringBg,
                  "border backdrop-blur-xl shadow-inner",
                )}
                style={{
                  width: size,
                  height: size * 0.3,
                  boxShadow: "inset 0 10px 30px rgba(0,0,0,0.12), inset 0 -6px 12px rgba(255,255,255,0.6)",
                }}
              >
                {/* Indicador de yaw */}
                <div
                  className="absolute top-1/2 h-full w-2 bg-green-500 rounded-full transition-all duration-100"
                  style={{
                    left: `${(yaw + 1) * 50}%`,
                    transform: 'translateY(-50%)',
                  }}
                />
              </div>
              <div className="absolute top-1/2 -left-6 -translate-y-1/2 text-xs text-gray-500">Izq</div>
              <div className="absolute top-1/2 -right-6 -translate-y-1/2 text-xs text-gray-500">Der</div>
            </div>
            <div className="mt-2 text-center">
              <div className="text-xs text-gray-500">Yaw</div>
              <div className="text-sm font-mono">{yaw.toFixed(2)}</div>
            </div>
          </div>
        </div>
      </div>

      {/* Valores de control */}
      <div className="grid grid-cols-4 gap-4 text-center text-sm">
        <div className="bg-gray-100 p-2 rounded">
          <div className="text-gray-600">Roll</div>
          <div className="font-mono">{mainJoystick.x.toFixed(2)}</div>
        </div>
        <div className="bg-gray-100 p-2 rounded">
          <div className="text-gray-600">Pitch</div>
          <div className="font-mono">{mainJoystick.y.toFixed(2)}</div>
        </div>
        <div className="bg-gray-100 p-2 rounded">
          <div className="text-gray-600">Throttle</div>
          <div className="font-mono">{throttle.toFixed(2)}</div>
        </div>
        <div className="bg-gray-100 p-2 rounded">
          <div className="text-gray-600">Yaw</div>
          <div className="font-mono">{yaw.toFixed(2)}</div>
        </div>
      </div>
    </div>
  )
}
