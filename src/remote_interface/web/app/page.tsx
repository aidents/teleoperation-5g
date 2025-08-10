"use client"

import { useEffect, useMemo, useState } from "react"
import { Activity, AlertTriangle, Gamepad2, Gauge, Power, Radio, Rocket, Wifi } from "lucide-react"
import { Button } from "@/components/ui/button"
import { Card } from "@/components/ui/card"
import { Slider } from "@/components/ui/slider"
import { Separator } from "@/components/ui/separator"
import { cn } from "@/lib/utils"

import GlassCard from "@/components/glass-card"
import Joystick from "@/components/joystick"
import Sparkline from "@/components/sparkline"
import BatteryBar from "@/components/battery-bar"
import { useROS2WebSocket } from "@/hooks/useROS2WebSocket"

type Vec3 = { x: number; y: number; z: number }

export default function Page() {
  // Usar el hook ROS2 para conexión real
  const {
    isConnected,
    connectionStatus,
    droneState,
    networkState,
    sendControl,
    sendArmCommand,
    sendPing,
    reconnect,
    disconnect,
    messageCount,
    lastMessageTime
  } = useROS2WebSocket('ws://localhost:8080')

  // Estado local para controles
  const [joystick, setJoystick] = useState<{ x: number; y: number }>({ x: 0, y: 0 })
  const [altitude, setAltitude] = useState(0)
  const [yaw, setYaw] = useState(0)

  // Estado del dron desde ROS2
  const [armed, setArmed] = useState(false)
  const [mode, setMode] = useState<"STANDBY" | "AUTO" | "MANUAL">("STANDBY")
  const [pos, setPos] = useState<Vec3>({ x: 0, y: 0, z: 0 })
  const [vel, setVel] = useState<Vec3>({ x: 0, y: 0, z: 0 })
  const [battery, setBattery] = useState(100)
  const [latencies, setLatencies] = useState<number[]>(Array.from({ length: 40 }, () => 0))

  // Actualizar estado local cuando cambie el estado del dron
  useEffect(() => {
    if (droneState) {
      setArmed(droneState.armed)
      setPos(droneState.pose)
      setVel(droneState.velocity)
      setBattery(droneState.battery)

      // Actualizar modo basado en el estado
      if (droneState.armed) {
        setMode("MANUAL")
      } else {
        setMode("STANDBY")
      }
    }
  }, [droneState])

  // Actualizar latencias cuando cambie el estado de red
  useEffect(() => {
    if (networkState.latency > 0) {
      setLatencies(prev => {
        const newLatencies = [...prev.slice(1), networkState.latency]
        return newLatencies
      })
    }
  }, [networkState.latency])

  // Enviar comandos de control cuando cambien los controles
  useEffect(() => {
    if (isConnected && armed && mode === "MANUAL") {
      // Enviar comando de control cada 100ms para suavizar el movimiento
      const controlInterval = setInterval(() => {
        sendControl(joystick.x * 2, joystick.y * -2, (altitude - pos.z) * 0.2, yaw)
      }, 100)

      return () => clearInterval(controlInterval)
    }
  }, [isConnected, armed, mode, joystick, altitude, yaw, pos.z, sendControl])

  // Calcular estadísticas de latencia
  const avgLatency = useMemo(() => {
    const validLatencies = latencies.filter(l => l > 0)
    if (validLatencies.length === 0) return 0
    return Math.round(validLatencies.reduce((a, b) => a + b, 0) / validLatencies.length)
  }, [latencies])

  const minLatency = useMemo(() => {
    const validLatencies = latencies.filter(l => l > 0)
    if (validLatencies.length === 0) return 0
    return Math.round(Math.min(...validLatencies))
  }, [latencies])

  const maxLatency = useMemo(() => {
    const validLatencies = latencies.filter(l => l > 0)
    if (validLatencies.length === 0) return 0
    return Math.round(Math.max(...validLatencies))
  }, [latencies])

  // Funciones de control
  function handleConnectToggle() {
    if (isConnected) {
      disconnect()
    } else {
      reconnect()
    }
    setMode("STANDBY")
  }

  function handleArm() {
    sendArmCommand(true)
    setMode("MANUAL")
  }

  function handleDisarm() {
    sendArmCommand(false)
    setMode("STANDBY")
  }

  function handleTakeoff() {
    setAltitude(5)
    setMode("AUTO")
    // Enviar comando de despegue
    sendControl(0, 0, 1, 0)
  }

  function handleLand() {
    setAltitude(0)
    setMode("AUTO")
    // Enviar comando de aterrizaje
    sendControl(0, 0, -1, 0)
  }

  function handleEmergency() {
    // Parada de emergencia
    sendArmCommand(false)
    setMode("STANDBY")
    setJoystick({ x: 0, y: 0 })
    setAltitude(0)
    setYaw(0)
  }

  // Estado de conexión para mostrar
  const connected = isConnected && connectionStatus === 'connected'

  return (
    <main className={cn("relative min-h-svh w-full bg-gradient-to-b from-black via-neutral-900 to-neutral-800")}>
      <div
        aria-hidden="true"
        className="pointer-events-none absolute inset-0 bg-[radial-gradient(1000px_600px_at_50%_-120px,rgba(255,255,255,0.10),transparent_60%)]"
      />
      <div className="min-h-svh w-full">
        <div className="mx-auto max-w-7xl px-4 py-6 sm:px-6 lg:px-8">
          {/* Top Bar */}
          <header className="mb-6">
            <GlassCard className="px-4 py-3">
              <div className="flex items-center gap-3">
                <div className="size-9 rounded-2xl bg-white/70 grid place-items-center shadow-inner border border-neutral-200">
                  <Radio className="size-5 text-neutral-800" />
                </div>
                <h1 className="text-xl sm:text-2xl font-semibold tracking-tight text-neutral-900">
                  {"Telemetría Dron 5G"}
                </h1>
                <div className="ml-auto flex items-center gap-2">
                  <span
                    className={cn(
                      "inline-flex items-center gap-2 rounded-full px-3 py-1 text-sm font-medium",
                      "border border-neutral-200 bg-white/70 text-neutral-800",
                    )}
                  >
                    <span
                      className={cn(
                        "inline-block size-2 rounded-full shadow",
                        connected ? "bg-emerald-500" : "bg-rose-500",
                      )}
                    />
                    {connected ? "Conectado" : connectionStatus === 'connecting' ? "Conectando..." : "Desconectado"}
                  </span>
                  <Button
                    size="sm"
                    onClick={handleConnectToggle}
                    className={cn(
                      "rounded-full border border-neutral-300 bg-white text-neutral-900 hover:bg-neutral-100",
                    )}
                    variant="ghost"
                  >
                    <Wifi className="mr-2 size-4" />
                    {connected ? "Desconectar" : "Conectar"}
                  </Button>
                </div>
              </div>
            </GlassCard>
          </header>

          {/* Content */}
          <div className="grid grid-cols-1 gap-6 lg:grid-cols-2">
            {/* Controls */}
            <GlassCard title="Controles" icon={<Gamepad2 className="size-5 text-neutral-700" />} className="p-0">
              <div className="grid gap-6 p-4 sm:p-6">
                <div className="grid grid-cols-1 items-center gap-6 sm:grid-cols-2">
                  <div className="flex flex-col items-center">
                    <div className="text-neutral-500 text-sm mb-2">{"Adelante / Atrás / Izquierda / Derecha"}</div>
                    <Joystick
                      size={220}
                      onChange={(v) => setJoystick(v)}
                      springBack
                      theme="light"
                      disabled={!connected || !armed}
                    />
                  </div>
                  <div className="grid gap-6">
                    <div>
                      <div className="mb-2 flex items-center justify-between text-neutral-600 text-sm">
                        <span>Altura</span>
                        <span className="tabular-nums text-neutral-900">{altitude.toFixed(1)} m</span>
                      </div>
                      <Slider
                        value={[altitude]}
                        min={0}
                        max={20}
                        step={0.1}
                        onValueChange={(v) => setAltitude(v[0] ?? 0)}
                        className="cursor-pointer"
                        disabled={!connected || !armed}
                      />
                    </div>
                    <div>
                      <div className="mb-2 flex items-center justify-between text-neutral-600 text-sm">
                        <span>Rotación (Yaw)</span>
                        <span className="tabular-nums text-neutral-900">{yaw.toFixed(1)} °</span>
                      </div>
                      <Slider
                        value={[yaw]}
                        min={-180}
                        max={180}
                        step={1}
                        onValueChange={(v) => setYaw(v[0] ?? 0)}
                        className="cursor-pointer"
                        disabled={!connected || !armed}
                      />
                    </div>
                  </div>
                </div>

                <div className="grid grid-cols-2 sm:grid-cols-4 gap-3">
                  <Button
                    onClick={handleArm}
                    disabled={!connected || armed}
                    className="rounded-2xl bg-neutral-900 text-white hover:bg-neutral-800"
                  >
                    <Power className="mr-2 size-4" />
                    Armar
                  </Button>
                  <Button
                    onClick={handleDisarm}
                    disabled={!connected || !armed}
                    className="rounded-2xl border border-neutral-300 bg-white text-neutral-900 hover:bg-neutral-100"
                    variant="ghost"
                  >
                    <Power className="mr-2 size-4" />
                    Desarmar
                  </Button>
                  <Button
                    onClick={handleTakeoff}
                    disabled={!connected || !armed}
                    className="rounded-2xl bg-neutral-900 text-white hover:bg-neutral-800"
                  >
                    <Rocket className="mr-2 size-4" />
                    Despegar
                  </Button>
                  <Button
                    onClick={handleLand}
                    disabled={!connected || !armed}
                    className="rounded-2xl border border-neutral-300 bg-white text-neutral-900 hover:bg-neutral-100"
                  >
                    <Gauge className="mr-2 size-4" />
                    Aterrizar
                  </Button>
                </div>

                <Button
                  variant="destructive"
                  className="rounded-2xl bg-red-600 hover:bg-red-500 text-white"
                  onClick={handleEmergency}
                  disabled={!connected}
                >
                  <AlertTriangle className="mr-2 size-4" />
                  Emergencia
                </Button>
              </div>
            </GlassCard>

            {/* Telemetry */}
            <div className="grid gap-6">
              <GlassCard title="Telemetría" icon={<Activity className="size-5 text-neutral-700" />} className="p-0">
                <div className="grid grid-cols-1 gap-6 p-4 sm:grid-cols-2 sm:p-6">
                  {/* Posición */}
                  <Card className="rounded-2xl border border-neutral-200 bg-white shadow-sm">
                    <div className="p-4">
                      <div className="mb-3 text-sm font-medium text-neutral-600">Posición (m)</div>
                      <div className="grid grid-cols-3 gap-4 text-center">
                        <Metric label="X" value={pos.x.toFixed(2)} />
                        <Metric label="Y" value={pos.y.toFixed(2)} />
                        <Metric label="Z" value={pos.z.toFixed(2)} />
                      </div>
                    </div>
                  </Card>

                  {/* Velocidad */}
                  <Card className="rounded-2xl border border-neutral-200 bg-white shadow-sm">
                    <div className="p-4">
                      <div className="mb-3 text-sm font-medium text-neutral-600">Velocidad (m/s)</div>
                      <div className="grid grid-cols-3 gap-4 text-center">
                        <Metric label="VX" value={vel.x.toFixed(2)} />
                        <Metric label="VY" value={vel.y.toFixed(2)} />
                        <Metric label="VZ" value={vel.z.toFixed(2)} />
                      </div>
                    </div>
                  </Card>

                  {/* Batería */}
                  <Card className="rounded-2xl border border-neutral-200 bg-white shadow-sm sm:col-span-2">
                    <div className="p-4">
                      <div className="mb-3 flex items-center justify-between">
                        <div className="text-sm font-medium text-neutral-600">Batería</div>
                        <div className="text-sm tabular-nums text-neutral-900">{battery.toFixed(0)}%</div>
                      </div>
                      <BatteryBar percent={battery} />
                    </div>
                  </Card>

                  {/* Estado */}
                  <Card className="rounded-2xl border border-neutral-200 bg-white shadow-sm sm:col-span-2">
                    <div className="p-4">
                      <div className="mb-3 text-sm font-medium text-neutral-600">Estado</div>
                      <div className="grid grid-cols-2 gap-4 text-sm">
                        <div className="flex items-center justify-between">
                          <span className="text-neutral-600">Armado</span>
                          <span className={cn("font-semibold", armed ? "text-neutral-900" : "text-neutral-500")}>
                            {armed ? "Sí" : "No"}
                          </span>
                        </div>
                        <div className="flex items-center justify-between">
                          <span className="text-neutral-600">Modo</span>
                          <span className="font-semibold text-neutral-900">{mode}</span>
                        </div>
                        <div className="flex items-center justify-between">
                          <span className="text-neutral-600">Mensajes</span>
                          <span className="font-semibold text-neutral-900">{messageCount}</span>
                        </div>
                        <div className="flex items-center justify-between">
                          <span className="text-neutral-600">Última actualización</span>
                          <span className="font-semibold text-neutral-900">
                            {lastMessageTime ? new Date(lastMessageTime).toLocaleTimeString() : 'Nunca'}
                          </span>
                        </div>
                      </div>
                    </div>
                  </Card>
                </div>
              </GlassCard>

              {/* Latencia */}
              <GlassCard className="p-0" title="Latencia de Red" icon={<Wifi className="size-5 text-neutral-700" />}>
                <div className="p-4 sm:p-6">
                  <Sparkline values={latencies} height={100} width={800} tone="neutral" />
                  <Separator className="my-4 bg-neutral-200" />
                  <div className="grid grid-cols-3 text-sm text-neutral-600">
                    <div>
                      Promedio: <span className="font-semibold text-neutral-900">{avgLatency} ms</span>
                    </div>
                    <div className="text-center">
                      Mínima: <span className="font-semibold text-neutral-900">{minLatency} ms</span>
                    </div>
                    <div className="text-right">
                      Máxima: <span className="font-semibold text-neutral-900">{maxLatency} ms</span>
                    </div>
                  </div>
                  <div className="mt-2 text-xs text-neutral-500 text-center">
                    Clientes conectados: {networkState.connected_clients}
                  </div>
                </div>
              </GlassCard>
            </div>
          </div>

          {/* Footer */}
          <footer className="mt-6">
            <GlassCard className="px-4 py-3 text-center">
              <p className="text-xs sm:text-sm text-neutral-600">
                {"© 2025 Panel de Telemetría. Conectado a ROS2 en tiempo real."}
              </p>
            </GlassCard>
          </footer>
        </div>
      </div>
    </main>
  )
}

function Metric({ label, value }: { label: string; value: string }) {
  return (
    <div className="rounded-xl border border-neutral-200 bg-neutral-50 py-3">
      <div className="text-xs text-neutral-600">{label}</div>
      <div className="text-lg font-semibold tabular-nums text-neutral-900">{value}</div>
    </div>
  )
}
