"use client"

import React from "react"
import { cn } from "@/lib/utils"
import { useROS2WebSocket } from "@/hooks/useROS2WebSocket"

interface DroneTelemetryProps {
  className?: string
  theme?: "light" | "dark"
}

export default function DroneTelemetry({ className, theme = "light" }: DroneTelemetryProps) {
  const { 
    droneState, 
    networkState, 
    isConnected, 
    connectionStatus,
    messageCount,
    lastMessageTime 
  } = useROS2WebSocket()

  const textTone = theme === "dark" ? "text-neutral-300" : "text-neutral-500"
  const bgColor = theme === "dark" ? "bg-neutral-800" : "bg-white"
  const borderColor = theme === "dark" ? "border-neutral-700" : "border-neutral-200"

  // Función para formatear timestamp
  const formatTimestamp = (timestamp: number | null) => {
    if (!timestamp) return "N/A"
    return new Date(timestamp).toLocaleTimeString()
  }

  // Función para obtener el color del estado de la batería
  const getBatteryColor = (battery: number) => {
    if (battery > 50) return "text-green-600"
    if (battery > 20) return "text-yellow-600"
    return "text-red-600"
  }

  // Función para obtener el color del estado de conexión
  const getConnectionColor = (status: string) => {
    switch (status) {
      case 'connected': return "text-green-600"
      case 'connecting': return "text-yellow-600"
      case 'error': return "text-red-600"
      default: return "text-gray-600"
    }
  }

  return (
    <div className={cn("space-y-4", className)}>
      {/* Estado de conexión */}
      <div className={cn("p-4 rounded-lg border", bgColor, borderColor)}>
        <h3 className="text-lg font-semibold mb-3 text-gray-800 dark:text-gray-200">
          Estado de Conexión
        </h3>
        <div className="grid grid-cols-2 gap-4">
          <div className="flex items-center gap-2">
            <div className={cn(
              "w-3 h-3 rounded-full",
              connectionStatus === 'connected' ? "bg-green-500" :
              connectionStatus === 'connecting' ? "bg-yellow-500" :
              connectionStatus === 'error' ? "bg-red-500" : "bg-gray-500"
            )} />
            <span className={cn("text-sm", getConnectionColor(connectionStatus))}>
              {connectionStatus === 'connected' ? 'Conectado' :
               connectionStatus === 'connecting' ? 'Conectando...' :
               connectionStatus === 'error' ? 'Error' : 'Desconectado'}
            </span>
          </div>
          <div className="text-sm text-gray-600 dark:text-gray-400">
            Clientes: {networkState.connected_clients}
          </div>
        </div>
      </div>

      {/* Estado del dron */}
      <div className={cn("p-4 rounded-lg border", bgColor, borderColor)}>
        <h3 className="text-lg font-semibold mb-3 text-gray-800 dark:text-gray-200">
          Estado del Dron
        </h3>
        <div className="grid grid-cols-2 gap-4">
          <div className="flex items-center gap-2">
            <div className={cn(
              "w-3 h-3 rounded-full",
              droneState.armed ? "bg-green-500" : "bg-red-500"
            )} />
            <span className="text-sm text-gray-600 dark:text-gray-400">
              {droneState.armed ? 'ARMADO' : 'DESARMADO'}
            </span>
          </div>
          <div className="text-sm text-gray-600 dark:text-gray-400">
            Batería: <span className={cn("font-semibold", getBatteryColor(droneState.battery))}>
              {droneState.battery.toFixed(1)}%
            </span>
          </div>
        </div>
      </div>

      {/* Posición y orientación */}
      <div className={cn("p-4 rounded-lg border", bgColor, borderColor)}>
        <h3 className="text-lg font-semibold mb-3 text-gray-800 dark:text-gray-200">
          Posición y Orientación
        </h3>
        <div className="grid grid-cols-2 gap-4">
          <div>
            <div className="text-sm text-gray-600 dark:text-gray-400 mb-2">Posición (X, Y, Z)</div>
            <div className="font-mono text-sm">
              ({droneState.pose.x.toFixed(2)}, {droneState.pose.y.toFixed(2)}, {droneState.pose.z.toFixed(2)})
            </div>
          </div>
          <div>
            <div className="text-sm text-gray-600 dark:text-gray-400 mb-2">Yaw</div>
            <div className="font-mono text-sm">
              {(droneState.pose.yaw * 180 / Math.PI).toFixed(1)}°
            </div>
          </div>
        </div>
      </div>

      {/* Velocidad */}
      <div className={cn("p-4 rounded-lg border", bgColor, borderColor)}>
        <h3 className="text-lg font-semibold mb-3 text-gray-800 dark:text-gray-200">
          Velocidad
        </h3>
        <div className="grid grid-cols-3 gap-4">
          <div>
            <div className="text-sm text-gray-600 dark:text-gray-400 mb-1">X (Roll)</div>
            <div className="font-mono text-sm">
              {droneState.velocity.x.toFixed(2)} m/s
            </div>
          </div>
          <div>
            <div className="text-sm text-gray-600 dark:text-gray-400 mb-1">Y (Pitch)</div>
            <div className="font-mono text-sm">
              {droneState.velocity.y.toFixed(2)} m/s
            </div>
          </div>
          <div>
            <div className="text-sm text-gray-600 dark:text-gray-400 mb-1">Z (Throttle)</div>
            <div className="font-mono text-sm">
              {droneState.velocity.z.toFixed(2)} m/s
            </div>
          </div>
        </div>
      </div>

      {/* Red y estadísticas */}
      <div className={cn("p-4 rounded-lg border", bgColor, borderColor)}>
        <h3 className="text-lg font-semibold mb-3 text-gray-800 dark:text-gray-200">
          Red y Estadísticas
        </h3>
        <div className="grid grid-cols-2 gap-4">
          <div>
            <div className="text-sm text-gray-600 dark:text-gray-400 mb-1">Latencia</div>
            <div className="font-mono text-sm">
              {networkState.latency > 0 ? `${networkState.latency.toFixed(1)} ms` : 'N/A'}
            </div>
          </div>
          <div>
            <div className="text-sm text-gray-600 dark:text-gray-400 mb-1">Mensajes</div>
            <div className="font-mono text-sm">
              {messageCount}
            </div>
          </div>
          <div>
            <div className="text-sm text-gray-600 dark:text-gray-400 mb-1">Último mensaje</div>
            <div className="font-mono text-sm">
              {formatTimestamp(lastMessageTime)}
            </div>
          </div>
          <div>
            <div className="text-sm text-gray-600 dark:text-gray-400 mb-1">Estado</div>
            <div className="text-sm">
              <span className={cn(
                "px-2 py-1 rounded-full text-xs font-medium",
                isConnected ? "bg-green-100 text-green-800" : "bg-red-100 text-red-800"
              )}>
                {isConnected ? 'Activo' : 'Inactivo'}
              </span>
            </div>
          </div>
        </div>
      </div>

      {/* Gráfico de batería */}
      <div className={cn("p-4 rounded-lg border", bgColor, borderColor)}>
        <h3 className="text-lg font-semibold mb-3 text-gray-800 dark:text-gray-200">
          Nivel de Batería
        </h3>
        <div className="w-full bg-gray-200 rounded-full h-4 dark:bg-gray-700">
          <div 
            className={cn(
              "h-4 rounded-full transition-all duration-300",
              droneState.battery > 50 ? "bg-green-500" :
              droneState.battery > 20 ? "bg-yellow-500" : "bg-red-500"
            )}
            style={{ width: `${droneState.battery}%` }}
          />
        </div>
        <div className="flex justify-between text-sm text-gray-600 dark:text-gray-400 mt-2">
          <span>0%</span>
          <span>{droneState.battery.toFixed(1)}%</span>
          <span>100%</span>
        </div>
      </div>

      {/* Indicadores de estado visual */}
      <div className={cn("p-4 rounded-lg border", bgColor, borderColor)}>
        <h3 className="text-lg font-semibold mb-3 text-gray-800 dark:text-gray-200">
          Indicadores de Estado
        </h3>
        <div className="grid grid-cols-3 gap-4">
          <div className="text-center">
            <div className={cn(
              "w-4 h-4 rounded-full mx-auto mb-2",
              droneState.armed ? "bg-green-500" : "bg-red-500"
            )} />
            <div className="text-xs text-gray-600 dark:text-gray-400">Armado</div>
          </div>
          <div className="text-center">
            <div className={cn(
              "w-4 h-4 rounded-full mx-auto mb-2",
              isConnected ? "bg-green-500" : "bg-red-500"
            )} />
            <div className="text-xs text-gray-600 dark:text-gray-400">Conexión</div>
          </div>
          <div className="text-center">
            <div className={cn(
              "w-4 h-4 rounded-full mx-auto mb-2",
              droneState.battery > 20 ? "bg-green-500" : "bg-red-500"
            )} />
            <div className="text-xs text-gray-600 dark:text-gray-400">Batería</div>
          </div>
        </div>
      </div>
    </div>
  )
}
