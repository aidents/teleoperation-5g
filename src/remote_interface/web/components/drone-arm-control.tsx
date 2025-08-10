"use client"

import React, { useState } from "react"
import { cn } from "@/lib/utils"
import { useROS2WebSocket } from "@/hooks/useROS2WebSocket"

interface DroneArmControlProps {
  className?: string
  theme?: "light" | "dark"
}

export default function DroneArmControl({ className, theme = "light" }: DroneArmControlProps) {
  const { sendArmCommand, droneState, isConnected } = useROS2WebSocket()
  const [showConfirm, setShowConfirm] = useState(false)
  const [confirmAction, setConfirmAction] = useState<'arm' | 'disarm' | null>(null)

  const textTone = theme === "dark" ? "text-neutral-300" : "text-neutral-500"
  const bgColor = theme === "dark" ? "bg-neutral-800" : "bg-white"
  const borderColor = theme === "dark" ? "border-neutral-700" : "border-neutral-200"

  // Función para manejar el armado/desarmado
  const handleArmAction = (action: 'arm' | 'disarm') => {
    if (!isConnected) {
      alert('❌ No hay conexión con ROS2')
      return
    }

    if (action === 'arm' && !droneState.armed) {
      setConfirmAction('arm')
      setShowConfirm(true)
    } else if (action === 'disarm' && droneState.armed) {
      setConfirmAction('disarm')
      setShowConfirm(true)
    }
  }

  // Función para confirmar la acción
  const confirmAction = () => {
    if (confirmAction) {
      sendArmCommand(confirmAction === 'arm')
      setShowConfirm(false)
      setConfirmAction(null)
    }
  }

  // Función para cancelar la confirmación
  const cancelAction = () => {
    setShowConfirm(false)
    setConfirmAction(null)
  }

  // Función para parada de emergencia
  const handleEmergencyStop = () => {
    if (!isConnected) {
      alert('❌ No hay conexión con ROS2')
      return
    }
    
    // Enviar comando de parada de emergencia (todos los controles a 0)
    // Esto se puede implementar como un comando especial en el bridge
    alert('🚨 PARADA DE EMERGENCIA ACTIVADA')
  }

  return (
    <div className={cn("space-y-4", className)}>
      {/* Estado actual del dron */}
      <div className={cn("p-4 rounded-lg border", bgColor, borderColor)}>
        <h3 className="text-lg font-semibold mb-3 text-gray-800 dark:text-gray-200">
          Estado del Dron
        </h3>
        <div className="flex items-center gap-3">
          <div className={cn(
            "w-4 h-4 rounded-full",
            droneState.armed ? "bg-green-500" : "bg-red-500"
          )} />
          <span className="text-lg font-medium">
            {droneState.armed ? 'ARMADO' : 'DESARMADO'}
          </span>
        </div>
        <p className="text-sm text-gray-600 dark:text-gray-400 mt-2">
          {droneState.armed 
            ? 'El dron está listo para volar. Usa los controles para manejarlo.'
            : 'El dron está desarmado y no puede volar.'
          }
        </p>
      </div>

      {/* Controles de armado */}
      <div className={cn("p-4 rounded-lg border", bgColor, borderColor)}>
        <h3 className="text-lg font-semibold mb-3 text-gray-800 dark:text-gray-200">
          Control de Armado
        </h3>
        <div className="grid grid-cols-2 gap-4">
          <button
            onClick={() => handleArmAction('arm')}
            disabled={!isConnected || droneState.armed}
            className={cn(
              "px-6 py-3 rounded-lg font-medium transition-all duration-200",
              "disabled:opacity-50 disabled:cursor-not-allowed",
              droneState.armed 
                ? "bg-gray-100 text-gray-500 cursor-not-allowed"
                : "bg-green-500 hover:bg-green-600 text-white shadow-lg hover:shadow-xl"
            )}
          >
            🚁 ARMAR DRON
          </button>
          
          <button
            onClick={() => handleArmAction('disarm')}
            disabled={!isConnected || !droneState.armed}
            className={cn(
              "px-6 py-3 rounded-lg font-medium transition-all duration-200",
              "disabled:opacity-50 disabled:cursor-not-allowed",
              !droneState.armed 
                ? "bg-gray-100 text-gray-500 cursor-not-allowed"
                : "bg-red-500 hover:bg-red-600 text-white shadow-lg hover:shadow-xl"
            )}
          >
            ⚠️ DESARMAR DRON
          </button>
        </div>
        
        {!isConnected && (
          <p className="text-sm text-red-600 mt-2">
            ⚠️ No hay conexión con ROS2. Los controles están deshabilitados.
          </p>
        )}
      </div>

      {/* Parada de emergencia */}
      <div className={cn("p-4 rounded-lg border-2 border-red-300", bgColor)}>
        <h3 className="text-lg font-semibold mb-3 text-red-700 dark:text-red-400">
          🚨 PARADA DE EMERGENCIA
        </h3>
        <p className="text-sm text-gray-600 dark:text-gray-400 mb-3">
          Usa este botón solo en situaciones de emergencia. Detendrá inmediatamente todos los motores.
        </p>
        <button
          onClick={handleEmergencyStop}
          disabled={!isConnected || !droneState.armed}
          className={cn(
            "w-full px-6 py-4 rounded-lg font-bold text-lg transition-all duration-200",
            "disabled:opacity-50 disabled:cursor-not-allowed",
            "bg-red-600 hover:bg-red-700 text-white shadow-lg hover:shadow-xl",
            "border-2 border-red-500 hover:border-red-600"
          )}
        >
          🚨 PARADA DE EMERGENCIA
        </button>
      </div>

      {/* Información de seguridad */}
      <div className={cn("p-4 rounded-lg border border-yellow-300", bgColor)}>
        <h3 className="text-lg font-semibold mb-3 text-yellow-700 dark:text-yellow-400">
          ⚠️ Información de Seguridad
        </h3>
        <ul className="text-sm text-gray-600 dark:text-gray-400 space-y-2">
          <li>• Asegúrate de que el área esté libre de obstáculos antes de armar</li>
          <li>• Mantén una distancia segura del dron durante el vuelo</li>
          <li>• Ten siempre acceso al botón de parada de emergencia</li>
          <li>• Verifica el nivel de batería antes de cada vuelo</li>
          <li>• No vueles en condiciones meteorológicas adversas</li>
        </ul>
      </div>

      {/* Modal de confirmación */}
      {showConfirm && (
        <div className="fixed inset-0 bg-black bg-opacity-50 flex items-center justify-center z-50">
          <div className={cn("p-6 rounded-lg max-w-md w-full mx-4", bgColor, borderColor)}>
            <h3 className="text-lg font-semibold mb-4 text-gray-800 dark:text-gray-200">
              {confirmAction === 'arm' ? 'Confirmar Armado' : 'Confirmar Desarmado'}
            </h3>
            <p className="text-sm text-gray-600 dark:text-gray-400 mb-6">
              {confirmAction === 'arm' 
                ? '¿Estás seguro de que quieres ARMAR el dron? Esto activará los motores.'
                : '¿Estás seguro de que quieres DESARMAR el dron? Esto detendrá los motores.'
              }
            </p>
            <div className="flex gap-3">
              <button
                onClick={confirmAction}
                className={cn(
                  "flex-1 px-4 py-2 rounded-lg font-medium transition-all duration-200",
                  confirmAction === 'arm'
                    ? "bg-green-500 hover:bg-green-600 text-white"
                    : "bg-red-500 hover:bg-red-600 text-white"
                )}
              >
                {confirmAction === 'arm' ? 'ARMAR' : 'DESARMAR'}
              </button>
              <button
                onClick={cancelAction}
                className="flex-1 px-4 py-2 rounded-lg font-medium bg-gray-300 hover:bg-gray-400 text-gray-700 transition-all duration-200"
              >
                Cancelar
              </button>
            </div>
          </div>
        </div>
      )}
    </div>
  )
}
