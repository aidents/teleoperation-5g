import { useState, useEffect, useCallback, useRef } from 'react';

// Tipos para la comunicación con ROS2
export interface DronePose {
  x: number;
  y: number;
  z: number;
  yaw: number;
}

export interface DroneVelocity {
  x: number;
  y: number;
  z: number;
}

export interface DroneState {
  armed: boolean;
  pose: DronePose;
  velocity: DroneVelocity;
  battery: number;
}

export interface NetworkState {
  latency: number;
  connected_clients: number;
}

export interface TelemetryData {
  type: 'telemetry';
  timestamp: number;
  drone: DroneState;
  network: NetworkState;
}

export interface ControlCommand {
  type: 'control';
  control: {
    x: number;
    y: number;
    z: number;
    yaw: number;
  };
}

export interface ArmCommand {
  type: 'arm_drone';
  armed: boolean;
}

export interface PingCommand {
  type: 'ping';
}

export type WebSocketMessage = TelemetryData | ControlCommand | ArmCommand | PingCommand;

interface UseROS2WebSocketReturn {
  // Estado de conexión
  isConnected: boolean;
  connectionStatus: 'connecting' | 'connected' | 'disconnected' | 'error';
  
  // Estado del dron
  droneState: DroneState;
  networkState: NetworkState;
  
  // Funciones de control
  sendControl: (x: number, y: number, z: number, yaw: number) => void;
  sendArmCommand: (armed: boolean) => void;
  sendPing: () => void;
  
  // Estado de la conexión
  reconnect: () => void;
  disconnect: () => void;
  
  // Estadísticas
  messageCount: number;
  lastMessageTime: number | null;
}

const DEFAULT_DRONE_STATE: DroneState = {
  armed: false,
  pose: { x: 0, y: 0, z: 0, yaw: 0 },
  velocity: { x: 0, y: 0, z: 0 },
  battery: 100,
};

const DEFAULT_NETWORK_STATE: NetworkState = {
  latency: 0,
  connected_clients: 0,
};

export const useROS2WebSocket = (url: string = 'ws://localhost:8080'): UseROS2WebSocketReturn => {
  const [isConnected, setIsConnected] = useState(false);
  const [connectionStatus, setConnectionStatus] = useState<'connecting' | 'connected' | 'disconnected' | 'error'>('disconnected');
  const [droneState, setDroneState] = useState<DroneState>(DEFAULT_DRONE_STATE);
  const [networkState, setNetworkState] = useState<NetworkState>(DEFAULT_NETWORK_STATE);
  const [messageCount, setMessageCount] = useState(0);
  const [lastMessageTime, setLastMessageTime] = useState<number | null>(null);
  
  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<NodeJS.Timeout | null>(null);
  const pingIntervalRef = useRef<NodeJS.Timeout | null>(null);

  // Función para conectar al WebSocket
  const connect = useCallback(() => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      return;
    }

    setConnectionStatus('connecting');
    
    try {
      const ws = new WebSocket(url);
      wsRef.current = ws;

      ws.onopen = () => {
        console.log('🌐 Conectado al bridge ROS2-Web');
        setIsConnected(true);
        setConnectionStatus('connected');
        
        // Iniciar ping periódico para mantener la conexión
        pingIntervalRef.current = setInterval(() => {
          if (ws.readyState === WebSocket.OPEN) {
            sendPing();
          }
        }, 5000); // Ping cada 5 segundos
      };

      ws.onmessage = (event) => {
        try {
          const data: WebSocketMessage = JSON.parse(event.data);
          
          if (data.type === 'telemetry') {
            setDroneState(data.drone);
            setNetworkState(data.network);
            setMessageCount(prev => prev + 1);
            setLastMessageTime(Date.now());
          } else if (data.type === 'pong') {
            // Calcular latencia si es necesario
            const latency = Date.now() - data.timestamp;
            setNetworkState(prev => ({ ...prev, latency }));
          }
        } catch (error) {
          console.error('❌ Error parsing WebSocket message:', error);
        }
      };

      ws.onclose = (event) => {
        console.log('🌐 WebSocket cerrado:', event.code, event.reason);
        setIsConnected(false);
        setConnectionStatus('disconnected');
        
        // Limpiar intervalos
        if (pingIntervalRef.current) {
          clearInterval(pingIntervalRef.current);
          pingIntervalRef.current = null;
        }
        
        // Reintentar conexión después de un delay
        if (reconnectTimeoutRef.current) {
          clearTimeout(reconnectTimeoutRef.current);
        }
        reconnectTimeoutRef.current = setTimeout(() => {
          if (connectionStatus !== 'connected') {
            connect();
          }
        }, 3000);
      };

      ws.onerror = (error) => {
        console.error('❌ Error en WebSocket:', error);
        setConnectionStatus('error');
      };

    } catch (error) {
      console.error('❌ Error creando WebSocket:', error);
      setConnectionStatus('error');
    }
  }, [url, connectionStatus]);

  // Función para desconectar
  const disconnect = useCallback(() => {
    if (pingIntervalRef.current) {
      clearInterval(pingIntervalRef.current);
      pingIntervalRef.current = null;
    }
    
    if (reconnectTimeoutRef.current) {
      clearTimeout(reconnectTimeoutRef.current);
      reconnectTimeoutRef.current = null;
    }
    
    if (wsRef.current) {
      wsRef.current.close();
      wsRef.current = null;
    }
    
    setIsConnected(false);
    setConnectionStatus('disconnected');
  }, []);

  // Función para reconectar
  const reconnect = useCallback(() => {
    disconnect();
    setTimeout(connect, 1000);
  }, [connect, disconnect]);

  // Función para enviar comando de control
  const sendControl = useCallback((x: number, y: number, z: number, yaw: number) => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      const command: ControlCommand = {
        type: 'control',
        control: { x, y, z, yaw }
      };
      wsRef.current.send(JSON.stringify(command));
    }
  }, []);

  // Función para enviar comando de armado
  const sendArmCommand = useCallback((armed: boolean) => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      const command: ArmCommand = {
        type: 'arm_drone',
        armed
      };
      wsRef.current.send(JSON.stringify(command));
    }
  }, []);

  // Función para enviar ping
  const sendPing = useCallback(() => {
    if (wsRef.current?.readyState === WebSocket.OPEN) {
      const command: PingCommand = {
        type: 'ping'
      };
      wsRef.current.send(JSON.stringify(command));
    }
  }, []);

  // Efecto para conectar al montar el componente
  useEffect(() => {
    connect();
    
    return () => {
      disconnect();
    };
  }, [connect, disconnect]);

  // Efecto para limpiar timeouts al desmontar
  useEffect(() => {
    return () => {
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current);
      }
      if (pingIntervalRef.current) {
        clearInterval(pingIntervalRef.current);
      }
    };
  }, []);

  return {
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
    lastMessageTime,
  };
};
