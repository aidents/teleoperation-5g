#!/usr/bin/env python3
"""
Bridge ROS2-Web para la interfaz Next.js
Conecta el sistema ROS2 con la interfaz web para comunicación en tiempo real
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, Deadline
from std_msgs.msg import Bool, String, Float32
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import BatteryState
import json
import asyncio
import websockets
import threading
import time
import logging
from typing import Dict, Any, Set
from . import config

class ROS2WebBridge(Node):
    """
    Nodo puente entre ROS2 y la interfaz web Next.js
    """
    
    def __init__(self):
        super().__init__('ros2_web_bridge')
        
        # Configurar logging
        self.setup_logging()
        
        # Configuración QoS para baja latencia
        self.qos_control = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=config.QOS_CONFIG['control']['depth'],
            deadline=Deadline(seconds=config.QOS_CONFIG['control']['deadline'])
        )
        
        self.qos_telemetry = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=config.QOS_CONFIG['telemetry']['depth'],
            deadline=Deadline(seconds=config.QOS_CONFIG['telemetry']['deadline'])
        )
        
        self.qos_emergency = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=config.QOS_CONFIG['emergency']['depth'],
            deadline=Deadline(seconds=config.QOS_CONFIG['emergency']['deadline'])
        )
        
        # Estado del sistema
        self.drone_armed = False
        self.drone_pose = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'yaw': 0.0}
        self.drone_velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.battery_level = 100.0
        self.network_latency = 0.0
        self.connected_clients: Set[websockets.WebSocketServerProtocol] = set()
        self.last_control_time = 0.0
        self.control_count = 0
        
        # Publishers para comandos de control
        self.arm_pub = self.create_publisher(Bool, config.TOPIC_CONFIG['drone_arm'], self.qos_control)
        self.control_pub = self.create_publisher(Twist, config.TOPIC_CONFIG['drone_control'], self.qos_control)
        self.emergency_pub = self.create_publisher(Bool, config.TOPIC_CONFIG['drone_emergency'], self.qos_emergency)
        
        # Subscribers para telemetría
        self.arm_sub = self.create_subscription(
            Bool, config.TOPIC_CONFIG['drone_arm_status'], self.arm_status_callback, self.qos_telemetry
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, config.TOPIC_CONFIG['drone_pose'], self.pose_callback, self.qos_telemetry
        )
        self.velocity_sub = self.create_subscription(
            Twist, config.TOPIC_CONFIG['drone_velocity'], self.velocity_callback, self.qos_telemetry
        )
        self.battery_sub = self.create_subscription(
            BatteryState, config.TOPIC_CONFIG['drone_battery'], self.battery_callback, self.qos_telemetry
        )
        
        # Timer para actualizaciones periódicas
        self.update_timer = self.create_timer(
            1.0 / config.MESSAGE_CONFIG['max_telemetry_rate'], 
            self.broadcast_updates
        )
        
        # Timer para métricas de rendimiento
        self.metrics_timer = self.create_timer(
            config.MONITORING_CONFIG['metrics_interval'],
            self.update_metrics
        )
        
        # Iniciar servidor WebSocket en hilo separado
        self.websocket_thread = threading.Thread(target=self.run_websocket_server)
        self.websocket_thread.daemon = True
        self.websocket_thread.start()
        
        self.get_logger().info(f'🚁 Bridge ROS2-Web iniciado en {config.WEBSOCKET_CONFIG["host"]}:{config.WEBSOCKET_CONFIG["port"]}')
    
    def setup_logging(self):
        """Configura el sistema de logging"""
        logging.basicConfig(
            level=getattr(logging, config.LOGGING_CONFIG['level']),
            format=config.LOGGING_CONFIG['format'],
            handlers=[
                logging.FileHandler(config.LOGGING_CONFIG['file']),
                logging.StreamHandler()
            ]
        )
    
    def arm_status_callback(self, msg: Bool):
        """Callback para cambios en el estado de armado del dron"""
        self.drone_armed = msg.data
        self.get_logger().info(f'🔄 Estado de armado: {"ARMADO" if self.drone_armed else "DESARMADO"}')
    
    def pose_callback(self, msg: PoseStamped):
        """Callback para actualizaciones de posición del dron"""
        self.drone_pose = {
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z,
            'yaw': self.quaternion_to_yaw(msg.pose.orientation)
        }
    
    def velocity_callback(self, msg: Twist):
        """Callback para actualizaciones de velocidad del dron"""
        self.drone_velocity = {
            'x': msg.linear.x,
            'y': msg.linear.y,
            'z': msg.linear.z
        }
    
    def battery_callback(self, msg: BatteryState):
        """Callback para actualizaciones de batería"""
        self.battery_level = msg.percentage * 100.0
    
    def quaternion_to_yaw(self, orientation) -> float:
        """Convierte quaternion a ángulo yaw"""
        import math
        # Implementación simple - en producción usar transformaciones completas
        return math.atan2(2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                          1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))
    
    async def handle_websocket_connection(self, websocket, path):
        """Maneja conexiones WebSocket individuales"""
        # Verificar límite de conexiones
        if len(self.connected_clients) >= config.WEBSOCKET_CONFIG['max_connections']:
            await websocket.close(1013, "Límite de conexiones alcanzado")
            return
        
        # Verificar origen si la autenticación está habilitada
        if config.SECURITY_CONFIG['enable_authentication']:
            origin = websocket.request_headers.get('Origin')
            if origin not in config.SECURITY_CONFIG['allowed_origins']:
                await websocket.close(1008, "Origen no permitido")
                return
        
        self.connected_clients.add(websocket)
        self.get_logger().info(f'🌐 Cliente WebSocket conectado. Total: {len(self.connected_clients)}')
        
        try:
            async for message in websocket:
                await self.handle_web_message(websocket, message)
        except websockets.exceptions.ConnectionClosed:
            pass
        except Exception as e:
            self.get_logger().error(f'❌ Error en conexión WebSocket: {e}')
        finally:
            self.connected_clients.remove(websocket)
            self.get_logger().info(f'🌐 Cliente WebSocket desconectado. Total: {len(self.connected_clients)}')
    
    async def handle_web_message(self, websocket, message: str):
        """Procesa mensajes recibidos de la interfaz web"""
        try:
            # Verificar tamaño del mensaje
            if len(message) > config.SECURITY_CONFIG['max_message_size']:
                self.get_logger().warn('⚠️ Mensaje demasiado grande recibido')
                return
            
            data = json.loads(message)
            msg_type = data.get('type')
            
            if msg_type == 'arm_drone':
                # Comando para armar/desarmar dron
                arm_cmd = Bool(data=data.get('armed', False))
                self.arm_pub.publish(arm_cmd)
                self.get_logger().info(f'🎮 Comando de armado: {"ARMAR" if arm_cmd.data else "DESARMAR"}')
                
            elif msg_type == 'control':
                # Comando de control del dron con rate limiting
                current_time = time.time()
                if current_time - self.last_control_time < 1.0 / config.MESSAGE_CONFIG['max_control_rate']:
                    self.control_count += 1
                    if self.control_count > 10:  # Permitir algunos comandos extra
                        return
                else:
                    self.control_count = 0
                    self.last_control_time = current_time
                
                control_data = data.get('control', {})
                twist_msg = Twist()
                twist_msg.linear.x = control_data.get('x', 0.0)
                twist_msg.linear.y = control_data.get('y', 0.0)
                twist_msg.linear.z = control_data.get('z', 0.0)
                twist_msg.angular.z = control_data.get('yaw', 0.0)
                
                self.control_pub.publish(twist_msg)
                
            elif msg_type == 'emergency':
                # Comando de emergencia
                emergency_cmd = Bool(data=True)
                self.emergency_pub.publish(emergency_cmd)
                self.get_logger().warn('🚨 COMANDO DE EMERGENCIA RECIBIDO')
                
            elif msg_type == 'ping':
                # Ping para medir latencia
                start_time = time.time()
                response = {
                    'type': 'pong',
                    'timestamp': start_time,
                    'latency': 0.0
                }
                await websocket.send(json.dumps(response))
                
        except json.JSONDecodeError:
            self.get_logger().warn('⚠️ Mensaje JSON inválido recibido')
        except Exception as e:
            self.get_logger().error(f'❌ Error procesando mensaje: {e}')
    
    def broadcast_updates(self):
        """Envía actualizaciones a todos los clientes conectados"""
        if not self.connected_clients:
            return
        
        # Preparar datos de telemetría
        telemetry_data = {
            'type': 'telemetry',
            'timestamp': time.time(),
            'drone': {
                'armed': self.drone_armed,
                'pose': self.drone_pose,
                'velocity': self.drone_velocity,
                'battery': self.battery_level
            },
            'network': {
                'latency': self.network_latency,
                'connected_clients': len(self.connected_clients)
            }
        }
        
        # Enviar a todos los clientes en hilo separado
        asyncio.run_coroutine_threadsafe(
            self.broadcast_to_clients(telemetry_data),
            self.websocket_loop
        )
    
    async def broadcast_to_clients(self, data: Dict[str, Any]):
        """Envía datos a todos los clientes WebSocket conectados"""
        message = json.dumps(data)
        disconnected_clients = set()
        
        for client in self.connected_clients:
            try:
                await client.send(message)
            except websockets.exceptions.ConnectionClosed:
                disconnected_clients.add(client)
            except Exception as e:
                self.get_logger().error(f'❌ Error enviando a cliente: {e}')
                disconnected_clients.add(client)
        
        # Limpiar clientes desconectados
        self.connected_clients -= disconnected_clients
    
    def update_metrics(self):
        """Actualiza métricas de rendimiento"""
        if not config.MONITORING_CONFIG['enable_metrics']:
            return
        
        # Aquí podrías agregar métricas de CPU, memoria, etc.
        self.get_logger().debug(f'📊 Métricas: {len(self.connected_clients)} clientes, {self.control_count} comandos/seg')
    
    def run_websocket_server(self):
        """Ejecuta el servidor WebSocket en el hilo principal del event loop"""
        self.websocket_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.websocket_loop)
        
        start_server = websockets.serve(
            self.handle_websocket_connection,
            config.WEBSOCKET_CONFIG['host'],
            config.WEBSOCKET_CONFIG['port']
        )
        
        self.websocket_loop.run_until_complete(start_server)
        self.get_logger().info(f'🌐 Servidor WebSocket iniciado en {config.WEBSOCKET_CONFIG["host"]}:{config.WEBSOCKET_CONFIG["port"]}')
        self.websocket_loop.run_forever()

def main(args=None):
    rclpy.init(args=args)
    bridge = ROS2WebBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        bridge.get_logger().info('🛑 Bridge ROS2-Web detenido por el usuario')
    except Exception as e:
        bridge.get_logger().error(f'❌ Error en bridge ROS2-Web: {e}')
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
