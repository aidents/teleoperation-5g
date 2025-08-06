#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
import json
import asyncio
import websockets
from http.server import HTTPServer, SimpleHTTPRequestHandler
import threading
import os
from pathlib import Path


class WebServer(Node):
    """
    Servidor web que proporciona la interfaz de control remoto
    y maneja conexiones WebSocket para comunicación en tiempo real.
    """
    
    def __init__(self):
        super().__init__('web_server')
        
        # Configuración QoS para baja latencia
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers para comandos de control
        self.control_pub = self.create_publisher(
            Twist, '/drone/control', qos_profile)
        self.arm_pub = self.create_publisher(
            Bool, '/drone/arm', qos_profile)
        
        # Subscribers para telemetría
        self.telemetry_sub = self.create_subscription(
            Odometry, '/drone/telemetry', self.telemetry_callback, qos_profile)
        self.battery_sub = self.create_subscription(
            BatteryState, '/drone/battery', self.battery_callback, qos_profile)
        
        # Estado del dron
        self.drone_telemetry = {
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'orientation': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0},
            'velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'battery': 100.0,
            'armed': False
        }
        
        # Conexiones WebSocket activas
        self.websocket_connections = set()
        
        # Configuración del servidor
        self.web_port = 8080
        self.websocket_port = 8081
        
        # Iniciar servidores en hilos separados
        self.start_web_server()
        self.start_websocket_server()
        
        self.get_logger().info(f'Servidor web iniciado en puerto {self.web_port}')
        self.get_logger().info(f'Servidor WebSocket iniciado en puerto {self.websocket_port}')
    
    def start_web_server(self):
        """Iniciar servidor HTTP para servir archivos web"""
        class CustomHTTPRequestHandler(SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                # Cambiar al directorio web del paquete
                web_dir = Path(__file__).parent.parent / 'web'
                os.chdir(web_dir)
                super().__init__(*args, **kwargs)
            
            def end_headers(self):
                self.send_header('Access-Control-Allow-Origin', '*')
                self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
                self.send_header('Access-Control-Allow-Headers', 'Content-Type')
                super().end_headers()
        
        def run_server():
            server = HTTPServer(('', self.web_port), CustomHTTPRequestHandler)
            server.serve_forever()
        
        thread = threading.Thread(target=run_server, daemon=True)
        thread.start()
    
    def start_websocket_server(self):
        """Iniciar servidor WebSocket para comunicación en tiempo real"""
        async def websocket_handler(websocket, path):
            self.websocket_connections.add(websocket)
            self.get_logger().info('Nueva conexión WebSocket establecida')
            
            try:
                # Enviar estado inicial
                await websocket.send(json.dumps({
                    'type': 'telemetry',
                    'data': self.drone_telemetry
                }))
                
                # Manejar mensajes entrantes
                async for message in websocket:
                    try:
                        data = json.loads(message)
                        await self.handle_websocket_message(data)
                    except json.JSONDecodeError:
                        self.get_logger().warn('Mensaje JSON inválido recibido')
                        
            except websockets.exceptions.ConnectionClosed:
                pass
            finally:
                self.websocket_connections.discard(websocket)
                self.get_logger().info('Conexión WebSocket cerrada')
        
        async def run_websocket_server():
            async with websockets.serve(websocket_handler, '', self.websocket_port):
                await asyncio.Future()  # Ejecutar indefinidamente
        
        def run_async_server():
            asyncio.run(run_websocket_server())
        
        thread = threading.Thread(target=run_async_server, daemon=True)
        thread.start()
    
    async def handle_websocket_message(self, data):
        """Manejar mensajes recibidos por WebSocket"""
        msg_type = data.get('type')
        
        if msg_type == 'control':
            # Comando de control del dron
            control_data = data.get('data', {})
            twist_msg = Twist()
            
            # Mapear controles de la interfaz web
            twist_msg.linear.x = control_data.get('forward', 0.0)
            twist_msg.linear.y = control_data.get('right', 0.0)
            twist_msg.linear.z = control_data.get('up', 0.0)
            twist_msg.angular.z = control_data.get('yaw', 0.0)
            
            self.control_pub.publish(twist_msg)
            
        elif msg_type == 'arm':
            # Comando para armar/desarmar
            arm_msg = Bool()
            arm_msg.data = data.get('data', False)
            self.arm_pub.publish(arm_msg)
            
        elif msg_type == 'ping':
            # Responder ping para mantener conexión
            await self.broadcast_message({
                'type': 'pong',
                'timestamp': data.get('timestamp')
            })
    
    def telemetry_callback(self, msg):
        """Callback para telemetría del dron"""
        self.drone_telemetry['position'] = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z
        }
        self.drone_telemetry['orientation'] = {
            'x': msg.pose.pose.orientation.x,
            'y': msg.pose.pose.orientation.y,
            'z': msg.pose.pose.orientation.z,
            'w': msg.pose.pose.orientation.w
        }
        self.drone_telemetry['velocity'] = {
            'x': msg.twist.twist.linear.x,
            'y': msg.twist.twist.linear.y,
            'z': msg.twist.twist.linear.z
        }
        
        # Enviar telemetría a todos los clientes conectados
        asyncio.create_task(self.broadcast_telemetry())
    
    def battery_callback(self, msg):
        """Callback para estado de batería"""
        self.drone_telemetry['battery'] = msg.percentage * 100.0
    
    async def broadcast_telemetry(self):
        """Enviar telemetría a todos los clientes WebSocket"""
        message = {
            'type': 'telemetry',
            'data': self.drone_telemetry
        }
        await self.broadcast_message(message)
    
    async def broadcast_message(self, message):
        """Enviar mensaje a todos los clientes WebSocket conectados"""
        if not self.websocket_connections:
            return
        
        message_str = json.dumps(message)
        disconnected = set()
        
        for websocket in self.websocket_connections:
            try:
                await websocket.send(message_str)
            except websockets.exceptions.ConnectionClosed:
                disconnected.add(websocket)
        
        # Limpiar conexiones desconectadas
        self.websocket_connections -= disconnected


def main(args=None):
    rclpy.init(args=args)
    server = WebServer()
    
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        pass
    finally:
        server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 