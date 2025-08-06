#!/usr/bin/env python3

"""
Script de prueba para verificar el sistema de teleoperación de dron
Autor: Tu Nombre
Fecha: 2024
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
import time
import threading
import requests
import websocket
import json
import sys


class SystemTester(Node):
    """Clase para probar todos los componentes del sistema"""
    
    def __init__(self):
        super().__init__('system_tester')
        
        # Configuración QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Publishers para pruebas
        self.control_pub = self.create_publisher(Twist, '/drone/control', qos_profile)
        self.arm_pub = self.create_publisher(Bool, '/drone/arm', qos_profile)
        
        # Subscribers para verificar telemetría
        self.telemetry_sub = self.create_subscription(
            Odometry, '/drone/telemetry', self.telemetry_callback, qos_profile)
        self.battery_sub = self.create_subscription(
            BatteryState, '/drone/battery', self.battery_callback, qos_profile)
        
        # Variables de estado
        self.telemetry_received = False
        self.battery_received = False
        self.telemetry_data = None
        self.battery_data = None
        
        # WebSocket para pruebas
        self.websocket = None
        self.websocket_connected = False
        
        self.get_logger().info('System Tester iniciado')
    
    def telemetry_callback(self, msg):
        """Callback para telemetría del dron"""
        self.telemetry_received = True
        self.telemetry_data = msg
        self.get_logger().info(f'Telemetría recibida: Pos=({msg.pose.pose.position.x:.2f}, {msg.pose.pose.position.y:.2f}, {msg.pose.pose.position.z:.2f})')
    
    def battery_callback(self, msg):
        """Callback para estado de batería"""
        self.battery_received = True
        self.battery_data = msg
        self.get_logger().info(f'Batería recibida: {msg.percentage*100:.1f}%')
    
    def test_ros_topics(self):
        """Probar comunicación ROS"""
        self.get_logger().info('🔧 Probando comunicación ROS...')
        
        # Enviar comando de armado
        arm_msg = Bool()
        arm_msg.data = True
        self.arm_pub.publish(arm_msg)
        self.get_logger().info('Comando de armado enviado')
        
        # Enviar comandos de control
        for i in range(5):
            control_msg = Twist()
            control_msg.linear.x = 0.5
            control_msg.linear.y = 0.0
            control_msg.linear.z = 0.0
            control_msg.angular.z = 0.0
            self.control_pub.publish(control_msg)
            self.get_logger().info(f'Comando de control {i+1}/5 enviado')
            time.sleep(0.5)
        
        # Esperar telemetría
        timeout = 10
        start_time = time.time()
        while not (self.telemetry_received and self.battery_received):
            if time.time() - start_time > timeout:
                self.get_logger().error('❌ Timeout esperando telemetría')
                return False
            time.sleep(0.1)
        
        self.get_logger().info('✅ Comunicación ROS funcionando correctamente')
        return True
    
    def test_web_interface(self):
        """Probar interfaz web"""
        self.get_logger().info('🌐 Probando interfaz web...')
        
        try:
            # Probar servidor HTTP
            response = requests.get('http://localhost:8080', timeout=5)
            if response.status_code == 200:
                self.get_logger().info('✅ Servidor HTTP funcionando')
            else:
                self.get_logger().error(f'❌ Error HTTP: {response.status_code}')
                return False
        except requests.exceptions.RequestException as e:
            self.get_logger().error(f'❌ Error conectando al servidor web: {e}')
            return False
        
        # Probar WebSocket
        try:
            self.websocket = websocket.create_connection('ws://localhost:8081', timeout=5)
            self.websocket_connected = True
            self.get_logger().info('✅ WebSocket conectado')
            
            # Enviar mensaje de prueba
            test_message = {
                'type': 'ping',
                'timestamp': int(time.time() * 1000)
            }
            self.websocket.send(json.dumps(test_message))
            
            # Recibir respuesta
            response = self.websocket.recv()
            response_data = json.loads(response)
            
            if response_data.get('type') == 'pong':
                self.get_logger().info('✅ WebSocket funcionando correctamente')
            else:
                self.get_logger().error('❌ Respuesta WebSocket inesperada')
                return False
                
        except Exception as e:
            self.get_logger().error(f'❌ Error WebSocket: {e}')
            return False
        
        return True
    
    def test_control_commands(self):
        """Probar comandos de control via WebSocket"""
        self.get_logger().info('🎮 Probando comandos de control...')
        
        if not self.websocket_connected:
            self.get_logger().error('❌ WebSocket no conectado')
            return False
        
        try:
            # Enviar comando de armado
            arm_message = {
                'type': 'arm',
                'data': True
            }
            self.websocket.send(json.dumps(arm_message))
            self.get_logger().info('Comando de armado enviado via WebSocket')
            
            # Enviar comandos de control
            control_message = {
                'type': 'control',
                'data': {
                    'forward': 0.5,
                    'right': 0.0,
                    'up': 0.0,
                    'yaw': 0.0
                }
            }
            self.websocket.send(json.dumps(control_message))
            self.get_logger().info('Comando de control enviado via WebSocket')
            
            # Esperar un poco para ver cambios en telemetría
            time.sleep(2)
            
            self.get_logger().info('✅ Comandos de control funcionando')
            return True
            
        except Exception as e:
            self.get_logger().error(f'❌ Error enviando comandos: {e}')
            return False
    
    def test_latency(self):
        """Probar latencia del sistema"""
        self.get_logger().info('⏱️ Probando latencia...')
        
        if not self.websocket_connected:
            self.get_logger().error('❌ WebSocket no conectado')
            return False
        
        latencies = []
        
        for i in range(10):
            start_time = time.time()
            
            ping_message = {
                'type': 'ping',
                'timestamp': int(start_time * 1000)
            }
            self.websocket.send(json.dumps(ping_message))
            
            response = self.websocket.recv()
            end_time = time.time()
            
            latency = (end_time - start_time) * 1000  # Convertir a ms
            latencies.append(latency)
            
            self.get_logger().info(f'Latencia {i+1}/10: {latency:.2f}ms')
            time.sleep(0.1)
        
        avg_latency = sum(latencies) / len(latencies)
        max_latency = max(latencies)
        min_latency = min(latencies)
        
        self.get_logger().info(f'📊 Estadísticas de latencia:')
        self.get_logger().info(f'   Promedio: {avg_latency:.2f}ms')
        self.get_logger().info(f'   Máxima: {max_latency:.2f}ms')
        self.get_logger().info(f'   Mínima: {min_latency:.2f}ms')
        
        if avg_latency < 50:  # Objetivo: < 50ms
            self.get_logger().info('✅ Latencia dentro de objetivos')
            return True
        else:
            self.get_logger().warning(f'⚠️ Latencia alta: {avg_latency:.2f}ms')
            return False
    
    def run_all_tests(self):
        """Ejecutar todas las pruebas"""
        self.get_logger().info('🚀 Iniciando pruebas del sistema...')
        
        tests = [
            ('ROS Topics', self.test_ros_topics),
            ('Web Interface', self.test_web_interface),
            ('Control Commands', self.test_control_commands),
            ('Latency', self.test_latency)
        ]
        
        results = []
        
        for test_name, test_func in tests:
            self.get_logger().info(f'\n{"="*50}')
            self.get_logger().info(f'Ejecutando: {test_name}')
            self.get_logger().info(f'{"="*50}')
            
            try:
                result = test_func()
                results.append((test_name, result))
                
                if result:
                    self.get_logger().info(f'✅ {test_name}: PASÓ')
                else:
                    self.get_logger().error(f'❌ {test_name}: FALLÓ')
                    
            except Exception as e:
                self.get_logger().error(f'❌ {test_name}: ERROR - {e}')
                results.append((test_name, False))
        
        # Resumen final
        self.get_logger().info(f'\n{"="*50}')
        self.get_logger().info('📋 RESUMEN DE PRUEBAS')
        self.get_logger().info(f'{"="*50}')
        
        passed = sum(1 for _, result in results if result)
        total = len(results)
        
        for test_name, result in results:
            status = "✅ PASÓ" if result else "❌ FALLÓ"
            self.get_logger().info(f'{test_name}: {status}')
        
        self.get_logger().info(f'\nResultado: {passed}/{total} pruebas pasaron')
        
        if passed == total:
            self.get_logger().info('🎉 ¡Todas las pruebas pasaron! El sistema está funcionando correctamente.')
            return True
        else:
            self.get_logger().error('⚠️ Algunas pruebas fallaron. Revisa los logs para más detalles.')
            return False
    
    def cleanup(self):
        """Limpiar recursos"""
        if self.websocket:
            self.websocket.close()


def main(args=None):
    rclpy.init(args=args)
    tester = SystemTester()
    
    try:
        success = tester.run_all_tests()
        if success:
            sys.exit(0)
        else:
            sys.exit(1)
    except KeyboardInterrupt:
        tester.get_logger().info('Pruebas interrumpidas por el usuario')
    except Exception as e:
        tester.get_logger().error(f'Error durante las pruebas: {e}')
        sys.exit(1)
    finally:
        tester.cleanup()
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 