#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Header
import time
import random
import threading


class NetworkSimulator(Node):
    def __init__(self):
        super().__init__('network_simulator')
        
        # Parámetros de red 5G
        self.declare_parameter('latency_ms', 5.0)
        self.declare_parameter('jitter_ms', 1.0)
        self.declare_parameter('packet_loss_percent', 0.1)
        self.declare_parameter('bandwidth_mbps', 1000.0)
        
        self.latency_ms = self.get_parameter('latency_ms').value
        self.jitter_ms = self.get_parameter('jitter_ms').value
        self.packet_loss_percent = self.get_parameter('packet_loss_percent').value
        self.bandwidth_mbps = self.get_parameter('bandwidth_mbps').value
        
        # QoS para simular red 5G
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        
        # Publicadores y suscriptores para simular red
        self.control_pub = self.create_publisher(
            Twist, 
            'drone/control_networked', 
            qos_profile
        )
        
        self.telemetry_pub = self.create_publisher(
            Odometry, 
            'drone/telemetry_networked', 
            qos_profile
        )
        
        self.control_sub = self.create_subscription(
            Twist, 
            'drone/control', 
            self.control_callback, 
            qos_profile
        )
        
        self.telemetry_sub = self.create_subscription(
            Odometry, 
            'drone/telemetry', 
            self.telemetry_callback, 
            qos_profile
        )
        
        # Timer para estadísticas
        self.create_timer(5.0, self.print_stats)
        
        self.get_logger().info(f'Network Simulator iniciado con latencia: {self.latency_ms}ms, jitter: {self.jitter_ms}ms')
        
    def control_callback(self, msg):
        """Simula latencia de red en comandos de control"""
        if random.random() * 100 < self.packet_loss_percent:
            self.get_logger().warn('Paquete de control perdido')
            return
            
        # Simular latencia
        delay = self.latency_ms + random.uniform(-self.jitter_ms, self.jitter_ms)
        delay = max(0, delay) / 1000.0  # Convertir a segundos
        
        # Publicar con delay
        timer = threading.Timer(delay, self.publish_control, args=[msg])
        timer.start()
        
    def telemetry_callback(self, msg):
        """Simula latencia de red en telemetría"""
        if random.random() * 100 < self.packet_loss_percent:
            self.get_logger().warn('Paquete de telemetría perdido')
            return
            
        # Simular latencia
        delay = self.latency_ms + random.uniform(-self.jitter_ms, self.jitter_ms)
        delay = max(0, delay) / 1000.0  # Convertir a segundos
        
        # Publicar con delay
        timer = threading.Timer(delay, self.publish_telemetry, args=[msg])
        timer.start()
        
    def publish_control(self, msg):
        """Publica comando de control con latencia simulada"""
        self.control_pub.publish(msg)
        
    def publish_telemetry(self, msg):
        """Publica telemetría con latencia simulada"""
        self.telemetry_pub.publish(msg)
        
    def print_stats(self):
        """Imprime estadísticas de red"""
        self.get_logger().info(f'Red 5G - Latencia: {self.latency_ms}ms, Jitter: {self.jitter_ms}ms, Pérdida: {self.packet_loss_percent}%')


def main(args=None):
    rclpy.init(args=args)
    node = NetworkSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 