#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from sensor_msgs.msg import Imu, BatteryState
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Bool
import math
import numpy as np
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class DroneController(Node):
    """
    Controlador principal del dron que maneja comandos de control
    y publica el estado del dron en tiempo real.
    """
    
    def __init__(self):
        super().__init__('drone_controller')
        
        # Configuración QoS para baja latencia (simulación 5G)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Estado del dron
        self.drone_state = {
            'position': [0.0, 0.0, 0.0],  # x, y, z
            'orientation': [0.0, 0.0, 0.0],  # roll, pitch, yaw
            'velocity': [0.0, 0.0, 0.0],  # vx, vy, vz
            'battery': 100.0,
            'armed': False,
            'flight_mode': 'MANUAL'
        }
        
        # Parámetros de control
        self.max_velocity = 5.0  # m/s
        self.max_angular_velocity = 2.0  # rad/s
        self.control_rate = 50.0  # Hz
        
        # Publishers
        self.telemetry_pub = self.create_publisher(
            Odometry, '/drone/telemetry', qos_profile)
        self.imu_pub = self.create_publisher(
            Imu, '/drone/imu', qos_profile)
        self.battery_pub = self.create_publisher(
            BatteryState, '/drone/battery', qos_profile)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/drone/pose', qos_profile)
        
        # Subscribers
        self.control_sub = self.create_subscription(
            Twist, '/drone/control', self.control_callback, qos_profile)
        self.arm_sub = self.create_subscription(
            Bool, '/drone/arm', self.arm_callback, qos_profile)
        
        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer para actualización del estado
        self.timer = self.create_timer(1.0/self.control_rate, self.update_state)
        
        self.get_logger().info('Drone Controller iniciado')
    
    def control_callback(self, msg):
        """Callback para comandos de control del dron"""
        if not self.drone_state['armed']:
            self.get_logger().warn('Dron no armado. Ignorando comandos de control.')
            return
        
        # Aplicar límites de velocidad
        linear_vel = [
            np.clip(msg.linear.x, -self.max_velocity, self.max_velocity),
            np.clip(msg.linear.y, -self.max_velocity, self.max_velocity),
            np.clip(msg.linear.z, -self.max_velocity, self.max_velocity)
        ]
        
        angular_vel = [
            np.clip(msg.angular.x, -self.max_angular_velocity, self.max_angular_velocity),
            np.clip(msg.angular.y, -self.max_angular_velocity, self.max_angular_velocity),
            np.clip(msg.angular.z, -self.max_angular_velocity, self.max_angular_velocity)
        ]
        
        # Actualizar velocidades
        self.drone_state['velocity'] = linear_vel
        
        # Simular movimiento del dron
        dt = 1.0 / self.control_rate
        self.drone_state['position'][0] += linear_vel[0] * dt
        self.drone_state['position'][1] += linear_vel[1] * dt
        self.drone_state['position'][2] += linear_vel[2] * dt
        
        # Actualizar orientación
        self.drone_state['orientation'][0] += angular_vel[0] * dt
        self.drone_state['orientation'][1] += angular_vel[1] * dt
        self.drone_state['orientation'][2] += angular_vel[2] * dt
        
        # Mantener yaw en rango [-π, π]
        self.drone_state['orientation'][2] = math.atan2(
            math.sin(self.drone_state['orientation'][2]),
            math.cos(self.drone_state['orientation'][2])
        )
        
        # Simular consumo de batería
        velocity_magnitude = math.sqrt(sum(v**2 for v in linear_vel))
        self.drone_state['battery'] -= velocity_magnitude * 0.01 * dt
        
        if self.drone_state['battery'] < 0:
            self.drone_state['battery'] = 0
            self.drone_state['armed'] = False
            self.get_logger().warn('Batería agotada. Desarmando dron.')
    
    def arm_callback(self, msg):
        """Callback para armar/desarmar el dron"""
        if msg.data and self.drone_state['battery'] > 10.0:
            self.drone_state['armed'] = True
            self.get_logger().info('Dron armado')
        elif not msg.data:
            self.drone_state['armed'] = False
            self.get_logger().info('Dron desarmado')
        else:
            self.get_logger().warn('No se puede armar: batería baja')
    
    def update_state(self):
        """Actualizar y publicar el estado del dron"""
        # Publicar Odometry
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'drone'
        
        # Posición
        odom_msg.pose.pose.position.x = self.drone_state['position'][0]
        odom_msg.pose.pose.position.y = self.drone_state['position'][1]
        odom_msg.pose.pose.position.z = self.drone_state['position'][2]
        
        # Orientación (convertir Euler a Quaternion)
        q = self.euler_to_quaternion(
            self.drone_state['orientation'][0],
            self.drone_state['orientation'][1],
            self.drone_state['orientation'][2]
        )
        odom_msg.pose.pose.orientation = q
        
        # Velocidad
        odom_msg.twist.twist.linear.x = self.drone_state['velocity'][0]
        odom_msg.twist.twist.linear.y = self.drone_state['velocity'][1]
        odom_msg.twist.twist.linear.z = self.drone_state['velocity'][2]
        
        self.telemetry_pub.publish(odom_msg)
        
        # Publicar IMU (simulado)
        imu_msg = Imu()
        imu_msg.header = odom_msg.header
        imu_msg.header.frame_id = 'drone'
        
        # Simular ruido en el IMU
        imu_msg.angular_velocity.x = self.drone_state['velocity'][0] + np.random.normal(0, 0.01)
        imu_msg.angular_velocity.y = self.drone_state['velocity'][1] + np.random.normal(0, 0.01)
        imu_msg.angular_velocity.z = self.drone_state['velocity'][2] + np.random.normal(0, 0.01)
        
        imu_msg.linear_acceleration.x = np.random.normal(0, 0.1)
        imu_msg.linear_acceleration.y = np.random.normal(0, 0.1)
        imu_msg.linear_acceleration.z = 9.81 + np.random.normal(0, 0.1)
        
        self.imu_pub.publish(imu_msg)
        
        # Publicar estado de batería
        battery_msg = BatteryState()
        battery_msg.header = odom_msg.header
        battery_msg.percentage = self.drone_state['battery'] / 100.0
        battery_msg.voltage = 12.0 * (self.drone_state['battery'] / 100.0)
        battery_msg.present = True
        
        self.battery_pub.publish(battery_msg)
        
        # Publicar TF
        self.publish_tf()
    
    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convertir ángulos de Euler a cuaternión"""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        q = Quaternion()
        q.w = cy * cp * cr + sy * sp * sr
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr
        
        return q
    
    def publish_tf(self):
        """Publicar transformación TF del dron"""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'drone'
        
        t.transform.translation.x = self.drone_state['position'][0]
        t.transform.translation.y = self.drone_state['position'][1]
        t.transform.translation.z = self.drone_state['position'][2]
        
        q = self.euler_to_quaternion(
            self.drone_state['orientation'][0],
            self.drone_state['orientation'][1],
            self.drone_state['orientation'][2]
        )
        t.transform.rotation = q
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    controller = DroneController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 