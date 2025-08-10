#!/usr/bin/env python3
"""
Configuración del Bridge ROS2-Web
Parámetros ajustables para la comunicación entre ROS2 y la interfaz web
"""

# Configuración del servidor WebSocket
WEBSOCKET_CONFIG = {
    'host': 'localhost',
    'port': 8080,
    'max_connections': 10,
    'ping_interval': 5.0,  # segundos
    'ping_timeout': 3.0,   # segundos
}

# Configuración de QoS para ROS2
QOS_CONFIG = {
    'control': {
        'reliability': 'BEST_EFFORT',
        'durability': 'VOLATILE',
        'depth': 10,
        'deadline': 0.01  # 10ms para control crítico
    },
    'telemetry': {
        'reliability': 'RELIABLE',
        'durability': 'VOLATILE',
        'depth': 20,
        'deadline': 0.05  # 50ms para telemetría
    },
    'emergency': {
        'reliability': 'RELIABLE',
        'durability': 'TRANSIENT_LOCAL',
        'depth': 5,
        'deadline': 0.001  # 1ms para emergencias
    }
}

# Configuración de topics ROS2
TOPIC_CONFIG = {
    'drone_arm': '/drone/arm',
    'drone_control': '/drone/control',
    'drone_arm_status': '/drone/arm_status',
    'drone_pose': '/drone/pose',
    'drone_velocity': '/drone/velocity',
    'drone_battery': '/drone/battery',
    'drone_emergency': '/drone/emergency'
}

# Configuración de mensajes
MESSAGE_CONFIG = {
    'max_control_rate': 100,  # Hz - frecuencia máxima de comandos de control
    'max_telemetry_rate': 50,  # Hz - frecuencia máxima de telemetría
    'control_timeout': 0.1,    # segundos - timeout para comandos de control
    'emergency_timeout': 0.001, # segundos - timeout para comandos de emergencia
}

# Configuración de red simulada
NETWORK_CONFIG = {
    'simulate_latency': True,
    'base_latency': 0.005,     # 5ms latencia base
    'latency_variation': 0.002, # 2ms variación
    'packet_loss_rate': 0.001,  # 0.1% pérdida de paquetes
    'bandwidth_limit': 1000000, # 1MB/s límite de ancho de banda
}

# Configuración de logging
LOGGING_CONFIG = {
    'level': 'INFO',
    'format': '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    'file': 'ros2_web_bridge.log',
    'max_file_size': 10 * 1024 * 1024,  # 10MB
    'backup_count': 5
}

# Configuración de seguridad
SECURITY_CONFIG = {
    'enable_authentication': False,
    'allowed_origins': ['http://localhost:3000', 'http://127.0.0.1:3000'],
    'max_message_size': 1024 * 1024,  # 1MB
    'rate_limit': 1000,  # mensajes por minuto por cliente
}

# Configuración de monitoreo
MONITORING_CONFIG = {
    'enable_metrics': True,
    'metrics_interval': 1.0,  # segundos
    'performance_thresholds': {
        'max_latency': 0.1,      # 100ms
        'max_cpu_usage': 80.0,   # 80%
        'max_memory_usage': 512, # 512MB
        'min_fps': 30            # 30 FPS mínimo
    }
}

# Configuración de desarrollo
DEV_CONFIG = {
    'debug_mode': False,
    'mock_data': False,
    'auto_reconnect': True,
    'connection_retries': 3,
    'retry_delay': 1.0,  # segundos
}
