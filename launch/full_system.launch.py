#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import os


def generate_launch_description():
    # Argumentos de lanzamiento
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    enable_web_interface_arg = DeclareLaunchArgument(
        'enable_web_interface',
        default_value='true',
        description='Enable web interface'
    )
    
    enable_network_simulator_arg = DeclareLaunchArgument(
        'enable_network_simulator',
        default_value='true',
        description='Enable 5G network simulation'
    )
    
    # Obtener rutas de paquetes
    drone_controller_pkg = FindPackageShare('drone_controller')
    remote_interface_pkg = FindPackageShare('remote_interface')
    
    # Configuración QoS para 5G
    qos_config_path = PathJoinSubstitution([
        drone_controller_pkg, 'config', 'qos_5g.yaml'
    ])
    
    # Nodo controlador del dron
    drone_controller_node = Node(
        package='drone_controller',
        executable='drone_controller',
        name='drone_controller',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'max_velocity': 5.0},
            {'max_angular_velocity': 2.0},
            {'control_rate': 50.0},
            {'battery_drain_rate': 0.01}
        ],
        remappings=[
            ('/drone/control', '/drone/control'),
            ('/drone/telemetry', '/drone/telemetry'),
            ('/drone/imu', '/drone/imu'),
            ('/drone/battery', '/drone/battery')
        ]
    )
    
    # Nodo servidor web (solo si está habilitado)
    web_server_node = Node(
        package='remote_interface',
        executable='web_server',
        name='web_server',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_web_interface')),
        parameters=[
            {'web_port': 8080},
            {'websocket_port': 8081},
            {'enable_cors': True}
        ]
    )
    
    # Nodo simulador de red 5G (solo si está habilitado)
    network_simulator_node = Node(
        package='network_simulator',
        executable='network_simulator',
        name='network_simulator',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_network_simulator')),
        parameters=[
            {'latency_ms': 5.0},
            {'jitter_ms': 1.0},
            {'packet_loss_percent': 0.1},
            {'bandwidth_mbps': 1000.0}
        ]
    )
    
    # Nodo para publicar transformaciones TF
    tf_publisher_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    # Nodo para visualización RViz (opcional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            drone_controller_pkg, 'config', 'drone_visualization.rviz'
        ])],
        condition=IfCondition(LaunchConfiguration('use_sim_time'))
    )
    
    # Comando para abrir navegador web automáticamente
    open_browser = ExecuteProcess(
        cmd=['python3', '-c', 
             'import webbrowser; import time; time.sleep(3); webbrowser.open("http://localhost:8080")'],
        condition=IfCondition(LaunchConfiguration('enable_web_interface'))
    )
    
    return LaunchDescription([
        # Argumentos
        use_sim_time_arg,
        enable_web_interface_arg,
        enable_network_simulator_arg,
        
        # Nodos
        drone_controller_node,
        web_server_node,
        network_simulator_node,
        tf_publisher_node,
        rviz_node,
        
        # Acciones
        open_browser
    ]) 