#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def generate_launch_description():
    # Argumentos de lanzamiento
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    enable_network_simulator_arg = DeclareLaunchArgument(
        'enable_network_simulator',
        default_value='true',
        description='Enable 5G network simulation'
    )
    
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
    
    return LaunchDescription([
        # Argumentos
        use_sim_time_arg,
        enable_network_simulator_arg,
        
        # Nodos
        drone_controller_node,
        network_simulator_node,
    ]) 