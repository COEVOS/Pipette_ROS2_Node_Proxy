#!/usr/bin/env python3
"""
移液器 UDP 客户端节点启动文件
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # 参数声明
        DeclareLaunchArgument(
            'local_port',
            default_value='10000',
            description='本地 UDP 端口号'
        ),
        DeclareLaunchArgument(
            'discovery_interval',
            default_value='3',
            description='mDNS 发现间隔（秒）'
        ),
        
        # 启动节点
        Node(
            package='pipette_client',
            executable='pipette_client_node',
            name='pipette_client',
            output='screen',
            parameters=[
                {'local_port': '10000'},
                {'discovery_interval': '3'},
            ],
        ),
    ])
