import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

BEACON_CFG_DIR = "/home/ubuntu/beacon_configs"


def generate_launch_description():
    robot_ns   = LaunchConfiguration('robot_namespace', default='tb3_1')
    beacon_a   = LaunchConfiguration('beacon_a_addr',   default='1')
    beacon_b   = LaunchConfiguration('beacon_b_addr',   default='2')
    timeout    = LaunchConfiguration('beacon_timeout',  default='1.0')
    pos_sigma  = LaunchConfiguration('pos_sigma',       default='0.30')
    head_sigma = LaunchConfiguration('heading_sigma',   default='0.2236')

    config_a = os.path.join(BEACON_CFG_DIR, 'beacon_a.yaml')
    config_b = os.path.join(BEACON_CFG_DIR, 'beacon_b.yaml')

    driver_a = Node(
        package='marvelmind_ros2',
        executable='marvelmind_ros2',
        name=['marvelmind_driver_', beacon_a],
        output='screen',
        parameters=[config_a],
    )

    driver_b = Node(
        package='marvelmind_ros2',
        executable='marvelmind_ros2',
        name=['marvelmind_driver_', beacon_b],
        output='screen',
        parameters=[config_b],
    )

    bridge = Node(
        package='tb3_beacon_bridge',
        executable='beacon_bridge_node',
        name='beacon_bridge_node',
        namespace=robot_ns,
        output='screen',
        parameters=[{
            'robot_namespace': robot_ns,
            'beacon_a_addr':   beacon_a,
            'beacon_b_addr':   beacon_b,
            'beacon_timeout':  timeout,
            'pos_sigma':       pos_sigma,
            'heading_sigma':   head_sigma,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('robot_namespace', default_value='tb3_1'),
        DeclareLaunchArgument('beacon_a_addr',   default_value='1'),
        DeclareLaunchArgument('beacon_b_addr',   default_value='2'),
        DeclareLaunchArgument('beacon_timeout',  default_value='1.0'),
        DeclareLaunchArgument('pos_sigma',        default_value='0.30'),
        DeclareLaunchArgument('heading_sigma',    default_value='0.2236'),
        driver_a,
        driver_b,
        bridge,
    ])
