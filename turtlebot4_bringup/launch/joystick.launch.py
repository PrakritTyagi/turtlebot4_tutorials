from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    robot_namespace = LaunchConfiguration('robot_namespace')

    declare_robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='robot1',  # default robot if none specified
        description='Namespace of the robot'
    )


    joy_params = os.path.join(get_package_share_directory('turtlebot4_bringup'),'config','teleop.yaml')

    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[joy_params],
         )
    
    teleop_node = Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_node',
            parameters=[joy_params],
            remappings=[('/cmd_vel', [robot_namespace, '/cmd_vel'])],
         )

    return LaunchDescription([
        declare_robot_namespace_arg,
        joy_node,
        teleop_node
    ])
