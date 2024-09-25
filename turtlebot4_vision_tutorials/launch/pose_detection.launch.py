#!/usr/bin/env python3

# Copyright 2024 Clearpath Robotics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# @author Hilary Luo (hluo@clearpathrobotics.com)

# from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    # ffmpeg_param_file = LaunchConfiguration('ffmpeg_param_file')

    # turtlebot4_vision_tutorials = get_package_share_directory('turtlebot4_vision_tutorials')

    # arg_parameters = DeclareLaunchArgument(
    #     'ffmpeg_param_file',
    #     default_value=PathJoinSubstitution(
    #         [turtlebot4_vision_tutorials, 'config', 'ffmpeg.yaml']),
    #     description='Turtlebot4 ffmpeg compression param file'
    # )

    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='')

    # parameters = RewrittenYaml(
    #     source_file=ffmpeg_param_file,
    #     root_key=namespace,
    #     param_rewrites={},
    #     convert_types=True)

    ffmpeg_node = Node(
        package='turtlebot4_vision_tutorials',
        executable='pose_detection',
        namespace=namespace,
        name='pose_detection',
        # parameters=[parameters],
    )

    ld = LaunchDescription()
    # ld.add_action(arg_parameters)
    ld.add_action(arg_namespace)
    ld.add_action(ffmpeg_node)

    return ld
