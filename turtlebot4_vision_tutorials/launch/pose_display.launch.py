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

from launch import LaunchDescription
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespaces = ['tb21', 'tb10']
    # namespaces = LaunchConfiguration('namespaces')

    # arg_namespaces = DeclareLaunchArgument(
    #     'namespaces',
    #     default_value='[tb10, tb11]')

    ffmpeg_nodes = []

    for i, ns in enumerate(namespaces):
        ffmpeg_nodes.append(
            Node(
                package='image_transport',
                executable='republish',
                name=f'ffmpeg_decoder{i}',
                remappings=[
                    ('in/ffmpeg', f'/{ns}/oakd/rgb/preview/encoded/ffmpeg'),
                    ('out', f'/{ns}/oakd/rgb/preview/ffmpeg_decoded')],
                arguments=['ffmpeg', 'raw'],
            )
        )

    display_node = Node(
        package='turtlebot4_vision_tutorials',
        executable='pose_display',
        name='pose_display',
        parameters=[{'namespaces': namespaces}],

    )

    ld = LaunchDescription()
    # ld.add_action(arg_namespaces)
    for n in ffmpeg_nodes:
        ld.add_action(n)
    ld.add_action(display_node)

    return ld
