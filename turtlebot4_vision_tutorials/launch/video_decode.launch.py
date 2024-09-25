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
    namespace = LaunchConfiguration('namespace')

    arg_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='')

    parameters = {
        "/cpr_donatello/ffmpeg_decoder": {
            "ros__parameters": {
                "qos_overrides": {
                    "/cpr_donatello/oakd/rgb/preview/encoded/ffmpeg": {
                        "subscriber": {
                            "reliability": "best_effort",
                            "depth": 10,
                            "history": "keep_last"
                        }
                    }
                }
            }
        }
    }

    ffmpeg_node = Node(
        package='image_transport',
        executable='republish',
        namespace=namespace,
        name='ffmpeg_decoder',
        remappings=[
            ('in/ffmpeg', "oakd/rgb/preview/encoded/ffmpeg"),
            ('out', "oakd/rgb/preview/ffmpeg_decoded"),
            ],
        arguments=['ffmpeg', 'raw'],
        parameters=[parameters],
    )

    ld = LaunchDescription()
    ld.add_action(arg_namespace)
    ld.add_action(ffmpeg_node)

    return ld
