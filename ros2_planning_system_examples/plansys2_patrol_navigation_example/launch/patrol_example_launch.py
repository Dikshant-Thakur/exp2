# Copyright 2019 Intelligent Robotics Lab
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

import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    # Set environment variable for line buffering
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    # Define the return action server node
    return_cmd = Node(
        package='plansys2_patrol_navigation_example',
        executable='return_action_node',
        name='return_action_node',
        output='screen',
        parameters=[])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variable
    ld.add_action(stdout_linebuf_envvar)

    # Add the return action server node
    ld.add_action(return_cmd)

    return ld

