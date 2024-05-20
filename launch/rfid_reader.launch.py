# -----------------------------------------------------------------------------
# Copyright 2024 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    """Launch RFID driver node."""
    node = Node(
        package='rfid_reader',
        executable='rfid_reader_node',
        output='screen',
        name='rfid_reader',
        namespace='rfid_reader',
        parameters=[{'host_name': LaunchConfig('host_name'), 'port': LaunchConfig('port')}],
    )

    return [node]


def generate_launch_description():
    """Create composable node by calling opaque function."""
    return LaunchDescription(
        [
            LaunchArg(
                'host_name',
                default_value='192.168.0.116',
                description='name of rfid tcp server',
            ),
            LaunchArg(
                'port',
                default_value='4662',
                description='port used by rfid tcp server',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
