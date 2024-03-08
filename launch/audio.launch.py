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
from launch_ros.actions import Node


def generate_launch_description():
    audio_capture_node = Node(
        package='audio_capture',
        name='audio',
        executable='audio_capture_node',
        namespace='',
        parameters=[
            {
                'device': 'hw:LP32,0',
                'channels': 24,
                'depth': 24,
                'sample_rate': 48000,
                'sample_format': 'S24LE',
                'coding_format': 'wave',
            }
        ],
        remappings=[],
    )

    return LaunchDescription(
        [
            audio_capture_node,
        ]
    )
