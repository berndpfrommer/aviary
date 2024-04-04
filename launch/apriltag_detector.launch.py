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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def launch_setup(context, *args, **kwargs):
    container = ComposableNodeContainer(
        name='detector_container',
        namespace=LaunchConfig('camera'),
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='apriltag_ros',
                plugin='apriltag_ros::ApriltagDetectorComponent',
                name='detector',
                namespace=LaunchConfig('camera'),
                parameters=[
                    {
                        'tag_family': 0,  # 36h11
                        'detector': 0,
                        'black_border_width': 1,
                        'decimate': 0,
                    }
                ],
                remappings=[
                    ('image', 'image_raw'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )  # end of container
    return [container]


def generate_launch_description():
    return LaunchDescription(
        [
            LaunchArg(
                'camera',
                default_value='/cam_sync/cam0',
                description='name of camera (e.g. /cam_sync/cam0)',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
