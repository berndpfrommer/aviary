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

from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument as LaunchArg
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode

camera_list = {
    "cam0": "18057299",
    "cam1": "18025945",
    "cam2": "17408093",
    "cam3": "18288156",
    "cam4": "18057298",
    "cam5": "18057303",
    "cam6": "18057304",   # not working
    "cam7": "18025950",
    "cam8": "23199575",
}


def make_topics():
    topics = [f"/cam_sync/{c}/image_raw/ffmpeg" for c in camera_list]
    topics += [f"/cam_sync/{c}/camera_info" for c in camera_list]
    topics += [f"/audio/audio_stamped"]
    return topics


def make_name(prefix, context):
    now = datetime.now()
    return prefix.perform(context) + now.strftime('%Y_%m_%d-%H_%M_%S')


def launch_setup(context, *args, **kwargs):
    launch_action = LoadComposableNodes(
        target_container=LaunchConfig('container_name'),
        composable_node_descriptions=[
            ComposableNode(
                package='rosbag2_transport',
                plugin='rosbag2_transport::Recorder',
                name='recorder',
                parameters=[
                    {
                        'record.topics': make_topics(),
                        'record.start_paused': False,
                        'storage.uri': make_name(LaunchConfig('bag_prefix'), context),
                    }
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
    )
    return [launch_action]


def generate_launch_description():
    """Create composable node by calling opaque function."""
    return LaunchDescription(
        [
            LaunchArg(
                'container_name',
                default_value=['cam_sync_container'],
                description='name of cam_sync container node',
            ),
            LaunchArg('bag_prefix', default_value=['rosbag2_'], description='prefix of rosbag'),
            OpaqueFunction(function=launch_setup),
        ]
    )
