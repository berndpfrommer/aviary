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
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


example_parameters = {
    'blackfly': {
    'debug': False,
    'quiet': True,
    'buffer_queue_size': 1,
    'compute_brightness': True,
    'exposure_auto': 'Off',
    'exposure_time': 10000,  # not used under auto exposure
    'gain_auto': 'Off',
    'frame_rate_auto': 'Off',
    # 'frame_rate_enable': False,
    # 'frame_rate': 25.0,
    'trigger_mode': 'On',
    'trigger_source': 'Line0',
    'trigger_selector': 'FrameStart',
    'trigger_overlap': 'ReadOut',
    'trigger_activation': 'RisingEdge',
    'gev_scps_packet_size': 9000,
    'device_link_throughput_limit': 124944000,
    # You must enable chunk mode and at least frame_id
    'chunk_mode_active': True,
    'chunk_selector_frame_id': 'FrameID',
    'chunk_enable_frame_id': True,
    # exposure time and gain chunks are required for exposure control
    'chunk_selector_exposure_time': 'ExposureTime',
    'chunk_enable_exposure_time': True,
    'chunk_selector_gain': 'Gain',
    'chunk_enable_gain': True,
    # The Timestamp is not used at the moment
    'chunk_selector_timestamp': 'Timestamp',
    'chunk_enable_timestamp': True,
    }
}

def launch_setup(context, *args, **kwargs):
    """Launch camera driver node."""
    parameter_file = LaunchConfig('parameter_file').perform(context)
    camera_type = LaunchConfig('camera_type').perform(context)
    serial = str(LaunchConfig('serial').perform(context))
    camera_name = "cam_" + serial
    print(camera_name)
    if not parameter_file:
        parameter_file = PathJoinSubstitution(
            [FindPackageShare('spinnaker_camera_driver'), 'config',
             camera_type + '.yaml'])
    if camera_type not in example_parameters:
        raise Exception('no example parameters available for type ' + camera_type)

    node = Node(package='spinnaker_camera_driver',
                executable='camera_driver_node',
                output='screen',
                name=[camera_name],
                parameters=[example_parameters[camera_type],
                            {'ffmpeg_image_transport.encoding': 'hevc_nvenc',
                             'parameter_file': parameter_file,
                             'serial_number': serial}],
                remappings=[('~/control', '/exposure_control/control'), ])

    return [node]


def generate_launch_description():
    """Create composable node by calling opaque function."""
    return LaunchDescription([
        LaunchArg('camera_type', default_value='blackfly',
                  description='type of camera (blackfly, chameleon...)'),
        LaunchArg('serial', # default_value="'18057299'",
                  description='FLIR serial number of camera (in quotes!!)'),
        LaunchArg('parameter_file', default_value='',
                  description='path to ros parameter definition file (override camera type)'),
        OpaqueFunction(function=launch_setup)
        ])
