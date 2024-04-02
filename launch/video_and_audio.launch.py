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
from launch.substitutions import PathJoinSubstitution as PJoin
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

camera_list = {
    'cam0': '18057299',
    'cam1': '18025945',
    'cam2': '17408093',
    'cam3': '18288156',
    'cam4': '18057298',
    'cam5': '18057303',
    'cam6': '18057304',
    'cam7': '18025950',
    'cam8': '23199575',
    'cam9': '18288153',
}

camera_list_all = {
    'cam0': '18057299',
    'cam1': '18025945',
    'cam2': '17408093',
    'cam3': '18288156',
    'cam4': '18057298',
    'cam5': '18057303',
    'cam6': '18057304',
    'cam7': '18025950',
    'cam8': '23199575',
    'cam9': '18288153',
}

exposure_controller_parameters = {
    'type': 'individual',
    'brightness_target': 120,
    'brightness_tolerance': 20,
    # watch that max_exposure_time is short enough
    # to support the trigger frame rate!
    'max_exposure_time': 15000,  # usec
    'min_exposure_time': 5000,  # usec
    'max_gain': 29.9,
    'gain_priority': False,
}

cam_parameters = {
    'debug': False,
    'quiet': True,
    'buffer_queue_size': 1,
    'compute_brightness': True,
    'exposure_auto': 'Off',
    'exposure_time': 10000,  # not used under auto exposure
    'gain_auto': 'Off',
    'trigger_mode': 'On',
    'trigger_source': 'Line0',
    'trigger_selector': 'FrameStart',
    'trigger_overlap': 'ReadOut',
    'trigger_activation': 'RisingEdge',
    'gev_scps_packet_size': 9000,
    'device_link_throughput_limit': 124992000,  # 124992000, 115840000
    # You must enable chunk mode and at least frame_id
    'chunk_mode_active': True,
    'chunk_selector_frame_id': 'FrameID',
    'chunk_enable_frame_id': True,
    'balance_white_auto': 'Continuous',
    # exposure time and gain chunks are required for exposure control
    'chunk_selector_exposure_time': 'ExposureTime',
    'chunk_enable_exposure_time': True,
    'chunk_selector_gain': 'Gain',
    'chunk_enable_gain': True,
    # The Timestamp is not used at the moment
    'chunk_selector_timestamp': 'Timestamp',
    'chunk_enable_timestamp': True,
}


def make_parameters(context):
    """Launch synchronized camera driver node."""
    pd = LaunchConfig('camera_parameter_directory')
    calib_url = "file://" + LaunchConfig('calibration_directory').perform(context) + "/"

    exp_ctrl_names = [cam + '.exposure_controller' for cam in camera_list.keys()]
    driver_parameters = {
        'cameras': list(camera_list.keys()),
        'exposure_controllers': exp_ctrl_names,
        'ffmpeg_image_transport.encoding': 'hevc_nvenc',  # only for ffmpeg image transport
    }
    # generate identical exposure controller parameters
    for exp in exp_ctrl_names:
        driver_parameters.update(
            {exp + '.' + k: v for k, v in exposure_controller_parameters.items()}
        )

    # generate camera parameters
    cam_parameters['parameter_file'] = PJoin([pd, 'blackfly.yaml'])
    for cam, serial in camera_list.items():
        cam_params = {cam + '.' + k: v for k, v in cam_parameters.items()}
        cam_params[cam + '.serial_number'] = serial
        cam_params[cam + '.camerainfo_url'] = calib_url + serial + ".yaml"
        cam_params[cam + '.frame_id'] = cam
        driver_parameters.update(cam_params)  # insert into main parameter list
        # link the camera to its exposure controller
        driver_parameters.update({cam + '.exposure_controller_name': cam + '.exposure_controller'})
    return driver_parameters


def launch_setup(context, *args, **kwargs):
    container = ComposableNodeContainer(
        name='cam_sync_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='spinnaker_synchronized_camera_driver',
                plugin='spinnaker_synchronized_camera_driver::SynchronizedCameraDriver',
                name='cam_sync',
                parameters=[make_parameters(context)],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='audio_capture',
                plugin='audio_capture::AudioCaptureNode',
                name='audio',
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
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
        additional_env={
            'RCUTILS_CONSOLE_OUTPUT_FORMAT': '[{severity} {time}] [{name}]: {message}'
        },
    )  # end of container
    return [container]


def generate_launch_description():
    return LaunchDescription(
        [
            LaunchArg(
                'camera_parameter_directory',
                default_value=PJoin([FindPackageShare('spinnaker_camera_driver'), 'config']),
                description='root directory for camera parameter definitions',
            ),
            LaunchArg(
                'calibration_directory',
                default_value=PJoin([FindPackageShare('aviary'), 'config/calibration-2024-03']),
                description='root directory for camera calibration files',
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
