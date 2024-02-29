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
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import SetEnvironmentVariable

from datetime import datetime

camera_list = {
    "cam0": "18057299",
    "cam1": "18025945",
#    "cam2": "17408093",
#    "cam3": "18288156",
#    "cam4": "18057298",
#    "cam5": "18057303",
   # "cam6": "18057304",   # not working
#    "cam7": "18025950",
#    "cam8": "23199575",
}

camera_list_all = {
    "cam0": "18057299",
    "cam1": "18025945",
    "cam2": "17408093",
    "cam3": "18288156",
    "cam4": "18057298",
    "cam5": "18057303",
    # "cam6": "18057304",   # not working
    "cam7": "18025950",
    "cam8": "23199575"}

exposure_controller_parameters = {
    "type" : "individual",
    "brightness_target": 120,
    "brightness_tolerance": 5,
    # watch that max_exposure_time is short enough
    # to support the trigger frame rate!
    "max_exposure_time": 20000,  # usec
    "min_exposure_time": 5000,  # usec
    "max_gain": 29.9,
    "gain_priority": False
}

cam_parameters = {
    "debug": False,
    "quiet": True,
    "buffer_queue_size": 1,
    "compute_brightness": True,
    "exposure_auto": "Off",
    "exposure_time": 10000,  # not used under auto exposure
    "gain_auto": "Off",
    "trigger_mode": "On",
    "trigger_source": "Line0",
    "trigger_selector": "FrameStart",
    "trigger_overlap": "ReadOut",
    "trigger_activation": "RisingEdge",
    "gev_scps_packet_size": 9000,
    "device_link_throughput_limit": 124992000, # 124992000, 115840000
    # You must enable chunk mode and at least frame_id
    "chunk_mode_active": True,
    "chunk_selector_frame_id": "FrameID",
    "chunk_enable_frame_id": True,
    # exposure time and gain chunks are required for exposure control
    "chunk_selector_exposure_time": "ExposureTime",
    "chunk_enable_exposure_time": True,
    "chunk_selector_gain": "Gain",
    "chunk_enable_gain": True,
    # The Timestamp is not used at the moment
    "chunk_selector_timestamp": "Timestamp",
    "chunk_enable_timestamp": True,
}

def make_parameters():
    """Launch synchronized camera driver node."""
    pd = LaunchConfig("camera_parameter_directory")

    exp_ctrl_names = [cam + ".exposure_controller" for cam in camera_list.keys()]
    driver_parameters = {
        "cameras": list(camera_list.keys()),
        "exposure_controllers": exp_ctrl_names,
        "ffmpeg_image_transport.encoding": "hevc_nvenc",  # ignored unless using ffmpeg image transport
    }
    # generate identical exposure controller parameters
    for exp in exp_ctrl_names:
        driver_parameters.update(
            {exp + "." + k:v for k, v in exposure_controller_parameters.items()})
        
    # generate camera parameters
    cam_parameters["parameter_file"] = PJoin([pd, "blackfly.yaml"])
    for cam, serial in camera_list.items():
        cam_params = {cam + "." + k: v for k, v in cam_parameters.items()}
        cam_params[cam + ".serial_number"] = serial
        driver_parameters.update(cam_params)  # insert into main parameter list
        # link the camera to its exposure controller
        driver_parameters.update(
            {cam + ".exposure_controller_name": cam + ".exposure_controller"})
    return driver_parameters


def make_topics():
    topics = [f"/cam_sync/{c}/image_raw/ffmpeg" for c in camera_list]
    topics += [f"/cam_sync/{c}/camera_info" for c in camera_list]
    return topics

def make_name(prefix, context):
    now = datetime.now()
    return prefix.perform(context) + now.strftime('%Y-%m-%d-%H-%M-%S')

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
                parameters=[make_parameters()],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='rosbag2_transport',
                plugin='rosbag2_transport::Recorder',
                name='recorder',
                parameters=[
                    {   'record.topics': make_topics(),
                        'record.start_paused': True,
                        'storage.uri': make_name(LaunchConfig('bag_prefix'), context),
                    }],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
#            ComposableNode(
#                package='rosbag2_composable_recorder',
#                plugin='rosbag2_composable_recorder::ComposableRecorder',
#                name="recorder",
#                parameters=[{'topics': make_topics(),
#                             'storage_id': 'mcap',
#                             'record_all': False,
#                             'disable_discovery': False,
#                             'serialization_format': 'cdr',
#                             'start_recording_immediately': False,
#                             'bag_prefix': LaunchConfig('bag_prefix')}],
#                remappings=[],
#                extra_arguments=[{'use_intra_process_comms': True}],
#            )
        ],
        output='screen',
        additional_env={'RCUTILS_CONSOLE_OUTPUT_FORMAT':
                        "[{severity} {time}] [{name}]: {message}"},
    )  # end of container
    return [container]

def launch_setup_node(context, *args, **kwargs):
    """Launch synchronized camera driver node."""
    pd = LaunchConfig("camera_parameter_directory")
    driver_parameters = {
        "cameras": list(camera_list.keys()),
        "ffmpeg_image_transport.encoding": "hevc_nvenc",
    }
    cam_parameters["parameter_file"] = PJoin([pd, "blackfly.yaml"])

    for cam, serial in camera_list.items():
        cam_params = {cam + "." + k: v for k, v in cam_parameters.items()}
        cam_params[cam + ".serial_number"] = serial
        driver_parameters.update(cam_params)  # insert into main parameter list

    node = Node(
        package="spinnaker_synchronized_camera_driver",
        executable="synchronized_camera_driver_node",
        output="screen",
        # prefix=["xterm -e gdb -ex run --args"],
        name=[LaunchConfig("driver_name")],
        parameters=[
            driver_parameters,
        ],
    )
    return [node]


def generate_launch_description():
    """Create composable node by calling opaque function."""
    return LaunchDescription(
        [
            LaunchArg(
                "driver_name",
                default_value=["cam_sync"],
                description="name of driver node",
            ),
            LaunchArg(
                "camera_parameter_directory",
                default_value=PJoin(
                    [FindPackageShare("spinnaker_camera_driver"), "config"]
                ),
                description="root directory for camera parameter definitions",
            ),
            LaunchArg('bag_prefix', default_value=['rosbag2_'],
                      description='prefix of rosbag'),
            OpaqueFunction(function=launch_setup),
        ]
    )
