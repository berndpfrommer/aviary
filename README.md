# Repository for the UPenn Aviary

This repository hosts code for recording bird behavior at the UPenn aviary.

## Supported platforms

Should run on ROS2 distribution Humble and later.

## How to build

Set the following shell variables:
```bash
repo=aviary
url=https://github.com/ros-event-camera/${repo}.git
```
and follow the [instructions here](https://github.com/ros-misc-utilities/.github/blob/master/docs/build_ros_repository.md)

## How to run

To record the cameras, start the camera sync:
```
ros2 launch aviary cam_sync.launch.py
```

To start and stop the recording:
```
ros2 run aviary start_recording.py
ros2 run aviary stop_recording.py
```
To change parameters (e.g. location of the bag files etc), edit the launch
files in the ``launch`` directory

## License

This software is issued under the Apache License Version 2.0.
