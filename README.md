# Repository for the UPenn Aviary

This repository hosts code for recording bird behavior at the UPenn aviary.

## Supported platforms

Should run on ROS2 distribution Iron Irwini and later.

## How to build

Set the following shell variables:
```bash
repo=aviary
url=https://github.com/berndpfrommer/${repo}.git
```
and follow the [instructions here](https://github.com/ros-misc-utilities/.github/blob/master/docs/build_ros_repository.md)

## How to record data

Start the camera and audio server:
```
ros2 launch aviary video_and_audio.launch.py
```

To start and stop the recording:
```bash
ros2 run aviary start_recording.py bag_prefix:=/data/recordings/rosbag2_
ros2 run aviary stop_recording.py
```

## How to process data

The ``bag_to_wave`` tool converts a recorded rosbag to an audio ``.wv`` file:

```bash
bag_to_wave -b input_bag [-t topic] [-o out_file] [-T timestamp_file] [-s start_time] [-e end_time] [-E encoding] [-c channels] [-r rate]
```

Parameters:

- ``rate``: the rate that ffmpeg uses for encoding (default: 48000).
    Must match the rate at which audio is actually recorded.
- ``topic``: the ros recording topic (default:  ``/audio/audio_stamped``)
- ``timestamp_file``: file with the ros recording topic (default: ``timestamps.txt``).
  This file has the audio packet number, the header stamp (time when audio packet arrived),
  and the recording stamp (time when audio packet was written to the rosbag). 
- ``channels``: number of audio channels (default: 24, corresponding to current aviary setup)
- ``start_time``: beginning of extraction time, in seconds since epoch. Defaults to start of bag. Use ``ros2 bag info`` to get start time in seconds since epoch.
- ``end_time``: end of extraction time, in seconds since epoch. Defaults to start of bag. Use ``ros2 bag info`` to get start time in seconds since epoch.
- ``encoding``: ffmpeg encoding to use, defaults to ``wavepack``.


Example usage:

```bash
ros2 run aviary bag_to_wave  -b /data/recordings/rosbag2_2024_03_14-08_55_15
```

## Notes on bag incompatibility:

The bag recording requires a very latest implementation of the rosbag
recording node, which only is available on ROS2 versions greater than
Iron. Alas, the bag format has changed after ROS2 Iron, so to be able
to process the bags under ROS2 iron, they must be converted to the
older format like so:

```
ros2 run aviary modify_bag_meta.py -b <path_to_bag_directory>
```

## License

This software is issued under the Apache License Version 2.0.
