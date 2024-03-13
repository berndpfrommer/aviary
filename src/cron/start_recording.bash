#!/bin/bash
source /opt/ros/iron/setup.bash
source /home/bird/Documents/ws/install/setup.bash
ros2 run aviary start_recording.py bag_prefix:=/data/recordings/rosbag2_ >& /tmp/start_recording_output.txt

