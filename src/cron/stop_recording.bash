#!/bin/bash
source /opt/ros/iron/setup.bash
source /home/bird/Documents/ws/install/setup.bash
ros2 run aviary stop_recording.py >& /tmp/stop_recording_output.txt

