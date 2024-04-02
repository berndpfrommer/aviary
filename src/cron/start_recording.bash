#!/bin/bash -l
source /opt/ros/iron/setup.bash
source /home/bird/Documents/ws/install/setup.bash
ros2 run aviary start_recording.py >& /tmp/start_recording_output.txt

