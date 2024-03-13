#!/bin/bash
source /opt/ros/iron/setup.bash
source /home/bird/Documents/ws/install/setup.bash

# check if the container is still running, and don't start if it is
pid=`ps -eaf | grep -i '__node:=cam_sync_container' | grep -v grep | awk '{ print $2}'`

if [ -z "${pid}" ]; then
    # nothing running yet, start a new video/audio process
    ros2 launch aviary video_and_audio.launch.py >& /tmp/video_audio_log.txt &
    echo "started video/audio container!" >& /tmp/start_video_audio_output.txt
else
   echo "cam_sync_container is still already running!" >& /tmp/start_video_audio_output.txt
fi


