#!/bin/bash -l
source /opt/ros/iron/setup.bash
source /home/bird/Documents/ws/install/setup.bash

# check if the container is still running, and don't start if it is
pid=`ps -eaf | grep -i '__node:=rfid_reader' | grep -v grep | awk '{ print $2}'`

if [ -z "${pid}" ]; then
    # nothing running yet, start a new rfid reader process
    ros2 launch aviary rfid_reader.launch.py >& /tmp/rfid_reader_log.txt &
    echo "started rfid reader!" >& /tmp/start_rfid_reader_output.txt
else
   echo "rfid_reader is still already running!" >& /tmp/start_rfid_reader_output.txt
fi


