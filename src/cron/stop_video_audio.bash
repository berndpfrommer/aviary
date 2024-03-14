#!/bin/bash -l

pid=`ps -eaf | grep -i '__node:=cam_sync_container' | grep -v grep | awk '{ print $2}'`
if [ -z "${pid}" ]; then
    echo "cannot stop video/audio server because none is running!" > /tmp/stop_video_audio_output.txt
    # nothing running yet, start a new video/audio process
else
    echo "sending kill signal to process $pid!" > /tmp/stop_video_audio_output.txt
    kill $pid
    sleep 15
    pid2=`ps -eaf | grep -i '__node:=cam_sync_container' | grep -v grep | awk '{ print $2}'`
    if [ -z "${pid2}" ]; then
        echo "video_audio_server stopped successfully!" >> /tmp/stop_video_audio_output.txt
    else
	echo "FAILED TO STOP VIDEO/AUDIO SERVER!" >> /tmp/stop_video_audio_output.txt
    fi
fi


