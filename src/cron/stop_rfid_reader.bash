#!/bin/bash -l

pid=`ps -eaf | grep -i '__node:=rfid_reader' | grep -v grep | awk '{ print $2}'`
if [ -z "${pid}" ]; then
    echo "cannot stop rfid reader because none is running!" > /tmp/stop_rfid_reader_output.txt
    # nothing running yet, start a new video/audio process
else
    echo "sending kill signal to process $pid!" > /tmp/stop_rfid_reader_output.txt
    kill $pid
    sleep 15
    pid2=`ps -eaf | grep -i '__node:=rfid_reader' | grep -v grep | awk '{ print $2}'`
    if [ -z "${pid2}" ]; then
        echo "rfid_reader stopped successfully!" >> /tmp/stop_rfid_reader_output.txt
    else
	echo "FAILED TO STOP RFID READER!" >> /tmp/stop_rfid_reader_output.txt
    fi
fi


