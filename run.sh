#!/bin/bash
source /opt/ros/kinetic/setup.bash
source /rosserver/devel/setup.bash

chmod 777 /dev/video0 && roscd gscam &&  export GSCAM_CONFIG="v4l2src device=/dev/video0 ! video/x-raw-rgb,framerate=30/1 ! ffmpegcolorspace"

roscore &
rosrun gscam gscam & 
roslaunch rosbridge_server rosbridge_websocket.launch &

while true; do 
    if [[ "$(netstat -a | grep 9090)" ]]; then   
        echo "rosbridge found!!!!!!!!" 
        node /home/pi/rosserver/new/index.js 
        break
    else
        echo "rosbridge not found" 
        sleep 1
    fi
done &


