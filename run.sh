#!/bin/bash
source /opt/ros/kinetic/setup.bash

while true; do 
    if [[ "$(netstat -a | grep 9090)" ]]; then   
        echo "rosbridge found!!!!!!!!" 
        node src/index.js 
        break
    else
        echo "rosbridge not found" 
        sleep 1
    fi
done &

roscore &
chmod 777 /dev/video1 && roscd gscam &&  export GSCAM_CONFIG="v4l2src device=/dev/video1 ! video/x-raw-rgb,framerate=30/1 ! ffmpegcolorspace" && rosrun gscam gscam &
roslaunch rosbridge_server rosbridge_websocket.launch &
