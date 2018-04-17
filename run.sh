#!/bin/bash
source /opt/ros/kinetic/setup.bash
(
roscore &
chmod 777 /dev/video1 && roscd gscam &&  export GSCAM_CONFIG="v4l2src device=/dev/video1 ! video/x-raw-rgb,framerate=30/1 ! ffmpegcolorspace" && rosrun gscam gscam &
roslaunch rosbridge_server rosbridge_websocket.launch 
echo ros processes running 
) &&  
node src/index.js
