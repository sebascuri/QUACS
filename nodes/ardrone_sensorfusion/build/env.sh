#!/bin/sh


if [ $# -eq 0 ] ; then
    /bin/echo "Entering build environment at /home/parallels/ros_fuerte_ws/nodes/ardrone_sensorfusion/build"
    . /home/parallels/ros_fuerte_ws/nodes/ardrone_sensorfusion/build/setup.sh
    $SHELL -i
    /bin/echo "Exiting build environment at /home/parallels/ros_fuerte_ws/nodes/ardrone_sensorfusion/build"
else
    . /home/parallels/ros_fuerte_ws/nodes/ardrone_sensorfusion/build/setup.sh
    exec "$@"
fi


