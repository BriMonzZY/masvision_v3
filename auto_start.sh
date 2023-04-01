#!/bin/bash

echo "1376447388" | sudo -S chmod 777 /dev/ttyACM0

# base
source ~/masvision_v2/devel/setup.bash 
{
    gnome-terminal -t "start1" -- bash -c "roslaunch roborts_bringup base.launch"
}
sleep 2s

# us_cam
source ~/masvision_v2/devel/setup.bash 
{
    gnome-terminal -t "start2" -- bash -c "rosrun roborts_camera roborts_camera_node"
}
sleep 2s

# armor_detect
source ~/masvision_v2/devel/setup.bash 
{
    gnome-terminal -t "start3" -- bash -c "roslaunch roborts_bringup armor_detection.launch"
}
sleep 2s
