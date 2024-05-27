#!/bin/bash

echo "123456" | sudo -S chmod 777 /dev/ttyACM0
echo "123456" | sudo -S chmod 777 /dev/ttyACM1

# base
source ~/masvision_v3/devel/setup.bash 
{
    gnome-terminal -t "start1" -- bash -c "roslaunch roborts_bringup base.launch"
}
sleep 1s

# # us_cam
# source ~/masvision_v3/devel/setup.bash 
# {
#     gnome-terminal -t "start2" -- bash -c "rosrun roborts_camera roborts_camera_node"
# }
# sleep 1s

# hikrobot_mvs
source ~/masvision_v3/devel/setup.bash 
{
    gnome-terminal -t "start2" -- bash -c "roslaunch hikrobot_camera hikrobot_camera.launch"
}
sleep 1s

# armor_detect
source ~/masvision_v3/devel/setup.bash 
{
    gnome-terminal -t "start3" -- bash -c "roslaunch roborts_bringup armor_detection.launch"
}
sleep 1s
