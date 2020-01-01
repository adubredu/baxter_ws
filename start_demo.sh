#!/bin/bash

rosrun baxter_tools enable_robot.py -e ;
sleep 2
cd src/run_scripts/src && python move_to_init.py &
# roslaunch openni2_launch openni2.launch &
# sleep 15
python src/run_scripts/src/broadcast_transform.py &
# rosrun dynamic_reconfigure dynparam set /camera/driver depth_registration True;
source /home/bill/ork_ws/devel/setup.bash;
rosrun object_recognition_core detection -c `rospack find object_recognition_tabletop`/conf/detection.table.ros.ork &
python src/run_scripts/src/object_pose.py