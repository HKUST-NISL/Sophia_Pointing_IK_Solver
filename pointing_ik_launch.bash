source ../devel/setup.bash

#Launch moveit main for sophia
xterm -hold -e " roslaunch sophia_robot_moveit_config demo.launch" &
sleep 10

#Launch rviz 
rosrun rviz rviz -d ./standard_rviz_config.rviz &
sleep 10


#Launch main program
roslaunch sophia_pointing_ik main.launch



