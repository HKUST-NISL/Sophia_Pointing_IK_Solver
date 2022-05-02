search_mode=OPTIMIZE_MAX_JOINT
srdf_filename=sophia_robot.srdf
robot_name_in_srdf=sophia_robot
moveit_config_pkg=sophia_robot_moveit_config
robot_name=sophia_robot
planning_group_name=left_arm_group1
ikfast_plugin_pkg=sophia_robot_left_arm_group1_ikfast_plugin
base_link_name=shoulder_center
eef_link_name=left_elbow_artificial
ikfast_output_path=/home/yifan/Dropbox/Vital_Sign_Monitoring/HATP_based/src/sophia_robot_left_arm_group1_ikfast_plugin/src/sophia_robot_left_arm_group1_ikfast_solver.cpp

rosrun moveit_kinematics create_ikfast_moveit_plugin.py\
  --search_mode=$search_mode\
  --srdf_filename=$srdf_filename\
  --robot_name_in_srdf=$robot_name_in_srdf\
  --moveit_config_pkg=$moveit_config_pkg\
  $robot_name\
  $planning_group_name\
  $ikfast_plugin_pkg\
  $base_link_name\
  $eef_link_name\
  $ikfast_output_path
