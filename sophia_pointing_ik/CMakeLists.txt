cmake_minimum_required(VERSION 3.0.2)
project(sophia_pointing_ik)

# Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_compile_options(-std=c++14)
add_compile_options(-std=c++17)

#For eigen3
find_package( PkgConfig )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )
include_directories( ${EIGEN3_INCLUDE_DIRS} )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs 
  tf2
  tf2_ros
  tf2_msgs
  rosbag
  hr_msgs
  #Moveit related
  interactive_markers
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  rviz_visual_tools
  moveit_visual_tools
  pluginlib
  geometric_shapes
  pcl_ros
  pcl_conversions
  rosbag
  tf2_eigen
  tf2_geometry_msgs
)

# System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)


#fmt
find_package(fmt)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES generated
   CATKIN_DEPENDS 
      roscpp 
      std_msgs 
      geometry_msgs 
      tf2_msgs 
      hr_msgs
      moveit_core
      moveit_visual_tools
      moveit_ros_planning_interface
      interactive_markers
      tf2_geometry_msgs
      moveit_msgs
   DEPENDS 
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/src
)


## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node 
  src/main_node.cpp
  src/SophiaModelHandler.cpp
  )

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  )
