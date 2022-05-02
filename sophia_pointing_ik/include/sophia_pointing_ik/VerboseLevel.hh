#ifndef HATP_VERBOSE_LEVEL_HH_
#define HATP_VERBOSE_LEVEL_HH_

#include <iostream>
#include <fstream>

#include <ros/console.h>

//Handle verbose level
#define VERBOSE_1 if(VERBOSE_LEVEL>=1) ROS_INFO
#define VERBOSE_2 if(VERBOSE_LEVEL>=2) ROS_INFO
#define VERBOSE_3 if(VERBOSE_LEVEL>=3) ROS_INFO
#define VERBOSE_4 if(VERBOSE_LEVEL>=4) ROS_INFO
#define VERBOSE_5 if(VERBOSE_LEVEL>=5) ROS_INFO//This is usually the debug level for yifan

#endif
