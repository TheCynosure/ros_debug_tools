//
// Created by jack on 2/28/20.
//

#ifndef ROS_DEBUG_TOOLS_DEBUG_MAIN_H
#define ROS_DEBUG_TOOLS_DEBUG_MAIN_H

/*
 * Define USING ROS DEBUG if you want to use ros features like publishers
 * and display things in rviz.
 */

#ifdef USING_ROS_DEBUG
#include "ROS_DEBUG_TOOLS/ros_debug.h"
#else
#include "ROS_DEBUG_TOOLS/cimg_debug.h"
#endif

#endif //ROS_DEBUG_TOOLS_DEBUG_MAIN_H
