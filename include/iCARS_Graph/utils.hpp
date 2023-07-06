/* ----------------------------------------------------------------------------
    Copyright (c) 2023, Pierre Baptiste Demonceaux
    All Rights Reserved.
    This file is released under the "BSD-2-Clause License".
    
    See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#ifndef UTILS_HPP
#define UTILS_HPP


#include <iostream>
#include <cmath>
#include <algorithm>
#include <map>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>
#include <string>
#include <functional>
#include <queue>
#include <deque>
#include <boost/optional.hpp>

#include <gtsam/inference/Symbol.h>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>


// Ros2 include
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/convert.h>


// pcl include 
//#include <pcl/point_cloud.h>
//#include <pcl/point_types.h>


// 

// rad to deg <--> deg to rad conversion

//namespace ic_graph 
//{
 //   public:
//    imuTopic = "imuData";
//}


#endif