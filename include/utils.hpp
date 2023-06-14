/* ----------------------------------------------------------------------------
    Copyright (c) 2023, Pierre Baptiste Demonceaux
    All Rights Reserved.
    This file is released under the "BSD-2-Clause License".
    
    See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#ifndef _UTILS_HPP_
#define _UTILS_HPP_


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
#include <boost/>

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

#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/string.hpp>

// pcl include 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


// 

// rad to deg <--> deg to rad conversion

namespace ic_graph 
{
    public:
    imuTopic = "imuData";
}

#endif