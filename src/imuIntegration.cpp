/* ----------------------------------------------------------------------------
    Copyright (c) 2023, Pierre Baptiste Demonceaux
    All Rights Reserved.
    This file is released under the "BSD-2-Clause License".
    
    See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#include "imuIntegration.hpp"
#include "utils.hpp"

namespace ic_graph {
    
using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)

void test::imuCallback_(const sensor_msgs::Imu::ConstPtr& imuMsgPtr) {
  static bool firstCallbackFlag__ = true;
  if (firstCallbackFlag__) {
    firstCallbackFlag__ = false;
  }
  Eigen::Vector3d linearAcc(imuMsgPtr->linear_acceleration.x, imuMsgPtr->linear_acceleration.y, imuMsgPtr->linear_acceleration.z);
  Eigen::Vector3d angularVel(imuMsgPtr->angular_velocity.x, imuMsgPtr->angular_velocity.y, imuMsgPtr->angular_velocity.z);
  graph_msf::GraphMsfInterface::addImuMeasurementAndPublishState_(linearAcc, angularVel, imuMsgPtr->header.stamp.toSec());
}
  
void IMUintegration::initImu()
{
imuParams = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU();

imuParams->setAccelCovar(gtsam::Matrix33::Identity(3,3)*.....)

}
void IMUintegration::addImu2Buffer(double tsp, double accx, double accy, double accz, double gyrox, double gyroy, double gyroz)
  {
    imuData << accx, accz, accz, gyrox, gyroy, gyroz;
    imuBuffer[tsp] = imuData;

    if (imuBuffer.size() > bufferLenght) //erase the first data when buffer raise his lenght limit
      {
      imuBuffer.erase(imuBuffer.begin());
      }

  }
  
  void IMUintegration::addKey2Buffer(double tsp, gtsam::Key key)
  {
    imuBuffer[tsp] = key;

    if (keyBuffer.size() > bufferLenght) //erase the first data when buffer raise his lenght limit
      {
      keyBuffer.erase(keyBuffer.begin());
      }

  }



// create publisher, subscription
subscribeImu = create_subscription<sensor_msgs::msg::Imu>(imuTopic, ,);
publishImuOdometry = create_publisher<nav_msgs::msg::Odometry>(imuOdomTopic,);

// publish imu odometry through topic
publishImuOdometry->publish();

}


int main(int argc, char** argv){



    rclcpp::init(argc, argv);
    rclcpp::NodeOptions nodeParam;
    nodeParam.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor imuExec; //or use single thread, will see how many nodes I define
    auto imuPreint = std::make_shared<IMUintegration>();
    
    imuExec.add_node(imuPreint);

    imuExec.spin();

    return 0;
}