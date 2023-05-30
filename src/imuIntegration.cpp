/* ----------------------------------------------------------------------------
    Copyright (c) 2023, Pierre Baptiste Demonceaux
    All Rights Reserved.
    This file is released under the "BSD-2-Clause License".
    
    See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#include "imuIntegration.hpp"
#include "utils.hpp"

using namespace std::chrono_literals;
namespace ic_graph {

//IMUintegration::IMUintegration()
//{
//  imuResetFlag = true;
  // create publisher, subscription
//  subscribeImu = create_subscription<sensor_msgs::msg::Imu>("imuData", std::bind(&addImu2Buffer,));
//}

using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)

  
bool IMUintegration::initImu(const double grav, const std::string& gravityDir)
{
  if (gravityDir == "up")
  {
    preintParamsPtr = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(grav);
  }
  else 
  {
    preintParamsPtr = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD(grav);
  }

  preintParamsPtr->setAccelerometerCovariance(gtsam::Matrix33::Identity(3,3) * pow(imuAccNoise,2));
  preintParamsPtr->setINtegrationCovariance(gtsam::Matrix33::Identity (3, 3) * pow(imuGyrNoise,2)); ///finish this///

  preintParamsPtr->setGyroscopeCovariance(gtsam::Matrix33::Identity(3, 3) * 1e-8); ///finish this///

  // Set bias
  preintParamsPtr->setBiasAccCovariance(gtsam::Matrix33::Identity(3, 3) * pow(imuAccBias)); ///finish this///
  preintParamsPtr->setBiasOmegaCovariance(gtsam::Matrix33::Identity(3, 3) * pow(imuGyrBias)); ///finish this///
  preintParamsPtr->setBiasAccOmegaInt(gtsam::Matrix66::Identity(6, 6) * 1e-5); ///finish this///

  //init prior bias
  imuPriorBiasPtr = std::make_shared<gtsam::imuBias::ConstantBias>(priorAccBias, priorGyrBias);
  //init pointer
  imuPreintegrationPtr = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(preintParamsPtr, *imuPriorBiasPtr);

  std::cout <<"Imu initialised" << std::endl;
  return true;
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

  bool IMUintegration::resetImu()
  {
    imuPreintegration->resetIntegrationAndSetBias();
    imuResetFlag = true;
    return imuRese

  }

  void IMUintegration::updateIntegration()
  {
    imuPreintegrationPtr->resetIntegrationAndSetBias(prev_bias); //change variable
    for ()
    {
      double dt = current - prev; // to do
      imuPreintegrationPtr->integrateMeasurement(gtsam::Vector3(), gtsam::Vector3(), dt);
    }
    
   
  }

//publishImuOdometry = create_publisher<nav_msgs::msg::Odometry>(imuOdomTopic,);

//////
// Replace publisher by server-client system for trigger preintegration
//////



// publish imu odometry through topic
//publishImuOdometry->publish();

}

int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions nodeParam;
    nodeParam.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor imuExec; // one executor for the subscriber + one for the server/client 
    auto imuPreint = std::make_shared<IMUintegration>();
    
    imuExec.add_node(imuPreint);

    imuExec.spin();
    rclcpp::shutdown();
    return 0;
}