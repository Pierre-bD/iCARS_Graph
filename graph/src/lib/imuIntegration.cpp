
#include "graph/imuIntegration.hpp"
#include "graph/utils.hpp"

using namespace std::chrono_literals;
namespace imu_integration {

IMUintegration::IMUintegration(){}
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
  preintParamsPtr->setIntegrationCovariance(gtsam::Matrix33::Identity (3, 3) * pow(imuGyrNoise,2)); ///finish this///

  preintParamsPtr->setGyroscopeCovariance(gtsam::Matrix33::Identity(3, 3) * 1e-8); ///finish this///

  // Set bias
  preintParamsPtr->setBiasAccCovariance(gtsam::Matrix33::Identity(3, 3) * pow(imuAccBias)); ///finish this///
  preintParamsPtr->setBiasOmegaCovariance(gtsam::Matrix33::Identity(3, 3) * pow(imuGyrBias)); ///finish this///
  preintParamsPtr->setBiasAccOmegaInt(gtsam::Matrix66::Identity(6, 6) * 1e-5); ///finish this///

  //init prior bias
  imuPriorBiasPtr = std::make_shared<gtsam::imuBias::ConstantBias>(priorAccBias, priorGyrBias);
  //init pointer
  imuPreintegrationPtr_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(preintParamsPtr, *imuPriorBiasPtr);

  std::cout <<"Imu initialised" << std::endl;
  return true;
}

//void IMUintegration::convertImu()
//{
  //imuData = *imuRaw;
    
   // acc = extRot * acc;
   // imu.
    

    
    //gyro = exRot * gyro;
    //imu.
//}
/*void IMUintegration::addImu2Buffer(double tsp, double accx, double accy, double accz, double gyrox, double gyroy, double gyroz)
  {
    imuData << accx, accz, accz, gyrox, gyroy, gyroz;
    imuBuffer[tsp] = imuData;

    if (imuBuffer.size() > bufferLenght) //erase the first data when buffer raise his lenght limit
      {
      imuBuffer.erase(imuBuffer.begin());
      }

  }*/
void IMUintegration::addImu2Buffer(Eigen::Vector3d& acc, Eigen::Vector3d& vel, const double time)
{
  const imu_integration::ImuMeasurement filtered_measurement;
  filtered_measurement.accel = acc;
  filtered_measurement.angular_vel = vel;
  filtered_measurement.timestamp = time;
  imuMap_.emplace(filtered_measurement->timestamp, *filtered_measurement);  
}
  
void IMUintegration::addKey2Buffer(double tsp, gtsam::Key key)
  {
    imuBuffer[tsp] = key;

    if (keyBuffer.size() > bufferLenght) //erase the first data when buffer raise his lenght limit
      {
      keyBuffer.erase(keyBuffer.begin());
      }

  }

bool IMUintegration::resetImu(const gtsam::imuBias::ConstantBias& bias)
  {
    imuPreintegration->resetIntegrationAndSetBias(bias);
    //imuResetFlag = true;
    //return imuResetFlag;

  }

boost::optional<imu_integration::ImuMeasurement> IMUintegration::interpolateImu(const imu_integration::ImuMeasurement& meas_a,
                                                                const imu_integration::ImuMeasurement& meas_b,
                                                                const double timestamp)
{
  const double alpha = (timestamp - meas_a.timestamp) / (meas_b.timestamp - meas_a.timestamp);
  const Eigen::Vector3d interpolated_acc = (1.0 - alpha) * meas_a.accel + alpha * meas_b.accel;
  const Eigen::Vector3d interpolated_angular_vel = (1.0 - alpha) * meas_a.angular_vel + alpha * meas_b.angular_vel;

  return imu_integration::ImuMeasurement(interpolated_acc, interpolated_angular_vel, timestamp);
}
void IMUintegration::addMeasurement(const imu_integration::ImuMeasurement& imu_measurement, double& last_added_measurement_time)
{
  const double dt = imu_measurement.timestamp - last_added_measurement_time; 
  if (dt == 0)
  {
    std::cout<<"Error9 : Add measurement, Timestamp difference 0"<<std::endl;
  }
  imuPreintegrationPtr_->integrateMeasurement(imu_measurement.accel,imu_measurement.angular_vel, dt);
  last_added_measurement_time = imu_measurement.timestamp;
}

void IMUintegration::updateIntegration(const double start_time, const double end_time)
  {
    if (imuMap_.size()<2)
    {
      std::count<<"ERROR10 : Integrate Imu Measurement, less than 2 measurements available."<<std::endl;
      return
    }
    // Step 1 : integrate imu measurements
    imuPreintegrationPtr_->resetIntegrationAndSetBias(prev_bias);
    // Step2 
    auto measurement_it = imuMap_.upper_bound(start_time);
    //auto currentIter = measures.begin();
    //auto prevIter = currentIter;
    double last_added_measurement_time = start_time;
    int num_measurement_added = 0;
    for (measurement_it !=imuMap_.cend() && measurement_it->first <= end_time, ++measurement_it)
    {
      addMeasurement(measurement_it->second, last_added_measurement_time);
      ++num_measurement_added;
    }
    
    if(last_added_measurement_time != end_time)
    {
        const auto interpolated_measurement = imu_integration::interpolateImu(std::prev(measurement_it)->second, measurement_it->second, end_time);
        addMeasurement(*interpolated_measurement, last_added_measurement_time);
        ++num_measurement_added;
    }
   std::cout<<"IntegrateImuMeasurement : Num imu measurement integrated: " << num_measurement_added <<std::endl;
  }

//publishImuOdometry = create_publisher<nav_msgs::msg::Odometry>(imuOdomTopic,);

//////
// Replace publisher by server-client system for trigger preintegration
//////



// publish imu odometry through topic
//publishImuOdometry->publish();

}