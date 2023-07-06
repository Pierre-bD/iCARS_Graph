
#include "iCARS_Graph/imuIntegration.hpp"
#include "iCARS_Graph/utils.hpp"
#include "iCARS_Graph/imu.hpp"

using namespace std::chrono_literals;
namespace ic_graph {

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
    preintParamsPtr_ = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(grav);
  }
  else 
  {
    preintParamsPtr_ = gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedD(grav);
  }

  preintParamsPtr_->setAccelerometerCovariance(gtsam::Matrix33::Identity(3,3) * imuNoisePtr_->imuAccNoise);
  preintParamsPtr_->setIntegrationCovariance(gtsam::Matrix33::Identity (3, 3) * imuNoisePtr_->integrationNoise); 

  preintParamsPtr_->setGyroscopeCovariance(gtsam::Matrix33::Identity(3, 3) * imuNoisePtr_->imuGyrNoise);

  // Set bias
  preintParamsPtr_->setBiasAccCovariance(gtsam::Matrix33::Identity(3, 3) * imuNoisePtr_->imuAccBias); ///finish this///
  preintParamsPtr_->setBiasOmegaCovariance(gtsam::Matrix33::Identity(3, 3) * imuNoisePtr_->imuGyrBias); ///finish this///
  preintParamsPtr_->setBiasAccOmegaInit(gtsam::Matrix66::Identity(6, 6) * 1e-5); ///finish this///
  
  //init prior bias
  imuPriorBiasPtr_ = std::make_shared<gtsam::imuBias::ConstantBias>(priorAccBias, priorGyrBias);
  //init pointer
  imuPreintegrationPtr_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(preintParamsPtr_, *imuPriorBiasPtr_);

  std::cout <<"Imu initialised" << std::endl;
  return true;
}

void IMUintegration::addImu2Buffer(Eigen::Vector3d& acc, Eigen::Vector3d& vel, const double time)
{
  filtered_measurement->accel = acc;
  filtered_measurement->angular_vel = vel;
  //imu::ImuMeasurement filtered_measurement;
  //filtered_measurement.accel = acc;
  //filtered_measurement.angular_vel = vel;
  //filtered_measurement.timestamp = time;
  imuMap_.emplace(time, *filtered_measurement);
  if (imuMap_.size() > imuBufferLenght)
  {
    imuMap_.erase(imuMap_.begin());
  }
}
  
void IMUintegration::addKey2Buffer(const double time, gtsam::Key key)
{
  keyBuffer_[time] = key;
  keyBuffer_.emplace(time, key);

  if (keyBuffer_.size() > keyBufferLenght) //erase the first data when buffer raise his lenght limit
  {
    keyBuffer_.erase(keyBuffer_.begin());
  }

}

void IMUintegration::getClosestKey(const double timek, gtsam::Key& closestKey, double& time2Graph)
{
    std::_Rb_tree_iterator<std::pair<const double, gtsam::Key>> upperIterator;
    upperIterator = keyBuffer_.upper_bound(timek);
    auto lowerIterator = upperIterator;
    --lowerIterator;

    time2Graph = std::abs(timek - lowerIterator->first) < std::abs(upperIterator->first -timek) ? lowerIterator->first : upperIterator->first;
    closestKey = std::abs(timek - lowerIterator->first) < std::abs(upperIterator->first -timek) ? lowerIterator->second : upperIterator->second;
    double timeDeviation = time2Graph - timek;
}

bool IMUintegration::resetImu(const gtsam::imuBias::ConstantBias& bias)
  {
    imuPreintegrationPtr_->resetIntegrationAndSetBias(bias);
    imuResetFlag = true;
    return imuResetFlag;

  }

boost::optional<imu::ImuMeasurement> IMUintegration::interpolateImu(const imu::ImuMeasurement& meas_a,
                                                                const imu::ImuMeasurement& meas_b,
                                                                const double timestamp)
{
  const double alpha = (timestamp - meas_a.timestamp) / (meas_b.timestamp - meas_a.timestamp);
  const Eigen::Vector3d interpolated_acc = (1.0 - alpha) * meas_a.accel + alpha * meas_b.accel;
  const Eigen::Vector3d interpolated_angular_vel = (1.0 - alpha) * meas_a.angular_vel + alpha * meas_b.angular_vel;

  return imu::ImuMeasurement(interpolated_acc, interpolated_angular_vel, timestamp);
}
void IMUintegration::addMeasurement(const imu::ImuMeasurement& imu_measurement, double& last_added_measurement_time)
{
  const double dt = imu_measurement.timestamp - last_added_measurement_time; 
  if (dt == 0)
  {
    std::cout<<"Error9 : Add measurement, Timestamp difference 0"<<std::endl;
  }
  imuPreintegrationPtr_->integrateMeasurement(imu_measurement.accel,imu_measurement.angular_vel, dt);
  last_added_measurement_time = imu_measurement.timestamp;
}

void IMUintegration::updateIntegration(const double start_time, const double end_time, const gtsam::imuBias::ConstantBias& prevBias)
  {
    if (imuMap_.size()<2)
    {
      std::cout<<"ERROR10 : Integrate Imu Measurement, less than 2 measurements available."<<std::endl;
      return;
    }
    // Step 1 : integrate imu measurements
    imuPreintegrationPtr_->resetIntegrationAndSetBias(prevBias);
    // Step2 
    auto measurement_it = imuMap_.upper_bound(start_time);
    //auto currentIter = measures.begin();
    //auto prevIter = currentIter;
    double last_added_measurement_time = start_time;
    int num_measurement_added = 0;
    while (measurement_it !=imuMap_.cend() && measurement_it->first <= end_time)
    {
      addMeasurement(measurement_it->second, last_added_measurement_time);
      ++num_measurement_added;
      ++measurement_it;
    }
    
    if(last_added_measurement_time != end_time)
    {
        const auto interpolated_measurement = interpolateImu(std::prev(measurement_it)->second, measurement_it->second, end_time);
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
