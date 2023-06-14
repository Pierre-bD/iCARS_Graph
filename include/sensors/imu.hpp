#ifndef IMU_INTEGRATION_IMU_HPP
#define IMU_INTEGRATION_IMU_HPP


#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <Eigen/Core>
#include <string>

namespace imu_integration{

struct IntegrationParams
{
    gtsam::Vector3 gravity;
    gtsam::Pose3 bodyImu;
     // IMU parameters definition 
    double imuAccNoise // 0.0003924;
    double imuGyrNoise // 0.000205689024915;
    double imuAccBias // 0.004905;
    double imuGyrBias // 0.000001454441043;

};

struct ImuMeasurement
{
  explicit ImuMeasurement(const sensor_msgs::msg::imu& imu_msg) 
  {
    accel.x() = imu_msg.linear_acceleration.x;
    accel.y() = imu_msg.linear_acceleration.y;
    accel.z() = imu_msg.linear_acceleration.z;
    angular_vel.x() = imu_msg.angular_velocity.x;
    angular_vel.y() = imu_msg.angular_velocity.y;
    angular_vel.z() = imu_msg.angular_velocity.z;
    // Ros headers are stored as seconds and nanoseconds
    timestamp = imu_msg.header.stamp.sec + 1e-9 * imu_msg.header.stamp.nsec;
  }
    ImuMeasurement(const Eigen::Vector3d& accel, const Eigen::Vector3d& angular_vel, double timestamp) 
    : timestamp(timestamp), accel(accel), angular_vel(angular_vel){}

    Eigen::Vector3d accel;
    Eigen::Vector3d angular_vel;
    double timestamp;
};
}
#endif