
#ifndef _IMUINTEGRATION_HPP_
#define _IMUINTEGRATION_HPP_
#include "utils.hpp"
#include "sensors/imu.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <gtsam/navigation/CombinedImuFactor.h>


#include <Eigen/Dense>


#include <boost/shared_ptr.hpp>
#include <boost/optional.hpp>
#include <map>

namespace imu_integration
{
    class IMUintegration 
    {
        IMUintegration();
        public:


        // prior IMU parameters
        gtsam::Vector3 priorAccBias = gtsam::Vector3 (0.0, 0.0, 0.0);
        gtsam::Vector3 priorGyrBias = gtsam::Vector3 (0.0, 0.0, 0.0);


        bool imuResetFlag;
        std::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> preintParams_;
        std::shared_ptr<gtsam::imuBias::ConstantBias> imuPriorBias_;
        std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imuPreintegrationPtr_;

        gtsam::Vector6 imuData; 
       


        //typedef std::map<double, gtsam::Vector6, std::less<double>, Eigen::aligned_allocator<std::pair<const double, gtsam::Vector6>>> ImuMap;
        //typedef std::map<double, gtsam::Key, std::less<double>, Eigen::aligned_allocator<std::pair<const double, gtsam::Vector6>>> KeyMap;
        
        // define buffer of map type
        imuMap imuBuffer;  
        keyMap keyBuffer;
        
        
        //imuPreintegration = new gtsam::PreintegratedCombinedMeasurements(,);
        private: 
     
        bool initImu(const double grav, const std::string& gravityDir);

        //void addImu2Buffer(const );
        void addImu2Buffer(Eigen::Vector3d& acc, Eigen::Vector3d& vel, const double time);

        void addKey2Buffer(double tsp, gtsam::Key key);

        boost::optional<imu_integration::ImuMeasurement>interpolateImu(
            const imu_integration::ImuMeasurement& meas_a, 
            const imu_integration::ImuMeasurement& meas_b, 
            const double timestamp);

        std::map<const double, imu_integration::ImuMeasurement> imuMap_;
        bool resetImu(const gtsam::imuBias::ConstantBias& bias);
        void addMeasurement(const imu_integration::ImuMeasurement& imu_measurement, double& last_added_measurement_time);
        void updateIntegration(const double start_time, const double end_time);

        // see if I put the following in this file or in optimization or loc_graph

        //gtsam::noiseModel::Diagonal::shared_ptr
    };
}


#endif


#endif