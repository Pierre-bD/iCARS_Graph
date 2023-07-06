
#ifndef IMUINTEGRATION_HPP
#define IMUINTEGRATION_HPP
#include "iCARS_Graph/utils.hpp"
#include "iCARS_Graph/imu.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <gtsam/navigation/CombinedImuFactor.h>


#include <Eigen/Dense>
#include <Eigen/Core>


#include <boost/shared_ptr.hpp>
#include <boost/optional.hpp>
#include <map>

namespace ic_graph 
{
    class IMUintegration 
    {
        IMUintegration();
        private:
        const double maxTimeDeviation = 0.01;
        const int imuBufferLenght = 10;
        const int keyBufferLenght = 10;

        public:


        // prior IMU parameters
        gtsam::Vector3 priorAccBias = gtsam::Vector3 (0.0, 0.0, 0.0);
        gtsam::Vector3 priorGyrBias = gtsam::Vector3 (0.0, 0.0, 0.0);


        bool imuResetFlag;
        std::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> preintParamsPtr_;
        std::shared_ptr<gtsam::imuBias::ConstantBias> imuPriorBiasPtr_;
        std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imuPreintegrationPtr_;

        gtsam::Vector6 imuData; 
       
        std::shared_ptr<imu::ImuMeasurement> filtered_measurement;
        std::shared_ptr<imu::IntegrationParams> imuNoisePtr_;

        //typedef std::map<double, gtsam::Vector6, std::less<double>, Eigen::aligned_allocator<std::pair<const double, gtsam::Vector6>>> ImuMap;
        //typedef std::map<double, gtsam::Key, std::less<double>, Eigen::aligned_allocator<std::pair<const double, gtsam::Vector6>>> KeyMap;
        
        // define buffer of map type
        //imuMap imuBuffer;  
        //keyMap keyBuffer;
        
        
        //imuPreintegration = new gtsam::PreintegratedCombinedMeasurements(,);
     
        bool initImu(const double grav, const std::string& gravityDir);

        //void addImu2Buffer(const );
        void addImu2Buffer(Eigen::Vector3d& acc, Eigen::Vector3d& vel, const double time);

        void addKey2Buffer(const double time, gtsam::Key key);

        boost::optional<imu::ImuMeasurement>interpolateImu(
            const imu::ImuMeasurement& meas_a, 
            const imu::ImuMeasurement& meas_b, 
            const double timestamp);

        std::map<const double, imu::ImuMeasurement> imuMap_;
        std::map<const double, gtsam::Key> keyBuffer_;
        bool resetImu(const gtsam::imuBias::ConstantBias& bias);
        void addMeasurement(const imu::ImuMeasurement& imu_measurement, double& last_added_measurement_time);
        void updateIntegration(const double start_time, const double end_time, const gtsam::imuBias::ConstantBias& prevBias);
        void getClosestKey(const double timek, gtsam::Key& closestKey, double& time2Graph);

        // see if I put the following in this file or in optimization or loc_graph

        //gtsam::noiseModel::Diagonal::shared_ptr
    };
}


#endif
