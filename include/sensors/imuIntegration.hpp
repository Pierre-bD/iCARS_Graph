
#ifndef _IMUINTEGRATION_HPP_
#define _IMUINTEGRATION_HPP_

#include "utils.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include "srv/TrigIntegration.srv"


#include <Eigen/Dense>

#include <gtsam/inference/Symbol.h>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>


namespace ic_graph
{

    
    class IMUintegration {

        public:

        // IMU parameters definition 
        double imuAccNoise = 0.0003924;
        double imuGyrNoise = 0.000205689024915;
        double imuAccBias = 0.004905;
        double imuGyrBias = 0.000001454441043;
        // prior IMU parameters
        gtsam::Vector3 priorAccBias = gtsam::Vector3 (0.0, 0.0, 0.0);
        gtsam::Vector3 priorGyrBias = gtsam::Vector3 (0.0, 0.0, 0.0);


        bool imuResetFlag;
        boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> preintParams_;
        std::shared_ptr<gtsam::imuBias::ConstantBias> imuPriorBias_;
        std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imuPreintegration_;

        gtsam::Vector6 imuData; 
       


        typedef std::map<double, gtsam::Vector6, std::less<double>, Eigen::aligned_allocator<std::pair<const double, gtsam::Vector6>>> ImuMap;
        typedef std::map<double, gtsam::Key, std::less<double>, Eigen::aligned_allocator<std::pair<const double, gtsam::Vector6>>> KeyMap;
        
        // define buffer of map type
        imuMap imuBuffer;  
        keyMap keyBuffer;
        
        
        //imuPreintegration = new gtsam::PreintegratedCombinedMeasurements(,);
        private: 

        bool initImu();

        void addImu2Buffer(double tsp, double accx, double accy, double accz, double gyrox, double gyroy, double gyroz);
    
        void addKey2Buffer(double tsp, gtsam::Key key);

        void imuManager();

        bool resetImu();

        void updateIntegration();

        // see if I put the following in this file or in optimization or loc_graph

        gtsam::noiseModel::Diagonal::shared_ptr
    };
}


#endif