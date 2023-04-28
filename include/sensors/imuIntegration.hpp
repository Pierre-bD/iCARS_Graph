
#ifndef _IMUINTEGRATION_HPP_
#define _IMUINTEGRATION_HPP_

#include "utils.hpp"
#include <sensor_msgs/msg/imu.hpp>


#include <Eigen/Dense>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>


namespace ic_graph
{

    
    class IMUintegration {

        public:

        // param definition IMU
        double   imuAccNoise = 0.0003924;
        double imuGyrNoise = 0.000205689024915;
        double imuAccBias = 0.004905;
        double imuGyrBias = 0.000001454441043;
     

        imuTopic = "";

        gtsam::Vector6 imuData;

        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscribeImu;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publishImuOdometry;
        imuCallback = create_callbak_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions options;
        options.callback_group = imuCallback;
       

        void initImu();

        void addImu2Buffer(double tsp, double accx, double accy, double accz, double gyrox, double gyroy, double gyroz);
    
        void addKey2Buffer(double tsp, gtsam::Key key);

        void imuManager();

        private:
        
        typedef std::map<double, gtsam::Vector6, std::less<double>, Eigen::aligned_allocator<std::pair<const double, gtsam::Vector6>>> ImuMap;
        typedef std::map<double, gtsam::Key, std::less<double>, Eigen::aligned_allocator<std::pair<const double, gtsam::Vector6>>> KeyMap;
        
        // define buffer of map type
        imuMap imuBuffer;  
        keyMap keyBuffer;
        std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imuPreintegration;
        imuPreintegration = new gtsam::PreintegratedCombinedMeasurements(,);
        private: 

        // see if I put the following in this file or in optimization or loc_graph

        gtsam::noiseModel::Diagonal::shared_ptr
    };
}


#endif