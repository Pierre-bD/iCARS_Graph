#ifndef _OPTIMIZATION_HPP_
#define _OPTIMIZATION_HPP_

#include "utils.hpp"

//########//
// Include gtsam libraries
//########//
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/inference/Key.h>

#include <rclcpp/rclcpp.hpp>


namespace ic_graph {
    class Optimization : public rclcpp::Node
    {
                
        private:
        //rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subImuOdometry;    
        std::mutex lidarMutex_;
        std::chrono::time_point<std::chrono::high_resolution_clock> startOpti;
        std::chrono::time_point<std::chrono::high_resolution_clock> endOpti;
        //define noise model for testing with GTSAM data

        //######//
        //graph 
        //######//
        std::shared_ptr<gtsam::IncrementalFixedLagSmoother> fixedLagSmoother_;
        std::shared_ptr<gtsam::NonlinearFactorGraph> factorsGraph_;
        std::shared_ptr<gtsam::Values> graphValues_;
        gtsam::ISAM2Params isamParams; //ISAM2 parameters
        gtsam::FixedLagSmootherKeyTimestampMap keyTimeStampMap_;
        
        public: 

        Optimization();  //Constructor
        bool imuResetFlag = true;
        bool firstOptiFlag = false;
    
        
        //######//
        // Publisher / callback
        //######//
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscribeImu;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLidarOdom;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subGnss;
        //rclcpp::Subscription< > ; //create  carla vehicle pose node / truth path
       
        rclcpp::CallbackGroup::SharedPtr callbackGroupImu;
        rclcpp::CallbackGroup::SharedPtr callbackGroupOdom;
        rclcpp::CallbackGroup::SharedPtr callbackGroupLidarOdom;
        rclcpp::CallbackGroup::SharedPtr callbackGroupGnss;

        //Prior noise model 
<<<<<<< HEAD
        //auto priorPoseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished());
        //auto priorVelNoise = noiseModel::Isotropic::Sigma(3, 0.1);
        //auto priorBiasNoise = noiseModel::Isotropic::Sigma(6, 1e-3);
=======
        auto priorPoseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished());
        auto priorVelNoise = noiseModel::Isotropic::Sigma(3, 0.1);
        auto priorBiasNoise = noiseModel::Isotropic::Sigma(6, 1e-3);
>>>>>>> main
        auto pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished());  // rad,rad,rad,m, m, m
        auto velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);  // m/s
        auto bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);
        //Define parameters for ISAM2 optimizer
        //gtsam::ISAM2Params isamParams;
        //isamParams.relinearizeTreshold = 0.01; //define proper value
        //isamParams.relinearizeSkip = 1;
        //factorGraph.;

        //Previous State
        gtsam::Pose3 prevPose_;
        gtsam::Vector3 prevVel_;
        gtsam::NavState prevState_;
        gtsam::imuBias::ConstantBias prevBias_;
       
        // Current State
        gtsam::NavState currState;
        gtsam::key priorKey;
        gtsam::key key;

        gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
        //gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
        gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
        gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
        gtsam::noiseModel::Isotropic::shared_ptr priorVelNoise;
        // Transformations
        gtsam::Pose3 T_W_O_; //change name
        gtsam::Vector3 I_v_W_I; //change name


        // Transformations
        gtsam::Pose3 T_W_O_; //change name
        gtsam::Vector3 I_v_W_I; //change name


        void initGraph(); //init graph at start-up
        void optimizeGraph(); //optimize the graph after adding factors and values
        void addIMUFactor(const double time, const Eigen::Vector3d& linearAcc, Eigen::Vector3d& angularVel);
        //void addOdometryFactor();
        void addLidarOdomFactor();
        void addDualLidarOdomFactor();
        void addGnssFactor();
        void imuManager();
        void lidarOdomManager();
        void gnssManager();
        
    };

}


#endif