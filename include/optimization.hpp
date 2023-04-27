#ifndef _OPTIMIZATION_HPP_
#define _OPTIMIZATION_HPP_

#include "utils.hpp"
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>

#include <rclcpp/rclcpp.hpp>


namespace ic_graph {

    class optimization{
        optimization () //Constructor
        
        private:
        //gtsam::

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subImuOdometry;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub;
        //Previous State
        gtsam::Pose3 prevPose;
        gtsam::Vector3 prevVel;
        gtsam::NavState prevState;
        gtsam::imuBias::ConstantBias prevBias;
        // Current State
        gtsam::NavState currState;
        
        std::chrono::time_point<std::chrono::high_resolution_clock> startOpti;
        std::chrono::time_point<std::chrono::high_resolution_clock> endOpti;

        public: 
        std::shared_ptr<gtsam::IncrementalFixedLagSmoother> fixedLagSmoother;
        std::shared_ptr<gtsam::NonlinearFactorGraph> factorGraph;
        void addIMUFactor(const double time, const Eigen::Vector3d& linearAcc, Eigen::Vector3d& angularVel);
        void addOdometryFactor();
        void addLidarOdomFactor();
        void addGNSSpositionFactor();
        void addGNSSYawFactor();
        
        //Define parameters for ISAM2 optimizer
        gtsam::ISAM2Params isamParams;
        isamParams.relinearizeTreshold = 0.01; //define proper value
        isamParams.relinearizeSkip = 1;
        factorGraph.;

        //Define prior noise for first optimisation
        priorPoseNoise = gtsam::noiseModel::Diagonal::sigmas();
        priorVelNoise = gtsam::noiseModel::Diagonal::sigma();
        priorBiasNoise = gtsam::noiseModel::Diagonal::sigma();

        //Check with gtsam example
        std::shared_ptr<gtsam::PreintegrationParams> preintParams = gtsam::PreintegrationParams::MakeSharedU(); 
        preintParams->accelerometerCovariance = gtsam::Matrix33::Identity(3,3) * pow(,2) ;
        preintParams->gyroscopeCovariance = gtsam::Matrix33::Identity(3,3) * pow(,2);
        preintParams->integrationCovariance = gtsam::Matrix33::Identity(3,3) * pow(,2);

        protected: 

        void initGraph(); //init graph at start-up
        void optimizationgraph(); //optimize the graph after adding factors and values
        

    };



}


#endif