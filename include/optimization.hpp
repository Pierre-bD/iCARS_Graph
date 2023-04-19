
#include "utils.hpp"
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>
#include <gtsam/
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/ImuFactor.h>

#include <rclcpp/rclcpp.hpp>

#ifndef OPTIMIZATION_HPP_
#define OPTIMIZATION_HPP_

namespace ic_graph {

    class optimization{

        private:
        gtsam::



        public: 

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subImuOdometry;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub;

        gtsam::Pose3 prevPose_;
        gtsam::Vector3 prevVel_;
        gtsam::NavState prevState_;
        gtsam::imuBias::ConstantBias prevBias_;

        void addIMU();
        void addOdometry();
        void addLidarOdom();
        void addGNSSposition();
        

        protected: 

        void initGraph(); //init graph at start-up
        void optimizationgraph(); //optimize the graph after adding factors and values
 

    };



}


#endif