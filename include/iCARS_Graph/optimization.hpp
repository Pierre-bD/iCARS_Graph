#ifndef OPTIMIZATION_HPP
#define OPTIMIZATION_HPP

#include "iCARS_Graph/utils.hpp"
#include "iCARS_Graph/imuIntegration.hpp"
#include "iCARS_Graph/gnss.hpp"

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
        std::mutex optimizationMutex_;
        std::chrono::time_point<std::chrono::high_resolution_clock> startOpti;
        std::chrono::time_point<std::chrono::high_resolution_clock> endOpti;
        //define noise model for testing with GTSAM data
        bool initFlag = false;
        gtsam::Point3 initialePosition;
        //######//
        //graph 
        //######//
        std::shared_ptr<gtsam::IncrementalFixedLagSmoother> fixedLagSmoother_;
        std::shared_ptr<gtsam::NonlinearFactorGraph> factorsGraph_;
        std::shared_ptr<gtsam::Values> graphValues_;
        gtsam::ISAM2Params isamParams; //ISAM2 parameters
        gtsam::FixedLagSmootherKeyTimestampMap keyTimeStampMap_;
        gtsam::Key stateKey_;
        gtsam::NavState imuPredictedState_;
        geometry_msgs::msg::Pose gnssPose_{};

        //######//
        //factors
        //######//
        
        
        
        public: 

        Optimization();  //Constructor
        bool imuResetFlag = true;
        bool firstOptiFlag = false;
        
        bool useGnssFlag = false;
        gtsam::Key oldKey;
        gtsam::Key newKey;

        
        //######//
        // Publisher / callback
        //######//
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscribeImu;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subLidarOdom;
        rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr subGnss;
        //rclcpp::Subscription< > ; //create  carla vehicle pose node / truth path
        //rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdometry;
       
        rclcpp::CallbackGroup::SharedPtr callbackGroupImu;
        rclcpp::CallbackGroup::SharedPtr callbackGroupOdom;
        rclcpp::CallbackGroup::SharedPtr callbackGroupLidarOdom;
        rclcpp::CallbackGroup::SharedPtr callbackGroupGnss;

        size_t count_;

        //Prior noise model 

        gtsam::noiseModel::Diagonal::shared_ptr priorPoseNoise;
        gtsam::noiseModel::Diagonal::shared_ptr priorVelNoise;
        gtsam::noiseModel::Diagonal::shared_ptr priorBiasNoise;
        gtsam::noiseModel::Diagonal::shared_ptr correctionNoise;
        gtsam::noiseModel::Diagonal::shared_ptr correctionNoise2;
        gtsam::Vector noiseModelBetweenBias;

        //gtsam::noiseModel::Diagonal::shared_ptr

        //auto pose_noise_model = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished());  // rad,rad,rad,m, m, m
        //auto velocity_noise_model = noiseModel::Isotropic::Sigma(3, 0.1);  // m/s
        //auto bias_noise_model = noiseModel::Isotropic::Sigma(6, 1e-3);

        float imuAccBiasN;
        float imuGyrBiasN;


        //Previous State
        gtsam::Pose3 prevPose_;
        gtsam::Vector3 prevVel_;
        gtsam::NavState prevState_;
        gtsam::imuBias::ConstantBias prevBias_;
       
        // Current State
        gtsam::NavState currState;
        gtsam::Key priorKey;
        gtsam::Key key;
        
        
        const auto newStateKey_() {return ++stateKey_;}
        double stateTime_;
        double lastTime_;
        double gnssTimeStamp_;
        std::shared_ptr<IMUintegration> imuIntegrationPtr_ = NULL;
        std::shared_ptr<gnss> gnssPtr_ = NULL;

        // Transformations
        gtsam::Pose3 T_W_O_; //change name
        gtsam::Vector3 I_v_W_I; //change name

        void initGraph(double optiTime, gtsam::Point3& initialGnssPose); //init graph at start-up
        void imuManager(const sensor_msgs::msg::Imu::ConstSharedPtr& imuRaw);
        void gnssManager(const sensor_msgs::msg::NavSatFix::ConstSharedPtr GnssRaw);
        //void lidarOdometry(const nav_msgs::msg::Odometry::SharedPtr odomMsg);
        //void lidarOdomManager(const nav_msgs::msg::Odometry::SharedPtr odomMsg);    
        void addImuFactor(const double time);
        //void addOdometryFactor();
        //void addLidarOdomFactor();
        //void addDualLidarOdomFactor(const nav_msgs::msg::Odometry::SharedPtr odomData);
        void addGnssFactor(const double gpsTime, gtsam::Vector3& gnssPosition);

        
        void optimizeGraph(const double optiTime, gtsam::Key& optiKey); //optimize the graph after adding factors and values
        
    };

}


#endif
