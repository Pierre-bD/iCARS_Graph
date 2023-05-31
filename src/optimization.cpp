#include"optimization.hpp"
#include"imuIntegration.hpp"
#include"utils.hpp"
#include <ros

namespace ic_graph {

Optimization::Optimization() : Node("optimization"), count_(0)
{
    isamParams.relinearizeTreshold = 0.01; //define proper value
    isamParams.relinearizeSkip = 1;
    factorsGraph = std::make_shared<gtsam::NonlinearFactorGraph>();
    graphValues = std::make_shared<gtsam::Values>();
    bool initFlag = false;

    //Define prior noise for first optimisation
    //priorPoseNoise = gtsam::noiseModel::Diagonal::sigmas();
    //priorVelNoise = gtsam::noiseModel::Diagonal::sigma();
    //priorBiasNoise = gtsam::noiseModel::Diagonal::sigma();
    
    // create callback group
    callbackGroupImu = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callbackGroupOdom = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callbackGroupLidarOdom = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto imuOpt = rclcpp::SubscriptionOptions();
    imuOpt.callback_group = callbackGroupImu;
    auto odomOpt = rclcpp::SubscriptionOptions();
    odomOpt.callback_group = callbackGroupOdom;
    auto lidarOdomOpt = rclcpp::SubscriptionOptions();
    lidarOdomOpt.callback_group = callbackGroupLidarOdom;
    //Create callback for GNSS


    // create publisher, subscription                   //non definitive name//
    subscribeImu = create_subscription<sensor_msgs::msg::Imu>("carla/odometry/imu", , std::bind(&IMUintegration::addImu2Buffer, this, std::placeholders::_1), imuOpt); ///check for qos param
    subLidarOdom = create_subscription<nav_msgs::msg::Odometry>("ic_graph/odometry/lidar", , std::bind(&Optimization::lidarOdomManager, this, std::placeholders::_1), lidarOdomOpt);
    //subGnssData = create_subscription<>("carla/odometry/gnss", , std::bind(&Optimization::));
    //subOdometry = create_subscription<nav_msgs::msg::Odometry>("", , std::bind(&Optimization::odomManager, this, std::placeholders::_1), odomOpt);
    //publishPose = create_publisher<>
}
       
void optimization::initGraph() // graph initialization 
{
    ///void reset optimizarion ??

    factorsGraph->addPrior(X(,priorPose,pose_noise_model));
    factorsGraph->addPrior(V(,priorVelocity,velocity_noise_model));
    factorsGraph->addPrior(B(,priorImuBias,bias_noise_model));
    
    // Set initial pose
    prevPose =  ; //-->FINISH THIS
    gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose, priorPoseNoise);
    factorsGraph.add(priorPose);

    // Set initial velocity
    prevVel= gtsam::Vector3(0,0,0);
    gtsam::PriorFactor<gtsam::Vector3> priorVelocity(V(0), prevVel, priorVelNoise);
    factorsGraph->add(priorVelocity);

    // Set initial bias
    prevBias = gtsam::imuBias::ConstantBias();
    gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias, priorBiasNoise);
    factorsGraph->add(priorBias);

    // add values to the graph
    graphValues->insert(X(0), prevPose);
    graphValues->insert(V(0), prevVel);
    graphValues->insert(B(0), prevBias);

    // optimize once
    isam2->update(factorsGraph, graphValues); //CHECK THE DECLARATION OF OPTIMIZER
    factorsGraph->resize(0);
    graphValues->clear();

    key=1;
    initFlag = true;
    return ;
}
void optimization::imuManager()
{
 // see if it is useful
}
void optimization::addIMUFactor(const double time, const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel)
{
    
    factorsGraph->add(imuFactor_);
}

void optimization::addOdometryFactor()
{
    factorsGraph->add(odometryFactor_);
}

void optimization::addLidarOdomFactor()
{
    factorsGraph->add(lidarFactor_);
}

void optimization::addGNSSpositionFactor()
{
    factorsGraph->add(gnssFactor_);
}
    //void optimization::addGNSSYawFactor();

void optimization::optimizeGraph() 
{
    if (initFlag == false)
    {
        initGraph();
    }
    //if (imuMeasFlag) 
    //{
     //   addIMUFactor();
    //}
    // Update the IMU preintegrator
    IMUintegration::updateIntegration();

    startOpti = std::chrono::high_resolution_clock::now();
    
    gtsam::NavState optimizedState = updateGraph();
    optimizedState.pose();
    
    
    currState = 

    isam2->update(factorsGraph, valuesGraph);


    endOpti = std::chrono::high_resolution_clock::now();

}



}


int main(int argc, char** argv){

    //init optimization once (prior pose, vel bias, params....)    


    rclcpp::init(argc, argv);
    rclcpp::NodeOptions nodeParam;
    nodeParam.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor optiExec; //or use single thread, will see how many nodes I define
    
    auto poseOptimization = std::make_shared<Optimization>(nodeParam);

    optiExec.add_node(poseOptimization);

    optiExec.spin();

    rclcpp::shutdown();

    return 0;
}