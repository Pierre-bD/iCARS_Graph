#include"optimization.hpp"
#include"imu_integration/imu_Integration.hpp"
#include"utils.hpp"
#include <Eigen

namespace ic_graph {

Optimization::Optimization() : Node("optimization")
{
    isamParams.relinearizeTreshold = 0.01; //define proper value
    isamParams.relinearizeSkip = 1;
    double fixedLag = 2.0;
    priorKey = 0;
    factorsGraph_ = std::make_shared<gtsam::NonlinearFactorGraph>();
    graphValues_ = std::make_shared<gtsam::Values>();
    fixedLagSmoother_ = std::make_shared<gtsam::IncrementalFixedLagSmoother>(fixedLag,isamParams); 
    imuPreintegration_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imuparams, *imuPriorBias_);
    bool initFlag = false;
    bool useGnssFlag = false;
    //Define prior noise for first optimisation
    //priorPoseNoise = gtsam::noiseModel::Diagonal::sigmas();
    //priorVelNoise = gtsam::noiseModel::Diagonal::sigma();
    //priorBiasNoise = gtsam::noiseModel::Diagonal::sigma();
    
    // create callback group
    callbackGroupImu = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callbackGroupOdom = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callbackGroupLidarOdom = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    callbackGroupGnss = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    auto imuOpt = rclcpp::SubscriptionOptions();
    imuOpt.callback_group = callbackGroupImu;
    auto odomOpt = rclcpp::SubscriptionOptions();
    odomOpt.callback_group = callbackGroupOdom;
    auto lidarOdomOpt = rclcpp::SubscriptionOptions();
    lidarOdomOpt.callback_group = callbackGroupLidarOdom;
    auto gnssOpt = rclcpp::SubscriptionOptions();
    gnssOpt.callback_group = callbackGroupGnss;

    // create publisher, subscription                   //non definitive name//
    subscribeImu = create_subscription<sensor_msgs::msg::Imu>("carla/ego_vehicle/imu", , std::bind(&IMUintegration::imuManager, this, std::placeholders::_1), imuOpt); ///check for qos param
    subLidarOdom = create_subscription<nav_msgs::msg::Odometry>("ic_graph/odometry/lidar", , std::bind(&Optimization::lidarOdomManager, this, std::placeholders::_1), lidarOdomOpt);
    subGnss = create_subscription<>("carla/ego_vehicle/gnss", , std::bind(&Optimization::));
    //subOdometry = create_subscription<nav_msgs::msg::Odometry>("", , std::bind(&Optimization::odomManager, this, std::placeholders::_1), odomOpt);
    //publishPose = create_publisher<>
    //subCarlaPose = create_subscription<nav_msgs::msg
}
       
void Optimization::initGraph(const double time, const gtsam::Pose3& initialPose) // graph initialization 
{
    ///void reset optimization ??

    //factorsGraph_->addPrior(X(,priorPose, pose_noise_model));
    //factorsGraph_->addPrior(V(,priorVelocity, velocity_noise_model));
    //factorsGraph_->addPrior(B(,priorImuBias, bias_noise_model));

    priorPoseNoise->Sigmas((gtsam::Vector(6)<<1e-3, 1e-3, 1e-3, 1e-6, 1e-6, 1e-6).finished());
    priorVelNoise->Sigma(3, 1e-3);
    priorBiasNoise->Sigmas((gtsam::Vector(6)<<1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3).finished());
    
    // Set initial pose
    prevPose_ =  ; //-->FINISH THIS use carla first position
    gtsam::PriorFactor<gtsam::Pose3> priorPose(X(0), prevPose_, priorPoseNoise);
    factorsGraph_->add(priorPose);

    // Set initial velocity
    prevVel_= gtsam::Vector3(0,0,0);
    gtsam::PriorFactor<gtsam::Vector3> priorVelocity(V(0), prevVel_, priorVelNoise);
    factorsGraph_->add(priorVelocity);

    // Set initial bias
    prevBias_ = gtsam::imuBias::ConstantBias();
    gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(B(0), prevBias_, priorBiasNoise);
    factorsGraph->add(priorBias);
    imuPre
    // add values to the graph
    graphValues->insert(X(0), prevPose_);
    graphValues->insert(V(0), prevVel_);
    graphValues->insert(B(0), prevBias_);

    // optimize once
    isam2->update(factorsGraph, graphValues); //CHECK THE DECLARATION OF OPTIMIZER
    factorsGraph_->resize(0);
    graphValues->clear();

    imuPreintegrationPtr_->resetIntegrationAndSetBias(prev_bias); //change variable

    key=1;
    initFlag = true;
    return ;
}
void Optimization::imuManager(const sensor_msgs::msg::Imu::ConstPtr& imuRaw)
{
    Eigen::Vector3d linearAccel(imuRaw->linear_acceleration.x, imuRaw->linear_acceleration.y, imuRaw->linear_acceleration.z);
    Eigen::Vector3d angularVel(imuRaw->angular_velocity.x, imuRaw->angular_velocity.y, imuRaw->angular_velocity.z);

    IMUintegration::addImu2Buffer(linearAccel, angularVel, timestamp);
    if ()
    {
        Optimization::addImuFactor(,linearAccel, angularVel);
    }
    
    else 
    {
        return;
    }

}
void Optimization::gnssManager()
{

}

void Optimization::lidarOdomManager()
{
    if ()
    {
        Optimization::addDualLidarOdomFactor();
    }
    addImuFactor()

    /// If using NDT instead of ICP ???
    //else  
    //{
    //    Optimization::addLidarOdomFactor();
    //}
}


void Optimization::addIMUFactor(const Eigen::Vector3d& linearAcc, const Eigen::Vector3d& angularVel, const double time)
{
    gtsam::key oldKey;
    gtsam::key newKey;
    prevBias_ = gtsam::imuBias::ConstantBias();
    // Update the IMU preintegrator
    imu_integration::updateIntegration();
    gtsam::NavState imuPredictedState = imuPreintegrationPtr_->predict(imuPredictedState)
    gtsam::CombinedImuFactor imuFactor(gtsam::symbol_shorthand::X(oldKey), gtsam::symbol_shorthand::V(oldKey)
                                       gtsam::symbol_shorthand::X(newKey), gtsam::symbol_shorthand::V(newKey)
                                       gtsam::symbol_shorthand::B(oldKey), gtsam::symbol_shorthand::B(newKey), *imuPreintegrationPtr_);
    factorsGraph->add(imuFactor);

     // Add IMU values
    gtsam::Values valuesEstimate;
    valuesEstimate.insert(gtsam::symbol_shorthand::X(newKey), imuPredictedState_.pose());
    valuesEstimate.insert(gtsam::symbol_shorthand::V(newKey), imuPredictedState_.velocity());
    valuesEstimate.insert(gtsam::symbol_shorthand::B(newKey), prevBias_);

    graphValues_->insert(valuesEstimate);

    if (useGnssFlag)
    {
        Optimization::addGnssFactor();
    }
}

/*######*/
// Not used for now
/*######*/
//void Optimization::addOdometryFactor()
//{
 //   factorsGraph->add(odometryFactor_);
//}

void Optimization::addDualLidarOdomFactor(const nav_msgs::msg::Odometry::SharedPtr odomData)
{
    const std::lock_guard<std::mutex> lock(lidarMutex_);

    float pos_X = odomData-> pose.position.X;
    float pos_Y = odomData-> pose.position.Y;
    float pos_Z = odomData-> pose.position.Z;
    float rot_X = odomData-> pose.rotation.X;
    float rot_Y = odomData-> pose.rotation.Y;
    float rot_Z = odomData-> pose.rotation.Z;
    float rot_W = odomData-> pose.rotation.W;
    gtsam::Pose3 lidarPose = gtsam::Pose3(gtsam::Rot3::Quaternion(rot_W, rot_X, rot_Y, rot_Z), gtsam::Point3(pos_X, pos_Y, pos_W));
    factorsGraph->add(lidarFactor_);
}

void Optimization::addDualLidarOdomFactor()
{
    factorsGraph->add(lidarDualFactor_);
}

void Optimization::addGnssFactor()
{
    gnss::addGnssFactor(); // To Do 
    gnss::addGnssYaw(); // To Do
    factorsGraph->add(gnssPosFactor);
    factorsGraph->add(gnssYawFactor);
}

/*######*/
/*Graph optimization function*/
/*######*/
void Optimization::optimizeGraph() 
{
    if (initFlag == false)
    {
        initGraph();
    }
    //if (imuMeasFlag) 
    //{
     //   addIMUFactor();
    //}
    

    startOpti = std::chrono::high_resolution_clock::now();
    
    gtsam::NavState optimizedState = updateGraph();
    optimizedState.pose();
    
    
    currState = 

    fixedLagSmoother_->update(factorsGraph, valuesGraph);
    endOpti = std::chrono::high_resolution_clock::now();
    std::cout << " Whole optimization loop took :" << std::chrono::duration_cast<std::chrono::milliseconds>(endOpti - startOpti).count() << " milliseconds."
    << std::endl;

    factorsGraph_->resize(0);
    graphValues_->clear();


    

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