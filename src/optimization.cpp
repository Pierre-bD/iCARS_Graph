#include"optimization.hpp"
#include"imuIntegration.hpp"
#include"utils.hpp"
#include <Eigen>

namespace ic_graph {

Optimization::Optimization() : Node("optimization")
{
    isamParams.relinearizeTreshold = 0.01; //define proper value
    isamParams.relinearizeSkip = 1;
    double fixedLag = 2.0;
    const double gravity = 9.81;
    stateKey_ = 0;
    factorsGraph_ = std::make_shared<gtsam::NonlinearFactorGraph>();
    graphValues_ = std::make_shared<gtsam::Values>();
    fixedLagSmoother_ = std::make_shared<gtsam::IncrementalFixedLagSmoother>(fixedLag,isamParams); 
    imuPreintegration_ = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(imuparams, *imuPriorBias_);
    bool initFlag = false;
    bool useGnssFlag = false;

    //Define prior noise for first optimisation
    priorPoseNoise = noiseModel::Diagonal::Sigmas((Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished());
    priorVelNoise = noiseModel::Isotropic::Sigma(3, 0.1);
    priorBiasNoise = noiseModel::Isotropic::Sigma(6, 1e-3);
    correctionNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.05, 0.05, 0.05, 0.1, 0.1, 0.1).finished()); // rad,rad,rad,m, m, m
    correctionNoise2 = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 1, 1, 1, 1, 1, 1).finished()); // rad,rad,rad,m, m, m
    noiseModelBetweenBias = (gtsam::Vector(6) << imuAccBiasN, imuAccBiasN, imuAccBiasN, imuGyrBiasN, imuGyrBiasN, imuGyrBiasN).finished();
    
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
       
void Optimization::initGraph(const double optiTime, const gtsam::Pose3& initialPose) // graph initialization 
{
    ///void reset optimization ??

    //factorsGraph_->addPrior(X(,priorPose, pose_noise_model));
    //factorsGraph_->addPrior(V(,priorVelocity, velocity_noise_model));
    //factorsGraph_->addPrior(B(,priorImuBias, bias_noise_model));

    imu_integration::initImu(gravity, "up");

    priorPoseNoise->Sigmas((gtsam::Vector(6)<<1e-3, 1e-3, 1e-3, 1e-6, 1e-6, 1e-6).finished());
    priorVelNoise->Sigma(3, 1e-3);
    priorBiasNoise->Sigmas((gtsam::Vector(6)<<1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3).finished());
    
    // Set initial pose
    prevPose_ = initialPose ; //-->FINISH THIS use carla first position
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
    // add values to the graph
    graphValues->insert(X(0), prevPose_);
    graphValues->insert(V(0), prevVel_);
    graphValues->insert(B(0), prevBias_);

    // optimize once
    fixedLagSmoother_->update(factorsGraph, graphValues, optiTime); //CHECK THE DECLARATION OF OPTIMIZER
    factorsGraph_->resize(0);
    graphValues->clear();

    imuPredictedState_ = gtsam::NavState(initialPose, gtsam::Vector3(0, 0, 0));
    
    imu_integration::resetImu();
    key=1;
    initFlag = true;
    return ;
}
void Optimization::imuManager(const sensor_msgs::msg::Imu::ConstPtr& imuRaw)
{
    Eigen::Vector3d linearAccel(imuRaw->linear_acceleration.x, imuRaw->linear_acceleration.y, imuRaw->linear_acceleration.z);
    Eigen::Vector3d angularVel(imuRaw->angular_velocity.x, imuRaw->angular_velocity.y, imuRaw->angular_velocity.z);

    stateTime_ = imuRaw->header.stamp.sec + 1e-9 * imuRaw->header.stamp.nanosec;

    IMUintegration::addImu2Buffer(linearAccel, angularVel, stateTime_);
    if ()
    {
        Optimization::addImuFactor(linearAccel, angularVel);
        optimizeGraph();

    }

}
/*void Optimization::gnssManager()
{

}*/

/*void Optimization::lidarOdomManager()
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
}*/


void Optimization::addIMUFactor(const double imuTime)
{
    {
    stateTime_ = imuTime;
    prevBias_ = gtsam::imuBias::ConstantBias();

    //get keys
    oldKey = stateKey_ ; 
    newKey = newStateKey_();
    imu_integration::addKey2Buffer(imuTime, newKey);

    // Update the IMU preintegrator
    imu_integration::updateIntegration(start_time, end_time);
    
    gtsam::CombinedImuFactor imuFactor(gtsam::symbol_shorthand::X(oldKey), gtsam::symbol_shorthand::V(oldKey)
                                       gtsam::symbol_shorthand::X(newKey), gtsam::symbol_shorthand::V(newKey)
                                       gtsam::symbol_shorthand::B(oldKey), gtsam::symbol_shorthand::B(newKey), *imuPreintegrationPtr_);
    factorsGraph_->add(imuFactor);
    factorsGraph_->add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(B(newKey), B(oldKey), gtsam::imuBias::ConstantBias(),gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuPreintegrationPtr_->deltaTij()) * noiseModelBetweenBias)));


     // Add IMU values
    gtsam::Values valuesEstimate;
    imuPredictedState_ = imuPreintegrationPtr_->predict(imuPredictedState, bias);
    valuesEstimate.insert(gtsam::symbol_shorthand::X(newKey), imuPredictedState_.pose());
    valuesEstimate.insert(gtsam::symbol_shorthand::V(newKey), imuPredictedState_.velocity());
    valuesEstimate.insert(gtsam::symbol_shorthand::B(newKey), prevBias_);

    graphValues_->insert(valuesEstimate);

   // if (useGnssFlag)
    //{
      //  Optimization::addGnssFactor();
    //}
    //}
}

/*######*/
// Not used for now
/*######*/
//void Optimization::addOdometryFactor()
//{
 //   factorsGraph->add(odometryFactor_);
//}
/*
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
*/
/*void Optimization::addGnssFactor()
{
    gnss::addGnssFactor(); // To Do 
    gnss::addGnssYaw(); // To Do
    factorsGraph->add(gnssPosFactor);
    factorsGraph->add(gnssYawFactor);
}*/

/*######*/
/*Graph optimization function*/
/*######*/
void Optimization::optimizeGraph() 
{
    std::lock_guard<std::mutex> lock(optimizationMutex_);
    if (initFlag == false)
    {
        initGraph(stateTime_, );
    }
    //if (imuMeasFlag) 
    //{
     //   addIMUFactor();
    //}  
    startOpti = std::chrono::high_resolution_clock::now();
    
    //gtsam::NavState optimizedState = updateGraph();
    //optimizedState.pose();

    fixedLagSmoother_->update(factorsGraph, valuesGraph);
    endOpti = std::chrono::high_resolution_clock::now();
    std::cout << " Whole optimization loop took :" << std::chrono::duration_cast<std::chrono::milliseconds>(endOpti - startOpti).count() << " milliseconds."
    << std::endl;


    factorsGraph_->resize(0);
    graphValues_->clear();

    gtsam::Values result = fixedLagSmoother_->calculateEstimate();
    prevPose_ = result.at<gtsam::Pose3>(X(newKey));
    prevVel_ = result.at<gtsam::Vector3>(V(newKey));
    prevState_ = result.at<gtsam::NavState(prevPose_, prevVel_);
    prevBias_ = result.at<gtsam::imuBias::ConstantBias>(B(newKey));

    imu_integration::resetImu(prevBias_);  

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