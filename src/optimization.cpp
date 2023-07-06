#include"iCARS_Graph/optimization.hpp"
namespace ic_graph {

Optimization::Optimization() : Node("Optimization_publisher")
{
    std::cout<<"Inside constructor initialisation"<<std::endl;
    isamParams.relinearizeThreshold = 0.01; //define proper value
    isamParams.relinearizeSkip = 1;
    double fixedLag = 2.0;
    imuAccBiasN = 5e-4;
    imuGyrBiasN = 7e-5;
    factorsGraph_ = std::make_shared<gtsam::NonlinearFactorGraph>();
    graphValues_ = std::make_shared<gtsam::Values>();
    fixedLagSmoother_ = std::make_shared<gtsam::IncrementalFixedLagSmoother>(fixedLag,isamParams); 

        //Define prior noise for first optimisation
    priorPoseNoise = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.5, 0.5, 0.5).finished());
    priorVelNoise = gtsam::noiseModel::Isotropic::Sigma(3, 0.1);
    priorBiasNoise = gtsam::noiseModel::Isotropic::Sigma(6, 1e-3);
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
    subscribeImu = create_subscription<sensor_msgs::msg::Imu>("/carla/ego_vehicle/imu", 10, std::bind(&Optimization::imuManager, this, std::placeholders::_1), imuOpt); ///check for qos param
    //subLidarOdom = create_subscription<nav_msgs::msg::Odometry>("ic_graph/odometry/lidar", 10, std::bind(&Optimization::lidarOdomManager, this, std::placeholders::_1), lidarOdomOpt);
    subGnss = create_subscription<sensor_msgs::msg::NavSatFix>("/carla/ego_vehicle/gnss", 10, std::bind(&Optimization::gnssManager, this, std::placeholders::_1), gnssOpt);
    //subOdometry = create_subscription<nav_msgs::msg::Odometry>("", 10, std::bind(&Optimization::odomManager, this, std::placeholders::_1), odomOpt);
    //publishPose = create_publisher<>
    //subCarlaPose = create_subscription<nav_msgs::msg
    std::cout<<"End of constructor initialization"<<std::endl;
    std::cout<<"Waiting first message"<<std::endl;

}
       
void Optimization::initGraph(double optiTime, gtsam::Point3& initialGnssPose) // graph initialization 
{
    ///void reset optimization ??

    //factorsGraph_->addPrior(X(,priorPose, pose_noise_model));
    //factorsGraph_->addPrior(V(,priorVelocity, velocity_noise_model));
    //factorsGraph_->addPrior(B(,priorImuBias, bias_noise_model));
    stateKey_ = 0;
    const double gravity = 9.81;
    imuIntegrationPtr_->initImu(gravity, "up");
    lastTime_ = optiTime; //time for imu integration
    keyTimeStampMap_[stateKey_] = lastTime_;
    gtsam::Rot3 priorRotation = gtsam::Rot3::Quaternion(1, 0.0, 0.0, 0.0);
    gtsam::Pose3 initialPose = gtsam::Pose3(priorRotation, initialGnssPose);

    // Set initial pose
    prevPose_ = initialPose ;
    gtsam::PriorFactor<gtsam::Pose3> priorPose(gtsam::symbol_shorthand::X(0), prevPose_, priorPoseNoise);
    factorsGraph_->add(priorPose);

    // Set initial velocity
    prevVel_= gtsam::Vector3(0,0,0);
    gtsam::PriorFactor<gtsam::Vector3> priorVelocity(gtsam::symbol_shorthand::V(0), prevVel_, priorVelNoise);
    factorsGraph_->add(priorVelocity);

    // Set initial bias
    prevBias_ = gtsam::imuBias::ConstantBias();
    gtsam::PriorFactor<gtsam::imuBias::ConstantBias> priorBias(gtsam::symbol_shorthand::B(0), prevBias_, priorBiasNoise);
    factorsGraph_->add(priorBias);
    // add values to the graph
    graphValues_->insert(gtsam::symbol_shorthand::X(stateKey_), prevPose_);
    graphValues_->insert(gtsam::symbol_shorthand::V(stateKey_), prevVel_);
    graphValues_->insert(gtsam::symbol_shorthand::B(stateKey_), prevBias_);

    // optimize once
    fixedLagSmoother_->update(*factorsGraph_, *graphValues_, keyTimeStampMap_); //CHECK THE DECLARATION OF OPTIMIZER
    factorsGraph_->resize(0);
    graphValues_->clear();

    imuPredictedState_ = gtsam::NavState(initialPose, gtsam::Vector3(0, 0, 0));
    
    imuIntegrationPtr_->resetImu(prevBias_);

    key=1;
    initFlag = true;
    return ;
}

void Optimization::imuManager(const sensor_msgs::msg::Imu::ConstSharedPtr& imuRaw)
{

    static int imuCounter = 0;
    Eigen::Vector3d linearAccel(imuRaw->linear_acceleration.x, imuRaw->linear_acceleration.y, imuRaw->linear_acceleration.z);
    Eigen::Vector3d angularVel(imuRaw->angular_velocity.x, imuRaw->angular_velocity.y, imuRaw->angular_velocity.z);
    stateTime_ = imuRaw->header.stamp.sec + 1e-9 * imuRaw->header.stamp.nanosec;
    imuIntegrationPtr_->addImu2Buffer(linearAccel, angularVel, stateTime_);
    ++imuCounter; 
    if (initFlag == false)
    {
        //initGraph(stateTime_, );
        return;
    }
    else if (imuCounter == 4)
    {

        addImuFactor(stateTime_);
        optimizeGraph(stateTime_, newKey);
        imuCounter = 0;
    }
}

void Optimization::gnssManager(const sensor_msgs::msg::NavSatFix::ConstSharedPtr GnssRaw)
{
    std::cout<<"gnss message received"<<std::endl;
    //static int gnssCounter = 0;
    //Eigen::Vector3d gnssCoordinate = Eigen::Vector3d(GnssRaw->latitude, GnssRaw->longitude, GnssRaw->altitude);
    //Eigen::Vector3d covarianceXYZ(GnssRaw->position_covariance[0], GnssRaw->position_covariance[4], GnssRaw->position_covariance[8]);
    
    const auto gnssStat = gnssPtr_->convert2Cartesian(GnssRaw); 
    const auto gnssPosition = gnssPtr_->getGnssPosition(gnssStat); //see if need to convert data into gtsam type
    
    //const auto gnssOrientation = getGnssOrientation();
    //gtsam::Rot3 gnssOrientation = gtsam::Rot3;
    //gnssPosition = gnssPosition;
    //gnssPose_.orientation = gnssOrientation;

    gtsam::Vector3 gnssVectorPoints = gtsam::Vector3(gnssPosition.x, gnssPosition.y, gnssPosition.z);
    gnssTimeStamp_ = GnssRaw->header.stamp.sec + 1e-9 * GnssRaw->header.stamp.nanosec;

    std::cout<<"gnss TimeStamp: " <<gnssTimeStamp_<<"gnss_Pose: "<<gnssVectorPoints<<std::endl;

    //tf2::Transform tf_map2gnss{};
    //tf2::fromMsg();
   /* if (initFlag == false)
    { 
        
        initialePosition = gtsam::Point3(gnssVectorPoints);
        
        initGraph(gnssTimeStamp_, initialePosition);
        return;
    }*/
    
    //orientation = ;
    //prev_position = ; 
    //gnssPose_.position = 0.0;
    //gnssPose

    //addGnssFactor(gnssTimeStamp_, gnssVectorPoints);


}

/*void Optimization::lidarOdomManager(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
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


void Optimization::addImuFactor(const double imuTime)
{
    {
    stateTime_ = imuTime;
    prevBias_ = gtsam::imuBias::ConstantBias();

    //get keys
    oldKey = stateKey_ ; 
    newKey = newStateKey_();
    //imuIntegrationPtr_->addKey2Buffer(stateTime_, newKey);

    // Update the IMU preintegrator
    imuIntegrationPtr_->updateIntegration(lastTime_, stateTime_, prevBias_);
    
    gtsam::CombinedImuFactor imuFactor(gtsam::symbol_shorthand::X(oldKey), gtsam::symbol_shorthand::V(oldKey),
                                       gtsam::symbol_shorthand::X(newKey), gtsam::symbol_shorthand::V(newKey),
                                       gtsam::symbol_shorthand::B(oldKey), gtsam::symbol_shorthand::B(newKey), *imuIntegrationPtr_->imuPreintegrationPtr_);
    factorsGraph_->add(imuFactor);
    factorsGraph_->add(gtsam::BetweenFactor<gtsam::imuBias::ConstantBias>(gtsam::symbol_shorthand::B(newKey), gtsam::symbol_shorthand::B(oldKey), gtsam::imuBias::ConstantBias(),gtsam::noiseModel::Diagonal::Sigmas(sqrt(imuIntegrationPtr_->imuPreintegrationPtr_->deltaTij()) * noiseModelBetweenBias)));

     // Add IMU values
    gtsam::Values valuesEstimate;
    imuPredictedState_ = imuIntegrationPtr_->imuPreintegrationPtr_->predict(imuPredictedState_, prevBias_);
    valuesEstimate.insert(gtsam::symbol_shorthand::X(newKey), imuPredictedState_.pose());
    valuesEstimate.insert(gtsam::symbol_shorthand::V(newKey), imuPredictedState_.velocity());
    valuesEstimate.insert(gtsam::symbol_shorthand::B(newKey), prevBias_);

    graphValues_->insert(valuesEstimate);

   // if (useGnssFlag)
    //{
      //  Optimization::addGnssFactor();
    //}
    }
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
void Optimization::addGnssFactor(const double gpsTime, gtsam::Vector3& gnssPosition)
{
    double closestTime;
    gtsam::Key gnssKey;
    auto noise = gtsam::noiseModel::Isotropic::Sigma(3, 1.0);
    auto robustGnssError = gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(0.5), noise); // see if value is adapted
    imuIntegrationPtr_->getClosestKey(gpsTime, gnssKey, closestTime);
    gtsam::GPSFactor gnssFactor_(gtsam::symbol_shorthand::X(closestTime), gnssPosition, robustGnssError); //finish the symbol
    factorsGraph_->add(gnssFactor_);
    //factorsGraph_->add(gnssYawFactor);
}
/*######*/
/*Graph optimization function*/
/*######*/
void Optimization::optimizeGraph(const double optiTime, gtsam::Key& optiKey) 
{
    std::lock_guard<std::mutex> lock(optimizationMutex_);
    if (initFlag == false)
    {
        //initGraph(stateTime_, );
        return;
    }
    //if (imuMeasFlag) 
    //{
     //   addIMUFactor();
    //}  
    keyTimeStampMap_[optiKey] = optiTime;
    startOpti = std::chrono::high_resolution_clock::now();
    
    //gtsam::NavState optimizedState = updateGraph();
    //optimizedState.pose();

    fixedLagSmoother_->update(*factorsGraph_, *graphValues_, keyTimeStampMap_);
    endOpti = std::chrono::high_resolution_clock::now();
    std::cout << " Whole optimization loop took :" << std::chrono::duration_cast<std::chrono::milliseconds>(endOpti - startOpti).count() << " milliseconds."
    << std::endl;


    factorsGraph_->resize(0);
    graphValues_->clear();

    gtsam::Values result = fixedLagSmoother_->calculateEstimate();
    prevPose_ = result.at<gtsam::Pose3>(gtsam::symbol_shorthand::X(optiKey));
    prevVel_ = result.at<gtsam::Vector3>(gtsam::symbol_shorthand::V(optiKey));
    prevState_ = gtsam::NavState(prevPose_, prevVel_);
    prevBias_ = result.at<gtsam::imuBias::ConstantBias>(gtsam::symbol_shorthand::B(optiKey));

    imuIntegrationPtr_->resetImu(prevBias_);
}

}

int main(int argc, char** argv){

    //init optimization once (prior pose, vel bias, params....)    
    std::cout<<"Starting initializing node"<<std::endl;
    rclcpp::init(argc, argv);
    //rclcpp::NodeOptions nodeParam;
    //nodeParam.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor optiExec; //or use single thread, will see how many nodes I define
    std::cout<<"Node is initialized"<<std::endl;
    auto poseOptimization = std::make_shared<ic_graph::Optimization>();

    optiExec.add_node(poseOptimization);

    optiExec.spin();

    rclcpp::shutdown();

    return 0;
}
