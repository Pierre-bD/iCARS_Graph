#include"optimization.hpp"

namespace ic_graph {
optimization::optimization()
{
    
    isamParams.relinearizeTreshold = 0.01; //define proper value
    isamParams.relinearizeSkip = 1;
    bool initFlag = false;

    //Define prior noise for first optimisation
    //priorPoseNoise = gtsam::noiseModel::Diagonal::sigmas();
    //priorVelNoise = gtsam::noiseModel::Diagonal::sigma();
    //priorBiasNoise = gtsam::noiseModel::Diagonal::sigma();
    subImuPreint = create_subscription<>(,,std::bind());
    subLidarOdom = create_subscription<>();
    subGnssData = create_subscription<>();
    subOdometry = create_subscription<>();
}
       
void optimization::addIMUFactor(const double time, const Eigen::Vector3d& linearAcc, Eigen::Vector3d& angularVel);
    //void optimization::addOdometryFactor();
    //void optimization::addLidarOdomFactor();
    //void optimization::addGNSSpositionFactor();
    //void optimization::addGNSSYawFactor();
    //void optimization::optimizationgraph();

void optimization::initGraph()
{
    factorGraph->addPrior(X(,priorPose,pose_noise_model));
    factorGraph->addPrior(V(,priorVelocity,velocity_noise_model));
    factorGraph->addPrior(B(,priorImuBias,bias_noise_model));

    initFlag = true;
    return initFlag;
}

void optimization::optimizationgraph() 
{
    if (initFlag == false):
    {
        initGraph();
    }
    startOpti = std::chrono::high_resolution_clock::now();
    
    gtsam::NavState optimizedState = updateGraph();
    optimizedState.pose();
    
    endOpti = std::chrono::high_resolution_clock::now();
    currState = 
}




}

int main(int argc, char** argv){

    //init optimization once (prior pose, vel bias, params....)    


    rclcpp::init(argc, argv);
    rclcpp::NodeOptions nodeParam;
    nodeParam.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor imuExec; //or use single thread, will see how many nodes I define
    

    imuExec.add_node();

    //imuExec.spin();


    return 0;
}