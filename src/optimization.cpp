#include"optimization.hpp"

namespace ic_graph {
optimization::optimization(){}
    //Imu prior parameters    
    priorPoseNoise =
    priorVelNoise  = 
    priorBiasNoise =
    
    std::shared_ptr<gtsam::ISAM2(isamParams)> isam2;

        
    void optimization::addIMUFactor(const double time, const Eigen::Vector3d& linearAcc, Eigen::Vector3d& angularVel);
    //void optimization::addOdometryFactor();
    //void optimization::addLidarOdomFactor();
    //void optimization::addGNSSpositionFactor();
    //void optimization::addGNSSYawFactor();
    //void optimization::optimizationgraph();

void optimization::initGraph()
{

    initFlag = true;
}

void optimization::optimizationgraph() 
{
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