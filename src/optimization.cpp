#include"optimization.hpp"

namespace ic_graph {
    
    //Imu prior parameters    
    priorPoseNoise =
    priorVelNoise  = 
    priorBiasNoise =


        
        void addIMU();
        //void addOdometry();
        //void addLidarOdom();
        //void addGNSSposition();
        //void optimizationgraph();

void optimization::initGraph()
{


}

void optimization::optimizationgraph() 
{

        



    
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