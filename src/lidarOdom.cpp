#include "utils.hpp"

int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions nodeParam;
    nodeParam.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor imuExec; //or use single thread, will see how many nodes I define

    imuExec.add_node();

    imuExec.spin();

    return 0;
}