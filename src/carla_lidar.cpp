#include <chrono>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h> 


class carlaLidar : public rclcpp::Node
{
public:
    carlaLidar() : Node("")
    {
        publisher_

    }


    void combineClouds(const PointCloud2::ConstSharedPtr& in1, const PointCloud2::ConstSharedPtr& in2, PointCloud2::SharedPtr& out)
    {
        jdjd;
            //get base frame --> correspond to the last lidar measurement

        pcl_ros::transformPointCloud(//base_frame_ , ros_pc, transf_ros_pc, tf_listener_);
        pcl::concatenatePointCloud(pc_1, pc_2, out);
        pc_2 = out;


        publish_()
    }
}




int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions nodeParam;
    nodeParam.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor optiExec; //or use single thread, will see how many nodes I define

    auto subCarla = std::make_shared<Optimization>(nodeParam);
    auto pubAccuLidar = std::make_shared

    optiExec.add_node(poseOptimization);

    optiExec.spin();

    rclcpp::shutdown();

    return 0;
}