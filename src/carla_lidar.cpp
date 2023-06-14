#include <chrono>
#include <memory>
#include <string>
#include <queue>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h> 
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


class carlaLidar : public rclcpp::Node
{
protected:
    tf2_ros::TransformListener tf2_listener;
    typedef pcl::PointCloud<pcl::PointXYZ> point_cloud_t;
    point_cloud_t fused_cloud;

    std:: string base_frame_id;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriber_lid;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_lid;
    
    void publishCloud ()
    {
        pcl::PCLPointCloud2 tmp_pc;
        pcl::toPCLPointCloud2(fused_cloud, tmp_pc);
    }


public:
    carlaLidar() : Node("carla_lidar"), count_(0)
    {
        subscriber_lid = this->create_subscription<sensor_msgs::msg::PointCloud2>("", 10, std::bind(&carlaLidar::add_cloud, this, _1));
        publisher_lid = this->create_publisher<sensor_msgs::msg::PointCloud2>("ic_graph/cloud/fusion_cloud",10); //check for value 10
        int counter = 0;

    }
    deque<sensor_msgs::msg::PointCloud2> cloudQueue;

    void add_cloud(const sensor_msgs::msg::PointCloud2& ros_pc)
    {
        cloudQueue.push_back(ros_pc);
        counter +=1 ;
        if (counter == 10)
        {
            carlaLidar::combineClouds(cloudQueue);
            counter = 0; // Set counter back to 0

        }
    }

    void combineClouds(deque<sensor_msgs::msg::PointCloud2>& deque)
    {
        fused_cloud.clear();
        sensor_msgs::msg::PointCloud2 last_lidar;
        last_lidar = deque.back();
        base_frame_id = last_lidar.header.frame_id ; //get base frame id --> correspond to the last lidar measurement

        pcl_ros::transformPointCloud(base_frame_id , ros_pc, transf_ros_pc, tf_listener_);
        pcl::concatenatePointCloud(pc_1, pc_2, out);
        pc_2 = out;

        ros_cloud.header.frame_id = base_frame_id;
        pub_.publish(ros_cloud);
    }
}


int main(int argc, char** argv) {

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions nodeParam;
    nodeParam.use_intra_process_comms(true);
    rclcpp::executors::SingleThreadedExecutor carlaLidarExec; 

    auto accuLidar = std::make_shared<carlaLidar>(nodeParam);

    carlaLidarExec.add_node(poseOptimization);

    carlaLidarExec.spin();

    rclcpp::shutdown();

    return 0;
}