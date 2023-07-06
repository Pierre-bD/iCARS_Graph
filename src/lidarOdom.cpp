#include "graph/utils.hpp"
#include "graph/lidar.hpp"
#include <tf2_eigen/tf2_eigen.hpp>
namespace lidr_odom  
{
    lidarOdom::lidarOdom()
    {
        callbackGroupLidar = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callbackGroupImu = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        //Other callback group ??
        
        auto lidarOption = rclcpp::SubscriptionOptions();
        auto imuOption = rclcpp::SubscriptionOptions();

        lidarOption.callback_group = callbackGroupLidar;
        imuOption.callback_group = imuOption;

        subLidarCloud = create_subscription<sensor_msgs::msg::PointCloud2>(lidarTopic, qos_lidar, std::bind(, this, std::placeholders::_1), lidarOption);
        subImu = create_subscription<sensor_msgs::msg::Imu>(imuTopic, qos_imu, std::bind(lidarOdom::imuBuffer, this, std::placeholders::_1), imuOption);

        pubFilteredCloud = create_publisher<sensor_msgs::msg::PointCloud2>();
        pubDeskewCloud = create_publisher<sensor_msgs::msg::PointCloud2>();
        pubLidarOdom = create_publisher<nav_msgs::msg::Odometry>();


    }
    void imuBuffer(const sensor_msgs::msg::Imu::Sharedtr imuMsg)
    {
        sensor_msgs::msg::Imu imuData = imuDataConverter(*imuMsg);
        imuQueue.push_back(imuData);   
        sensor_msgs::msg::
    }

    void odomBuffer(const nav_msgs::msg::Odometry::SharedPtr odomMsg)
    {
        odomQueue.push_back(*odomMsg);
    }


// put a condition if use with zoe houster
    pclPoint deskewCloud(pclPoint *point, double relativeTime)
    {
        float currentXPos, currentYPos, currentZPos;

        float currentXRot, currentYRot, currentZRot;

        if (firstPointFlag == true)
        {
            firstTransform = (plc::getTransformation(current)) ;
            firstPointFlag = false;
        }
        Eigen::Affine3f  lastTransform = pcl::getTransformation(currentXPos, currentYPos, currentZPos, currentXRot, currentYRot, currentZRot);
        Eigen::Affine3f betweenTransform = * lastTransform;
        
    }

    void motionCompensation()
    {


    }

    void pubLidarOdom()
    {





    }




}

int main(int argc, char** argv){

    rclcpp::init(argc, argv);
    rclcpp::NodeOptions nodeParam;
    nodeParam.use_intra_process_comms(true);
    rclcpp::executors::MultiThreadedExecutor odomExec; //or use single thread, will see how many nodes I define

    odomExec.add_node();

    odomExec.spin();

    return 0;
}