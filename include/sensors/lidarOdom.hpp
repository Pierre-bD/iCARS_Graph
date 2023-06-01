#ifndef _LIDAR_HPP_
#define _LIDAR_HPP_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/organized.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <boost/circular_buffer.hpp>



#include <rclcpp/rclcpp.hpp>

#include "utils.hpp"

typedef pcl::PointXYZI pclPoint;
namespace ic_graph 
{
    class lidarOdom : public rclcpp:Node
    {
            
        private:
            rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subLidarCloud;
            rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubFilteredCloud;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubDeskewCloud;
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubLidarOdom;

            rclcpp::CallbackGroup::SharedPtr callbackGroupLidar;
            std::deque<sensor_msgs::msg::Imu> imuQueue;
            std::deque<nav_msgs::msf::Odometry>odomQueue;
            std::deque<sensor_msgs::msg::PointCloud2>cloudQueue;


        public: 
            lidarOdom();
            void imuBuffer(const sensor_msgs::msg::Imu::Sharedtr imuData);
            void odomBuffer();
            void motionCompensation();
            void pubLidarOdom();
            boost::circular_buffer<PointCloud2Ptr> pointCloudBuffer_;
    }



}