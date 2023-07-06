#ifndef _GNSS_HPP_
#define _GNSS_HPP_


//#include <Eigen>
#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Geocentric.hpp>
#include <cmath>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/buffer.h"
#include "iCARS_Graph/utils.hpp"

namespace ic_graph {

struct GNSSStat
{
 GNSSStat() : x(0), y(0), z(0), latitude(0), longitute(0), altitude(0)
    {
    }
    double x;
    double y;
    double z;
    double latitude;
    double longitute;
    double altitude;
};

class gnss
{
private:
    std::string bodyFrame_;
    std::string gnssFrame_;

    geometry_msgs::msg::Quaternion orientation;
    
    

    //std::shared_ptr<tf2_ros::Buffer> tf2_buffer_{nullptr};
    //std::shared_ptr<tf2_ros::TransformListener> tf2_listener_{nullptr};
    //tf2::BufferCore tf2_buffer_;


public:
    gnss();
    void setReference();

    geometry_msgs::msg::Point getGnssPosition(const GNSSStat& gnss_stat);
   // geometry_msgs::msg::Quaternion getGnssOrientation(const int heading);

    GNSSStat convert2Cartesian(const sensor_msgs::msg::NavSatFix::ConstSharedPtr gnssRaw);
    void addGnssPosition();
    void addGnssYaw();

    //tf2_buffer_ = std::make_unique<tf2_ros::Buffer>(const double& gnssTime);
    //tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_byffer_);
};
}

#endif
