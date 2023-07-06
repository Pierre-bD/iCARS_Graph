#include "iCARS_Graph/utils.hpp"
#include "iCARS_Graph/gnss.hpp"

namespace ic_graph {

gnss::gnss()
{
}

    void gnss::setReference()
    {
        
    }
    GNSSStat gnss::convert2Cartesian(const sensor_msgs::msg::NavSatFix::ConstSharedPtr gnssRaw)
    {   
        GNSSStat localCartesian_;
        try
        {
            GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f());
            //double gnssOrigin_latitude = 0, gnssOrigin_longitude = 2.20;

            std::cout<<"GNSS:: Entered in try loop"<<std::endl;
            GeographicLib::LocalCartesian geo2Cartesian(0, 0, 0, earth);
            geo2Cartesian.Forward(gnssRaw->latitude, gnssRaw->longitude, gnssRaw->altitude, localCartesian_.x, localCartesian_.y, localCartesian_.z);
        }
        catch(const GeographicLib::GeographicErr& convertError)
        {
            //std::cerr << convertError.what() << '\n';
            std::cout<<"can't convert to cartesian"<<std::endl;
        }
        localCartesian_.y = -localCartesian_.y;
        localCartesian_.latitude = gnssRaw->longitude;
        localCartesian_.longitute = gnssRaw->longitude;
        localCartesian_.altitude = gnssRaw->altitude;
        std::cout<<"GNSS:: localCartesian will be returned"<<std::endl;
        return localCartesian_;
        
    }
    geometry_msgs::msg::Point gnss::getGnssPosition(const GNSSStat & gnss_stat)
    {
        geometry_msgs::msg::Point gnssPoint_;
        gnssPoint_.x = gnss_stat.x;
        gnssPoint_.y = gnss_stat.y;
        gnssPoint_.z = gnss_stat.z;
        return gnssPoint_;
    }
   /* geometry_msgs::msg::Quaternion gnss::getGnssOrientation(const int heading)
    {
        int heading_conv = 0;

        if (heading >= 0 && heading <= 27000000) 
        {
        heading_conv = 9000000 - heading;
        } 
        else 
        {
        heading_conv = 45000000 - heading;
        }
        const double yaw = (heading_conv * 1e-5) * M_PI / 180.0;
        tf2::Quaternion quaternion;
        quaternion.setRPY(0, 0, yaw);
        
        return quaternion;
    }*/
    
    void gnss::addGnssPosition()
    {
        //program
    }

    void gnss::addGnssYaw()
    {
        //program
    }

}