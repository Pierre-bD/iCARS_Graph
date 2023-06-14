#ifndef IMU_INTEGRATION_IMU_FILTER_HPP_
#define IMU_INTEGRATION_IMU_FILTER_HPP_

#include <test.hpp>
#include <memory>
#include <boost/optional.hpp>
#include <sensors/imu.hpp>

namespace imu_integration
{
    class imufilter
    {
    private:
        std::unique_ptr<Filter> acc_x_filter_;
        std::unique_ptr<Filter> acc_y_filter_;
        std::unique_ptr<Filter> acc_z_filter_;

        std::unique_ptr<Filter> angVel_x_filter_;
        std::unique_ptr<Filter> angVel_y_filter_;
        std::unique_ptr<Filter> angVel_z_filter_;

    public:
        imufilter();

        boost::optional<imu_integration::ImuMeasurement> addMeasures(const & imu_measures);
        
    };
}

#endif