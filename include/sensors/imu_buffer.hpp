/* ----------------------------------------------------------------------------
    Copyright (c) 2023, Pierre Baptiste Demonceaux
    All Rights Reserved.
    This file is released under the "BSD-2-Clause License".
    
    See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#ifndef GNSS_HPP_
#define GNSS_HPP_
#include <Eigen/Dense>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/symbol.h>

namespace ic_graph {

    class imu_buffer {
    public:
        void add2buffer(double ts, double accx, double accy, double accz, double gyrox, double gyroy, double gyroz);

        void addkey2buffer(gtsam::Key);

    };
}

#endif