/* ----------------------------------------------------------------------------
    Copyright (c) 2023, Pierre Baptiste Demonceaux
    All Rights Reserved.
    This file is released under the "BSD-2-Clause License".
    
    See LICENSE for the license information
 * -------------------------------------------------------------------------- */

#include "imu.hpp"

using gtsam::symbol_shorthand::X;  // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V;  // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B;  // Bias  (ax,ay,az,gx,gy,gz)