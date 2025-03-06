// Copyright (C) 2024 Jan Michalczyk, Control of Networked Systems, University
// of Klagenfurt, Austria.
//
// All rights reserved.
//
// This software is licensed under the terms of the BSD-2-Clause-License with
// no commercial use allowed, the full terms of which are made available
// in the LICENSE file. No license in patents is granted.
//
// You can contact the author at <jan.michalczyk@aau.at>

#include "aaucns_rio/fg/imu_measurement.h"

namespace aaucns_rio
{
IMUMeasurement::IMUMeasurement(const sensor_msgs::ImuConstPtr& msg)
{
    a_m_ << msg->linear_acceleration.x, msg->linear_acceleration.y,
        msg->linear_acceleration.z;
    w_m_ << msg->angular_velocity.x, msg->angular_velocity.y,
        msg->angular_velocity.z;
    timestamp_s_ = msg->header.stamp.toSec();
}

}  // namespace aaucns_rio