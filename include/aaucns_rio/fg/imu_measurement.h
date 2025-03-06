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

#ifndef _IMU_MEASUREMENT_H_
#define _IMU_MEASUREMENT_H_

#include <sensor_msgs/Imu.h>

#include <Eigen/Dense>

namespace aaucns_rio
{
class IMUMeasurement
{
   public:
    IMUMeasurement(const sensor_msgs::ImuConstPtr& msg);
    IMUMeasurement() = default;

    // Angular velocity from IMU.
    Eigen::Matrix<double, 3, 1> w_m_;
    ///< acceleration from IMU.
    Eigen::Matrix<double, 3, 1> a_m_;
    double timestamp_s_;
};

}  // namespace aaucns_rio

#endif /* IMU_MEASUREMENT_H_ */
