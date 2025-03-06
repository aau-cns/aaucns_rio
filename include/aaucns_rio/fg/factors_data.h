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

#ifndef _FACTORS_DATA_H_
#define _FACTORS_DATA_H_

#include <aaucns_rio/features.h>
#include <aaucns_rio/state.h>
#include <gtsam/navigation/CombinedImuFactor.h>

#include <Eigen/Dense>

namespace aaucns_rio
{
class IMUFactorData
{
   public:
    IMUFactorData(){};
    IMUFactorData(
        const std::shared_ptr<gtsam::PreintegratedCombinedMeasurements>&
            imu_integrator)
        : imu_integrator_(imu_integrator)
    {
    }
    std::shared_ptr<gtsam::PreintegratedCombinedMeasurements> imu_integrator_;
};

class RadialVelocityFactorData
{
   public:
    // TODO(jan): Implement move constructor.
    RadialVelocityFactorData(){};
    RadialVelocityFactorData(const Eigen::MatrixXd& velocities_and_points,
                             const Eigen::Matrix<double, 3, 1>& w_m)
        : velocities_and_points_(velocities_and_points), w_m_(w_m)
    {
    }

    Eigen::MatrixXd velocities_and_points_;
    // Angular velocity from IMU.
    Eigen::Matrix<double, 3, 1> w_m_;
};

class RadarDistanceFactorData
{
   public:
    // TODO(jan): Implement move constructor.
    RadarDistanceFactorData(){};
    RadarDistanceFactorData(const Features& features) : features_(features) {}

    Features features_;
};

class RadarPFDistanceFactorData
{
   public:
    // TODO(jan): Implement move constructor.
    RadarPFDistanceFactorData(){};
    RadarPFDistanceFactorData(const State& most_recent_updated_state)
        : most_recent_updated_state_(most_recent_updated_state)
    {
    }

    State most_recent_updated_state_;
};

}  // namespace aaucns_rio

#endif /* _FACTORS_DATA_H_ */
