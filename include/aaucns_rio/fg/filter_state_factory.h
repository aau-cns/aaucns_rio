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

#ifndef _FILTER_STATE_FACTORY_H_
#define _FILTER_STATE_FACTORY_H_

#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

#include <Eigen/Dense>

#include "aaucns_rio/fg/imu_measurement.h"
#include "aaucns_rio/state.h"

namespace aaucns_rio
{
class FilterStateFactory
{
   public:
    static State createState(const gtsam::NavState& current_state,
                             const gtsam::NavState& previous_state,
                             const IMUMeasurement& imu_measurement,
                             const gtsam::imuBias::ConstantBias& imu_bias,
                             const gtsam::Pose3& radar_to_imu_transform)
    {
        State state;
        state.p_ = current_state.pose().translation();
        state.v_ = current_state.v();
        state.q_ = current_state.pose().rotation().toQuaternion();
        state.b_w_ = imu_bias.gyroscope();
        state.b_a_ = imu_bias.accelerometer();
        state.p_ri_ = radar_to_imu_transform.translation();
        state.q_ri_ = radar_to_imu_transform.rotation().toQuaternion();
        state.w_m_ = imu_measurement.w_m_;
        state.a_m_ = imu_measurement.a_m_;
        // Holds the last past pose used in trail matching.
        state.past_positions_.push_front(previous_state.pose().translation());
        state.past_orientations_.push_front(
            previous_state.pose().rotation().toQuaternion());
        state.persistent_features_.clear();

        return state;
    };

    static void updateState(const gtsam::NavState& current_state,
                            const gtsam::NavState& previous_state,
                            const IMUMeasurement& imu_measurement,
                            const gtsam::imuBias::ConstantBias& imu_bias,
                            const gtsam::Pose3& radar_to_imu_transform,
                            State& state)
    {
        state.p_ = current_state.pose().translation();
        state.v_ = current_state.v();
        state.q_ = current_state.pose().rotation().toQuaternion();
        state.b_w_ = imu_bias.gyroscope();
        state.b_a_ = imu_bias.accelerometer();
        state.p_ri_ = radar_to_imu_transform.translation();
        state.q_ri_ = radar_to_imu_transform.rotation().toQuaternion();
        state.w_m_ = imu_measurement.w_m_;
        state.a_m_ = imu_measurement.a_m_;
        // Holds the last past pose used in trail matching.
        state.past_positions_.push_front(previous_state.pose().translation());
        state.past_orientations_.push_front(
            previous_state.pose().rotation().toQuaternion());
        // Persistent features are kept intact in this function.
        // PFs are removed in the points associator class.
    };
};

}  // namespace aaucns_rio

#endif /* _FILTER_STATE_FACTORY_H_ */
