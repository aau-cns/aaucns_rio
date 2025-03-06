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

#ifndef _STATE_H_
#define _STATE_H_

#include <aaucns_rio/DoubleArrayStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <vector>

#include "aaucns_rio/circular_buffer.h"
#include "aaucns_rio/config.h"
#include "aaucns_rio/parameters.h"
#include "aaucns_rio/trailpoint.h"

namespace aaucns_rio
{
/**
 * This class defines the state, its associated error state covariance and the
 * system inputs. The values in the braces determine the state's position in the
 * state vector / error state vector.
 */
class State
{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Error state size - we add seven artificial states in order to
    // reuse ssf implementation details such as state and covariance propagation
    // matrices.
    static constexpr std::size_t kNState = 21 + 7;
    // Nominal state.
    static constexpr std::size_t kNFullState = 23;
    // IMU state size.
    static constexpr std::size_t kNImuState = 15;
    // Calibration error states size.
    static constexpr std::size_t kNCalibState = 6;
    // State of the system with no cloned states yet.
    static constexpr std::size_t kNBaseMultiWindowState =
        kNImuState + kNCalibState;
    // Number of state variables of the augmented system.
    static constexpr std::size_t kNAugmentedState =
        kNBaseMultiWindowState + Config::kMaxPastElements * 6;

    State();

    // States varying during propagation.
    // Position (IMU centered) (0-2 / 0-2)
    Eigen::Matrix<double, 3, 1> p_;
    // Velocity (3-5 / 3-5)
    Eigen::Matrix<double, 3, 1> v_;
    // Attitude (6-9 / 6-8)
    Eigen::Quaternion<double> q_;
    // Gyro biases (10-12 / 9-11)
    Eigen::Matrix<double, 3, 1> b_w_;
    // Acceleration biases (13-15 / 12-14)
    Eigen::Matrix<double, 3, 1> b_a_;
    // Position of Radar wrt IMU (16-18 / 14-16)
    Eigen::Matrix<double, 3, 1> p_ri_;
    // Orientation of radar wrt IMU (19-22 / 17-19)
    Eigen::Quaternion<double> q_ri_;

    // states not varying during propagation.
    CircularBuffer<Eigen::Matrix<double, 3, 1>, Config::kMaxPastElements>
        past_positions_;
    CircularBuffer<Eigen::Quaternion<double>, Config::kMaxPastElements>
        past_orientations_;

    // Persistent features.
    // The corrected position is kept in the `most_recent_coordinates`
    // and the matched measurement in the matches history first row's
    // last three coordinates.
    std::vector<TrailPoint> persistent_features_;

    // System inputs.
    // Angular velocity from IMU.
    Eigen::Matrix<double, 3, 1> w_m_;
    ///< acceleration from IMU.
    Eigen::Matrix<double, 3, 1> a_m_;

    // Error state covariance.
    // Eigen::Matrix<double, kNState, kNState> P_;
    Eigen::MatrixXd P_;
    // Time of this state estimate.
    double time_;

    // resets the state to ->
    // 3D vectors: 0; quaternion: unit quaternion; time:0; Error
    // covariance: zeros.
    void reset();
    // Assembles a DoubleArrayStamped message from the state.
    // It does not set the header.
    void toStateMsg(aaucns_rio::DoubleArrayStampedPtr state);
    void getPoseCovariance(
        geometry_msgs::PoseWithCovariance::_covariance_type& cov);
    void toPoseMsg(geometry_msgs::PoseWithCovarianceStampedPtr pose);
    void toPoseNoCovMsg(geometry_msgs::PoseStampedPtr pose);
    void acceptPersistentFeature(const TrailPoint& trailpoint,
                                 const Parameters& parameters);
    std::vector<std::size_t> removePersistentFeaturesAndUpdateCovariance();
};

}  // namespace aaucns_rio

#endif /* _STATE_H_ */
