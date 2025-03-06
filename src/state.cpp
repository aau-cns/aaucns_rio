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

#include "aaucns_rio/state.h"

#include <algorithm>

#include "aaucns_rio/config.h"
#include "aaucns_rio/debug.h"
#include "aaucns_rio/util.h"

namespace aaucns_rio
{
constexpr std::size_t State::kNState;
constexpr std::size_t State::kNImuState;

State::State() { reset(); }

void State::reset()
{
    // States varying during propagation.
    p_.setZero();
    v_.setZero();
    q_.setIdentity();
    b_w_.setZero();
    b_a_.setZero();
    w_m_.setZero();
    a_m_.setZero();
    p_ri_.setZero();
    q_ri_.setIdentity();
    P_.resize(kNAugmentedState, kNAugmentedState);
    P_.setZero();
    past_positions_.reset();
    past_orientations_.reset();
    // Reset persistent features.
    persistent_features_.clear();
    time_ = 0;
}

void State::toStateMsg(aaucns_rio::DoubleArrayStampedPtr state)
{
    state->data[0] = p_[0];
    state->data[1] = p_[1];
    state->data[2] = p_[2];
    state->data[3] = v_[0];
    state->data[4] = v_[1];
    state->data[5] = v_[2];
    state->data[6] = q_.w();
    state->data[7] = q_.x();
    state->data[8] = q_.y();
    state->data[9] = q_.z();
    state->data[10] = b_w_[0];
    state->data[11] = b_w_[1];
    state->data[12] = b_w_[2];
    state->data[13] = b_a_[0];
    state->data[14] = b_a_[1];
    state->data[15] = b_a_[2];
    state->data[16] = p_ri_[0];
    state->data[17] = p_ri_[1];
    state->data[18] = p_ri_[2];
    state->data[19] = q_ri_.w();
    state->data[20] = q_ri_.x();
    state->data[21] = q_ri_.y();
    state->data[22] = q_ri_.z();
}

void State::getPoseCovariance(
    geometry_msgs::PoseWithCovariance::_covariance_type& cov)
{
    assert(cov.size() == 36);
    Eigen::Matrix<double, 6, 6> P;
    P.block<3, 3>(0, 0) = P_.block<3, 3>(0, 0);
    P.block<3, 3>(3, 3) = P_.block<3, 3>(6, 6);
    P.block<3, 3>(3, 0) = P_.block<3, 3>(6, 0);
    P.block<3, 3>(0, 3) = P_.block<3, 3>(0, 6);
    P.transposeInPlace();
    Eigen::VectorXd B(
        Eigen::Map<Eigen::VectorXd>(P.data(), P.cols() * P.rows()));
    for (int i = 0; i < B.size(); ++i)
    {
        cov[i] = B(i);
    }
}

void State::toPoseMsg(geometry_msgs::PoseWithCovarianceStampedPtr pose)
{
    util::vector3dToPoint(p_, pose->pose.pose.position);
    util::quaternionToMsg(q_, pose->pose.pose.orientation);
    getPoseCovariance(pose->pose.covariance);
}

void State::toPoseNoCovMsg(geometry_msgs::PoseStampedPtr pose)
{
    util::vector3dToPoint(p_, pose->pose.position);
    util::quaternionToMsg(q_, pose->pose.orientation);
}

std::vector<std::size_t> State::removePersistentFeaturesAndUpdateCovariance()
{
    std::vector<std::size_t> removed_pfs_indexes;
    for (int i = 0; i < persistent_features_.size(); ++i)
    {
        if (!persistent_features_[i].is_active)
        {
            if (i == (persistent_features_.size() - 1))
            {
                // Do nothing - just resize the matrix.
            }
            else
            {
                // Update covariance matrix.
                P_.block(kNAugmentedState + 3 * i, kNAugmentedState + 3 * i,
                         3 * (persistent_features_.size() - 1 - i),
                         3 * (persistent_features_.size() - 1 - i)) =
                    P_.block(kNAugmentedState + 3 * (i + 1),
                             kNAugmentedState + 3 * (i + 1),
                             3 * (persistent_features_.size() - 1 - i),
                             3 * (persistent_features_.size() - 1 - i))
                        .eval();
                P_.block(kNAugmentedState + 3 * i, 0,
                         3 * (persistent_features_.size() - 1 - i),
                         kNAugmentedState + 3 * i) =
                    P_.block(kNAugmentedState + 3 * (i + 1), 0,
                             3 * (persistent_features_.size() - 1 - i),
                             kNAugmentedState + 3 * i)
                        .eval();
                P_.block(0, kNAugmentedState + 3 * i, kNAugmentedState + 3 * i,
                         3 * (persistent_features_.size() - 1 - i)) =
                    P_.block(0, kNAugmentedState + 3 * (i + 1),
                             kNAugmentedState + 3 * i,
                             3 * (persistent_features_.size() - 1 - i))
                        .eval();
            }
            P_.conservativeResize(
                kNAugmentedState + 3 * (persistent_features_.size() - 1),
                kNAugmentedState + 3 * (persistent_features_.size() - 1));
            // Remove pf.
            removed_pfs_indexes.push_back(persistent_features_[i].id);
            persistent_features_.erase(persistent_features_.begin() + i);
            --i;
        }
    }
    return removed_pfs_indexes;
}

void State::acceptPersistentFeature(const TrailPoint& trailpoint,
                                    const Parameters& parameters)
{
    persistent_features_.push_back(trailpoint);

    // Update covariance matrix after adding a persistent feature.
    // Terms (for states p, q) of the Jacobian of the transformation function of
    // a pf from local to global.
    Eigen::MatrixXd H_rr_pf(3, kNAugmentedState);

    H_rr_pf.setZero();
    H_rr_pf.block<3, 3>(0, 0) = Eigen::MatrixXd::Identity(3, 3);
    H_rr_pf.block<3, 3>(0, 6) =
        -q_.toRotationMatrix() *
        util::getSkewSymmetricMat(
            q_ri_.toRotationMatrix() *
                trailpoint.most_recent_coordinates.transpose() +
            p_ri_);
    // Term for pf itself.
    Eigen::MatrixXd H_pi_pf(3, 3);
    H_pi_pf.setZero();
    H_pi_pf = q_.toRotationMatrix() * q_ri_.toRotationMatrix();

    // Term for noise of the pf measurement.
    Eigen::MatrixXd R_pf = Eigen::MatrixXd::Identity(3, 3);
    Eigen::Matrix<double, 3, 1> diagonal;
    diagonal << parameters.noise_pf_x_ * parameters.noise_pf_x_,
        parameters.noise_pf_y_ * parameters.noise_pf_y_,
        parameters.noise_pf_z_ * parameters.noise_pf_z_;
    R_pf.diagonal() = diagonal;

    const Eigen::MatrixXd P_pf =
        H_rr_pf * P_.block<kNAugmentedState, kNAugmentedState>(0, 0) *
            H_rr_pf.transpose() +
        H_pi_pf * R_pf * H_pi_pf.transpose();

    // Place the new pf covariance block in the covariance matrix.
    P_.conservativeResize(P_.rows() + 3, P_.cols() + 3);
    P_.block<3, 3>(P_.rows() - 3, P_.cols() - 3) = P_pf;

    P_.block(kNAugmentedState + 3 * (persistent_features_.size() - 1), 0, 3,
             kNAugmentedState + 3 * (persistent_features_.size() - 1)) =
        H_rr_pf *
        P_.block(0, 0, kNAugmentedState,
                 kNAugmentedState + 3 * (persistent_features_.size() - 1))
            .eval();

    P_.block(0, kNAugmentedState + 3 * (persistent_features_.size() - 1),
             kNAugmentedState + 3 * (persistent_features_.size() - 1), 3) =
        P_.block(kNAugmentedState + 3 * (persistent_features_.size() - 1), 0, 3,
                 kNAugmentedState + 3 * (persistent_features_.size() - 1))
            .transpose()
            .eval();
}

}  // namespace aaucns_rio
