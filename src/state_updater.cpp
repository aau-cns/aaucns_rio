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

#include "aaucns_rio/state_updater.h"

#include <algorithm>
#include <cassert>
#include <utility>
#include <vector>

#include "aaucns_rio/debug.h"
#include "aaucns_rio/trail.h"
#include "aaucns_rio/trailpoint.h"
#include "aaucns_rio/util.h"

namespace aaucns_rio
{
void StateUpdater::augmentStateAndCovarianceMatrix(State& state)
{
    const int n_old_clones = state.getNClones();
    const int n_pf_state_variables = 3 * state.persistent_features_.size();
    const int old_pf_index = state.getPersistentFeaturesIndex();
    assert(old_pf_index ==
           static_cast<int>(State::getCloneIndex(n_old_clones)));
    // Once the window is full the oldest clone is dropped.
    const int n_kept_clones =
        std::min(n_old_clones, Config::kMaxPastElements - 1);

    // Add new past state to the state vector.
    state.past_positions_.push_front(state.p_);
    state.past_orientations_.push_front(state.q_);

    // Every error state after augmentation is a copy of one error state
    // before it: the base state, then the new clone (a copy of the current
    // position and orientation), then the kept clones shifted by one slot and
    // finally the persistent features. The new covariance is therefore the
    // old one re-indexed with `indices`, cross-correlations included.
    std::vector<int> indices;
    const std::size_t n_state_variables =
        State::getCloneIndex(state.getNClones()) + n_pf_state_variables;
    indices.reserve(n_state_variables);
    for (int i = 0; i < State::kNBaseMultiWindowState; ++i)
    {
        indices.push_back(i);
    }
    for (int i = 0; i < 3; ++i)
    {
        indices.push_back(i);
    }
    for (int i = 6; i < 9; ++i)
    {
        indices.push_back(i);
    }
    for (int i = 0; i < n_kept_clones * State::kNCloneState; ++i)
    {
        indices.push_back(State::getCloneIndex(0) + i);
    }
    for (int i = 0; i < n_pf_state_variables; ++i)
    {
        indices.push_back(old_pf_index + i);
    }
    assert(indices.size() == n_state_variables);

    Eigen::MatrixXd P(indices.size(), indices.size());
    for (int col = 0; col < P.cols(); ++col)
    {
        for (int row = 0; row < P.rows(); ++row)
        {
            P(row, col) = state.P_(indices[row], indices[col]);
        }
    }
    state.P_ = std::move(P);
}

bool StateUpdater::applyAllMeasurements(
    Features& features, const Eigen::MatrixXd& velocities_and_points,
    const Parameters& parameters, State& closest_to_measurement_state)
{
    // Call prepareMatricesForUpdate() and
    // get the matrices ready.
    // Jacobian of h(x) wrt error state variables.
    Eigen::MatrixXd H;
    // Measurement residual.
    // n_of_measurements x 1 - dynamic.
    Eigen::MatrixXd r;
    // Jacobian of h(x) wrt noises.
    // n_of_measurements x n_of_measurements - dynamic.
    Eigen::MatrixXd R;

    prepareMatricesForUpdate(parameters, closest_to_measurement_state, features,
                             H, R, r);

    Eigen::MatrixXd H_pf;
    Eigen::MatrixXd r_pf;
    Eigen::MatrixXd R_pf;

    prepareMatricesForPersistentFeaturesUpdate(
        parameters, closest_to_measurement_state, H_pf, R_pf, r_pf);

    Eigen::MatrixXd H_velocity;
    Eigen::MatrixXd r_velocity;
    Eigen::MatrixXd R_velocity;

    prepareMatricesForVelocityUpdate(parameters, closest_to_measurement_state,
                                     velocities_and_points, H_velocity,
                                     R_velocity, r_velocity);

    // Concatenate all matrices.
    Eigen::MatrixXd H_full(H.rows() + H_velocity.rows() + H_pf.rows(),
                           closest_to_measurement_state.getNStateVariables());
    Eigen::MatrixXd r_full(H.rows() + H_velocity.rows() + H_pf.rows(), 1);

    std::vector<Eigen::MatrixXd> list_of_H = {H, H_velocity, H_pf};
    util::concatenateMatricesVertically(H_full, list_of_H);
    std::vector<Eigen::MatrixXd> list_of_r = {r, r_velocity, r_pf};
    util::concatenateMatricesVertically(r_full, list_of_r);
    std::vector<Eigen::MatrixXd> list_of_R = {R, R_velocity, R_pf};
    Eigen::MatrixXd R_full = util::makeBlockDiagonal(list_of_R);

    if (!(H_full.rows() > 0))
    {
        return false;
    }

    // Do the update using EKF equations and
    // calculate the correction.
    const std::size_t n_state_variables =
        closest_to_measurement_state.getNStateVariables();
    Eigen::MatrixXd S(R_full.rows(), R_full.cols());
    Eigen::MatrixXd K(n_state_variables, R_full.rows());
    S = H_full * closest_to_measurement_state.P_ * H_full.transpose() + R_full;
    K = closest_to_measurement_state.P_ * H_full.transpose() * S.inverse();
    Eigen::MatrixXd correction(n_state_variables, 1);
    correction = K * r_full;
    Eigen::MatrixXd KH(n_state_variables, n_state_variables);
    const Eigen::MatrixXd identity =
        Eigen::MatrixXd::Identity(n_state_variables, n_state_variables);
    KH = (identity - K * H_full);
    closest_to_measurement_state.P_ =
        KH * closest_to_measurement_state.P_ * KH.transpose() +
        K * R_full * K.transpose();
    // Make sure P stays symmetric.
    closest_to_measurement_state.P_ =
        0.5 * (closest_to_measurement_state.P_.eval() +
               closest_to_measurement_state.P_.transpose().eval());
    util::fixEigenvalues(closest_to_measurement_state.P_);

    applyCorrection(correction, closest_to_measurement_state);
    return true;
}

void StateUpdater::applyCorrection(const Eigen::MatrixXd& correction,
                                   State& closest_to_measurement_state)
{
    // Retrieve the state for which the correction
    // is being applied and apply it. Also, take the imu state after correction
    // and write it as the previous radar pose.
    closest_to_measurement_state.p_ =
        closest_to_measurement_state.p_ + correction.block<3, 1>(0, 0);
    closest_to_measurement_state.v_ =
        closest_to_measurement_state.v_ + correction.block<3, 1>(3, 0);
    closest_to_measurement_state.b_w_ =
        closest_to_measurement_state.b_w_ + correction.block<3, 1>(9, 0);
    closest_to_measurement_state.b_a_ =
        closest_to_measurement_state.b_a_ + correction.block<3, 1>(12, 0);

    const Eigen::Quaternion<double> qdelta_q =
        util::quaternionFromSmallAngle(correction.block<3, 1>(6, 0));
    closest_to_measurement_state.q_ =
        closest_to_measurement_state.q_ * qdelta_q;
    closest_to_measurement_state.q_.normalize();

    closest_to_measurement_state.p_ri_ =
        closest_to_measurement_state.p_ri_ + correction.block<3, 1>(15, 0);
    const Eigen::Quaternion<double> qdelta_q_ri =
        util::quaternionFromSmallAngle(correction.block<3, 1>(18, 0));
    closest_to_measurement_state.q_ri_ =
        closest_to_measurement_state.q_ri_ * qdelta_q_ri;
    closest_to_measurement_state.q_ri_.normalize();

    // Correct the past poses.
    for (int i = 0; i < closest_to_measurement_state.past_positions_.size();
         ++i)
    {
        closest_to_measurement_state.past_positions_[i] =
            closest_to_measurement_state.past_positions_[i] +
            correction.block<3, 1>(State::kNBaseMultiWindowState + i * 6, 0);

        const Eigen::Quaternion<double> past_qdelta_q =
            util::quaternionFromSmallAngle(correction.block<3, 1>(
                State::kNBaseMultiWindowState + (i * 6 + 3), 0));
        closest_to_measurement_state.past_orientations_[i] =
            closest_to_measurement_state.past_orientations_[i] * past_qdelta_q;
        closest_to_measurement_state.past_orientations_[i].normalize();
    }
    // Correct persistent features.
    for (int i = 0;
         i < closest_to_measurement_state.persistent_features_.size(); ++i)
    {
        closest_to_measurement_state.persistent_features_[i]
            .most_recent_coordinates =
            closest_to_measurement_state.persistent_features_[i]
                .most_recent_coordinates.eval() +
            correction
                .block<3, 1>(
                    closest_to_measurement_state.getPersistentFeaturesIndex() +
                        i * 3,
                    0)
                .transpose();
    }
}

void StateUpdater::prepareMatricesForUpdate(
    const Parameters& parameters, const State& closest_to_measurement_state,
    Features& features, Eigen::MatrixXd& H, Eigen::MatrixXd& R,
    Eigen::MatrixXd& r)
{
    constexpr double kChiSquare1DoFThresholdHigh = 5.02;
    constexpr double kChiSquare1DoFThresholdLow = 0.0;
    const double s_zp = parameters.noise_meas1_ * parameters.noise_meas1_;
    int H_row_index = 0;
    for (int j = 0; j < Config::kMaxPastElements; ++j)
    {
        if (features.positions_and_matched_or_not_[j].true_or_false)
        {
            for (int i = 0;
                 i < features.positions_and_matched_or_not_[j].positions.rows();
                 ++i)
            {
                // For jacobian computation use untransformed points from the
                // previous frames.
                const Eigen::MatrixXd candidate_row =
                    getJacobianForSingleFeature(
                        parameters, closest_to_measurement_state,
                        features.positions_and_matched_or_not_[j].positions.row(
                            i),
                        j);
                // Transform trailpoints into the correct frame.
                features.positions_and_matched_or_not_[j].positions.row(i).head(
                    3) =
                    Trail::rotateAndTranslateSingleVectorAtIndex(
                        closest_to_measurement_state, parameters, j,
                        features.positions_and_matched_or_not_[j]
                            .positions.row(i)
                            .head(3)
                            .eval());
                const double candidate_residual =
                    features.positions_and_matched_or_not_[j]
                        .positions.row(i)
                        .tail(3)
                        .norm() -
                    features.positions_and_matched_or_not_[j]
                        .positions.row(i)
                        .head(3)
                        .norm();

                // Run chi-square.
                const Eigen::MatrixXd hsht = candidate_row *
                                             closest_to_measurement_state.P_ *
                                             candidate_row.transpose();

                const double s = hsht(0, 0) + s_zp;

                const double chi_squared =
                    candidate_residual * (1.0 / s) * candidate_residual;

                if (chi_squared < kChiSquare1DoFThresholdHigh &&
                    chi_squared > kChiSquare1DoFThresholdLow)
                {
                    ++H_row_index;
                    // Prepare H.

                    H.conservativeResize(
                        H_row_index,
                        closest_to_measurement_state.getNStateVariables());

                    H.row(H_row_index - 1) = candidate_row;
                    // Prepare R.
                    R.conservativeResize(H_row_index, H_row_index);
                    R = s_zp *
                        Eigen::MatrixXd::Identity(H_row_index, H_row_index);
                    // Prepare r.
                    r.conservativeResize(H_row_index, 1);
                    r(H_row_index - 1, 0) = candidate_residual;
                }
            }
        }
    }
}

void StateUpdater::prepareMatricesForPersistentFeaturesUpdate(
    const Parameters& parameters, const State& closest_to_measurement_state,
    Eigen::MatrixXd& H_pf, Eigen::MatrixXd& R_pf, Eigen::MatrixXd& r_pf)
{
    constexpr double kChiSquare1DoFThresholdHigh = 5.02;
    // No lower bound: rejecting a "too good" (suspiciously small) normalized
    // residual is statistically backwards for an EKF -- a small residual
    // means the measurement agrees with P_, not that it's untrustworthy. If
    // P_ is inflated, this is exactly the correction that should shrink it
    // back down; rejecting it instead removes the only way back, producing
    // a self-reinforcing covariance-growth spiral. Matches the trail/
    // position update (prepareMatricesForUpdate), which never had a lower
    // bound.
    constexpr double kChiSquare1DoFThresholdLow = 0.0;
    const double s_zp = parameters.noise_meas4_ * parameters.noise_meas4_;
    int H_row_index = 0;
    for (int i = 0;
         i < closest_to_measurement_state.persistent_features_.size(); ++i)
    {
        const Eigen::MatrixXd candidate_row =
            getPersistentFeatureJacobianForSingleFeature(
                parameters, closest_to_measurement_state, i);

        const double candidate_residual =
            evaluatePersistentFeatureMeasurementEquation(
                parameters, closest_to_measurement_state, i)
                .norm() -
            closest_to_measurement_state.persistent_features_[i]
                .matches_history[0]
                .head(3)
                .norm();

        // Run chi-square.
        const Eigen::MatrixXd hsht = candidate_row *
                                     closest_to_measurement_state.P_ *
                                     candidate_row.transpose();

        const double s = hsht(0, 0) + s_zp;
        const double chi_squared =
            candidate_residual * (1.0 / s) * candidate_residual;

        if (chi_squared < kChiSquare1DoFThresholdHigh &&
            chi_squared > kChiSquare1DoFThresholdLow)
        {
            ++H_row_index;
            H_pf.conservativeResize(
                H_row_index,
                closest_to_measurement_state.getNStateVariables());
            H_pf.row(H_row_index - 1) = candidate_row;
            // Prepare r.
            r_pf.conservativeResize(H_row_index, 1);
            r_pf(H_row_index - 1, 0) = candidate_residual;
            // Prepare R.
            R_pf.conservativeResize(H_row_index, H_row_index);
            R_pf = s_zp * Eigen::MatrixXd::Identity(H_row_index, H_row_index);
        }
    }
}

void StateUpdater::prepareMatricesForVelocityUpdate(
    const Parameters& parameters, const State& closest_to_measurement_state,
    const Eigen::MatrixXd& velocities_and_points, Eigen::MatrixXd& H,
    Eigen::MatrixXd& R, Eigen::MatrixXd& r)
{
    constexpr double kChiSquare1DoFThresholdHigh = 5.02;
    // See prepareMatricesForPersistentFeaturesUpdate for why this is 0.0,
    // not the 2.5th-percentile value it used to be.
    constexpr double kChiSquare1DoFThresholdLow = 0.0;
    const double s_zp_v = parameters.noise_meas3_ * parameters.noise_meas3_;
    int H_row_index = 0;
    for (int i = 0; i < velocities_and_points.rows(); ++i)
    {
        const Eigen::MatrixXd candidate_row =
            getVelocityJacobianForSingleFeature(parameters,
                                                closest_to_measurement_state,
                                                velocities_and_points.row(i));
        const double candidate_residual =
            velocities_and_points.row(i)(0) -
            evaluateVelocityMeasurementEquation(parameters,
                                                closest_to_measurement_state,
                                                velocities_and_points.row(i));
        // Run chi-square.

        const Eigen::MatrixXd hsht = candidate_row *
                                     closest_to_measurement_state.P_ *
                                     candidate_row.transpose();
        const double s = hsht(0, 0) + s_zp_v;

        const double chi_squared =
            candidate_residual * (1.0 / s) * candidate_residual;
        if (chi_squared < kChiSquare1DoFThresholdHigh &&
            chi_squared > kChiSquare1DoFThresholdLow)
        {
            ++H_row_index;
            H.conservativeResize(
                H_row_index,
                closest_to_measurement_state.getNStateVariables());
            H.row(H_row_index - 1) = candidate_row;
            // Prepare r.
            r.conservativeResize(H_row_index, 1);
            r(H_row_index - 1, 0) = candidate_residual;
            // Prepare R.
            R.conservativeResize(H_row_index, H_row_index);
            R = s_zp_v * Eigen::MatrixXd::Identity(H_row_index, H_row_index);
        }
    }
}

double StateUpdater::evaluateVelocityMeasurementEquation(
    const Parameters& parameters, const State& closest_to_measurement_state,
    const Eigen::VectorXd& single_velocity_and_point)
{
    Eigen::Vector3d radar_velocity_in_radar_frame =
        closest_to_measurement_state.q_ri_.conjugate().toRotationMatrix() *
            closest_to_measurement_state.q_.conjugate().toRotationMatrix() *
            closest_to_measurement_state.v_ +
        closest_to_measurement_state.q_ri_.conjugate().toRotationMatrix() *
            util::getSkewSymmetricMat(closest_to_measurement_state.w_m_ -
                                      closest_to_measurement_state.b_w_) *
            closest_to_measurement_state.p_ri_;
    const double velocity_from_state_evaluated =
        (single_velocity_and_point.tail(3).transpose().eval() /
         single_velocity_and_point.tail(3).norm()) *
        radar_velocity_in_radar_frame;
    return velocity_from_state_evaluated;
}

Eigen::Vector3d StateUpdater::evaluatePersistentFeatureMeasurementEquation(
    const Parameters& parameters, const State& closest_to_measurement_state,
    const int index)
{
    Eigen::Vector3d h_evaluated =
        closest_to_measurement_state.q_ri_.conjugate().toRotationMatrix() *
        (closest_to_measurement_state.q_.conjugate().toRotationMatrix() *
             (closest_to_measurement_state.persistent_features_[index]
                  .most_recent_coordinates.transpose() -
              closest_to_measurement_state.p_) -
         closest_to_measurement_state.p_ri_);
    return h_evaluated;
}

Eigen::MatrixXd StateUpdater::getPersistentFeatureJacobianForSingleFeature(
    const Parameters& parameters, const State& closest_to_measurement_state,
    const int index)
{
    Eigen::MatrixXd H_not_reduced(
        3, closest_to_measurement_state.getNStateVariables());
    H_not_reduced.setZero();
    // H(0, 0).
    H_not_reduced.block<3, 3>(0, 0) =
        -closest_to_measurement_state.q_ri_.conjugate().toRotationMatrix() *
        closest_to_measurement_state.q_.conjugate().toRotationMatrix();
    // H(0, 1) = 0.
    // H(0, 2).
    const Eigen::Vector3d to_skew =
        closest_to_measurement_state.q_.conjugate().toRotationMatrix() *
        (closest_to_measurement_state.persistent_features_[index]
             .most_recent_coordinates.transpose() -
         closest_to_measurement_state.p_);
    H_not_reduced.block<3, 3>(0, 6) =
        closest_to_measurement_state.q_ri_.conjugate().toRotationMatrix() *
        util::getSkewSymmetricMat(to_skew);
    // H(0, 3) = 0.
    // H(0, 4) = 0.
    // H(0, 5) = 0.
    H_not_reduced.block<3, 3>(0, 15) =
        -closest_to_measurement_state.q_ri_.conjugate().toRotationMatrix();
    // H(0, 6) = 0.
    const Eigen::Vector3d to_skew_1 =
        closest_to_measurement_state.q_ri_.conjugate().toRotationMatrix() *
        closest_to_measurement_state.q_.conjugate().toRotationMatrix() *
        (closest_to_measurement_state.persistent_features_[index]
             .most_recent_coordinates.transpose() -
         closest_to_measurement_state.p_);
    const Eigen::Vector3d to_skew_2 =
        closest_to_measurement_state.q_ri_.conjugate().toRotationMatrix() *
        closest_to_measurement_state.p_ri_;
    H_not_reduced.block<3, 3>(0, 18) = util::getSkewSymmetricMat(to_skew_1) -
                                       util::getSkewSymmetricMat(to_skew_2);
    // H(0, N * clones) = 0.
    // H(0, index).
    H_not_reduced.block<3, 3>(
        0, closest_to_measurement_state.getPersistentFeaturesIndex() +
               index * 3) =
        closest_to_measurement_state.q_ri_.conjugate().toRotationMatrix() *
        closest_to_measurement_state.q_.conjugate().toRotationMatrix();

    const Eigen::Vector3d h_evaluated =
        evaluatePersistentFeatureMeasurementEquation(
            parameters, closest_to_measurement_state, index);
    return (h_evaluated.transpose() / h_evaluated.norm()) * H_not_reduced;
}

Eigen::MatrixXd StateUpdater::getVelocityJacobianForSingleFeature(
    const Parameters& parameters, const State& closest_to_measurement_state,
    const Eigen::VectorXd& single_velocity_and_point)
{
    Eigen::MatrixXd H_not_reduced(
        3, closest_to_measurement_state.getNStateVariables());
    H_not_reduced.setZero();
    // H(0, 0) = 0.
    // H(0, 1).
    H_not_reduced.block<3, 3>(0, 3) =
        closest_to_measurement_state.q_ri_.conjugate().toRotationMatrix() *
        closest_to_measurement_state.q_.conjugate().toRotationMatrix();
    // H(0, 2).
    const Eigen::Vector3d to_skew =
        closest_to_measurement_state.q_.conjugate().toRotationMatrix() *
        closest_to_measurement_state.v_;
    H_not_reduced.block<3, 3>(0, 6) =
        closest_to_measurement_state.q_ri_.conjugate().toRotationMatrix() *
        util::getSkewSymmetricMat(to_skew);
    // H(0, 3).
    H_not_reduced.block<3, 3>(0, 9) =
        closest_to_measurement_state.q_ri_.conjugate().toRotationMatrix() *
        util::getSkewSymmetricMat(closest_to_measurement_state.p_ri_);
    // H(0, 4) = 0.
    // H(0, 5).
    H_not_reduced.block<3, 3>(0, 15) =
        closest_to_measurement_state.q_ri_.conjugate().toRotationMatrix() *
        util::getSkewSymmetricMat(closest_to_measurement_state.w_m_ -
                                  closest_to_measurement_state.b_w_);
    // H(0, 6).
    const Eigen::Vector3d to_skew_1 =
        closest_to_measurement_state.q_ri_.conjugate().toRotationMatrix() *
        closest_to_measurement_state.q_.conjugate().toRotationMatrix() *
        closest_to_measurement_state.v_;
    const Eigen::Vector3d to_skew_2 =
        closest_to_measurement_state.q_ri_.conjugate().toRotationMatrix() *
        util::getSkewSymmetricMat(closest_to_measurement_state.w_m_ -
                                  closest_to_measurement_state.b_w_) *
        closest_to_measurement_state.p_ri_;
    H_not_reduced.block<3, 3>(0, 18) = util::getSkewSymmetricMat(to_skew_1) +
                                       util::getSkewSymmetricMat(to_skew_2);
    // H(0, 7) = 0.
    // H(0, 8) = 0.
    return (single_velocity_and_point.tail(3).transpose().eval() /
            single_velocity_and_point.tail(3).norm()) *
           H_not_reduced;
}

Eigen::MatrixXd StateUpdater::getJacobianForSingleFeature(
    const Parameters& parameters, const State& closest_to_measurement_state,
    const Eigen::VectorXd& single_matched_feature, const int index)
{
    Eigen::MatrixXd H_not_reduced(
        3, closest_to_measurement_state.getNStateVariables());
    H_not_reduced.setZero();
    // H(0, 0)
    H_not_reduced.block<3, 3>(0, 0) =
        -closest_to_measurement_state.q_ri_.conjugate().toRotationMatrix() *
        closest_to_measurement_state.q_.conjugate().toRotationMatrix();
    // H(0, 1) = 0
    // H(0, 2)
    const Eigen::Vector3d to_skew =
        -closest_to_measurement_state.q_.conjugate().toRotationMatrix() *
        (closest_to_measurement_state.past_orientations_[index]
                 .toRotationMatrix() *
             (closest_to_measurement_state.q_ri_.toRotationMatrix() *
                  single_matched_feature.head(3) +
              closest_to_measurement_state.p_ri_) -
         closest_to_measurement_state.p_ +
         closest_to_measurement_state.past_positions_[index]);
    H_not_reduced.block<3, 3>(0, 6) =
        -closest_to_measurement_state.q_ri_.conjugate().toRotationMatrix() *
        util::getSkewSymmetricMat(to_skew);
    // H(0, 3) = 0
    // H(0, 4) = 0
    // H(0, 5) = 0 - calibration position.
    H_not_reduced.block<3, 3>(0, 15) =
        -closest_to_measurement_state.q_ri_.conjugate().toRotationMatrix() +
        closest_to_measurement_state.q_ri_.conjugate().toRotationMatrix() *
            closest_to_measurement_state.q_.conjugate().toRotationMatrix() *
            closest_to_measurement_state.past_orientations_[index]
                .toRotationMatrix();
    // H(0, 6) = 0 - calibration - orientation.
    const Eigen::Vector3d to_skew_1 =
        closest_to_measurement_state.q_ri_.conjugate().toRotationMatrix() *
        closest_to_measurement_state.p_ri_;
    const Eigen::Vector3d to_skew_2 =
        closest_to_measurement_state.q_ri_.conjugate().toRotationMatrix() *
        closest_to_measurement_state.q_.conjugate().toRotationMatrix() *
        closest_to_measurement_state.p_;
    const Eigen::Vector3d to_skew_3 =
        closest_to_measurement_state.q_ri_.conjugate().toRotationMatrix() *
        closest_to_measurement_state.q_.conjugate().toRotationMatrix() *
        closest_to_measurement_state.past_positions_[index];
    const Eigen::Vector3d to_skew_4 =
        closest_to_measurement_state.q_ri_.conjugate().toRotationMatrix() *
        closest_to_measurement_state.q_.conjugate().toRotationMatrix() *
        closest_to_measurement_state.past_orientations_[index]
            .toRotationMatrix() *
        closest_to_measurement_state.p_ri_;
    const Eigen::Vector3d to_skew_5 =
        closest_to_measurement_state.q_ri_.conjugate().toRotationMatrix() *
        closest_to_measurement_state.q_.conjugate().toRotationMatrix() *
        closest_to_measurement_state.past_orientations_[index]
            .toRotationMatrix() *
        closest_to_measurement_state.q_ri_.toRotationMatrix() *
        single_matched_feature.head(3);
    H_not_reduced.block<3, 3>(0, 18) = -util::getSkewSymmetricMat(to_skew_1) -
                                       util::getSkewSymmetricMat(to_skew_2) +
                                       util::getSkewSymmetricMat(to_skew_3) +
                                       util::getSkewSymmetricMat(to_skew_4) +
                                       util::getSkewSymmetricMat(to_skew_5);
    // The five terms above reduce exactly to [h(0)]x, which is only the
    // derivative contribution from q_ri's OUTER occurrence (the leading
    // R_ri^-1 coefficient). q_ri also appears a second, NESTED time inside
    // R_past*(R_ri*z+p_ri) (the local observation is projected into world
    // frame via the past clone using the SAME calibration rotation) -- that
    // occurrence's contribution, -R_ri^-1*R^-1*R_past*R_ri*[z]x, was missing
    // entirely. (Does not apply to the persistent-feature or velocity
    // Jacobians: there, q_ri appears only once, as the outer coefficient, so
    // [h(0)]x is the complete answer for those.)
    const Eigen::Matrix<double, 3, 3> q_ri_inner_term =
        closest_to_measurement_state.q_ri_.conjugate().toRotationMatrix() *
        closest_to_measurement_state.q_.conjugate().toRotationMatrix() *
        closest_to_measurement_state.past_orientations_[index]
            .toRotationMatrix() *
        closest_to_measurement_state.q_ri_.toRotationMatrix();
    const Eigen::Vector3d local_observation = single_matched_feature.head(3);
    H_not_reduced.block<3, 3>(0, 18) -=
        q_ri_inner_term * util::getSkewSymmetricMat(local_observation);
    // H(0, 7)
    H_not_reduced.block<3, 3>(0, State::kNBaseMultiWindowState + index * 6) =
        closest_to_measurement_state.q_ri_.conjugate().toRotationMatrix() *
        closest_to_measurement_state.q_.conjugate().toRotationMatrix();
    // H(0, 8)
    H_not_reduced.block<3, 3>(0,
                              State::kNBaseMultiWindowState + (index * 6 + 3)) =
        closest_to_measurement_state.q_ri_.conjugate().toRotationMatrix() *
        closest_to_measurement_state.q_.conjugate().toRotationMatrix() *
        closest_to_measurement_state.past_orientations_[index]
            .toRotationMatrix() *
        util::getSkewSymmetricMat(
            -closest_to_measurement_state.q_ri_.toRotationMatrix() *
                single_matched_feature.head(3) -
            closest_to_measurement_state.p_ri_);
    // Reduce the jacobian.
    Eigen::Vector3d h_evaluated =
        closest_to_measurement_state.q_ri_.conjugate().toRotationMatrix() *
        (closest_to_measurement_state.q_.conjugate().toRotationMatrix() *
             (closest_to_measurement_state.past_orientations_[index]
                      .toRotationMatrix() *
                  (closest_to_measurement_state.q_ri_.toRotationMatrix() *
                       single_matched_feature.head(3) +
                   closest_to_measurement_state.p_ri_) -
              closest_to_measurement_state.p_ +
              closest_to_measurement_state.past_positions_[index]) -
         closest_to_measurement_state.p_ri_);
    return (h_evaluated.transpose() / h_evaluated.norm()) * H_not_reduced;
}

}  // namespace aaucns_rio
