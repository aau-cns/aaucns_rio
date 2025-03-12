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

#include "aaucns_rio/debug.h"
#include "aaucns_rio/trail.h"
#include "aaucns_rio/trailpoint.h"
#include "aaucns_rio/util.h"

namespace aaucns_rio
{
void StateUpdater::augmentStateAndCovarianceMatrix(State& state)
{
    // Add new past state to the state vector.
    state.past_positions_.push_front(state.p_);
    state.past_orientations_.push_front(state.q_);

    if (Config::kMaxPastElements > 1)
    {
        // First step - covariances and correlations one step forwards.
        state.P_.block<(Config::kMaxPastElements - 1) * 6,
                       (Config::kMaxPastElements - 1) * 6>(
            State::kNBaseMultiWindowState + 6,
            State::kNBaseMultiWindowState + 6) =
            state.P_
                .block<(Config::kMaxPastElements - 1) * 6,
                       (Config::kMaxPastElements - 1) * 6>(
                    State::kNBaseMultiWindowState,
                    State::kNBaseMultiWindowState)
                .eval();
        // Second step - copy corss-correlations between the newest clone and
        // the oldest one - cols - position.
        state.P_.block<(Config::kMaxPastElements - 1) * 6, 3>(
            State::kNBaseMultiWindowState + 6, State::kNBaseMultiWindowState) =
            state.P_
                .block<(Config::kMaxPastElements - 1) * 6, 3>(
                    State::kNBaseMultiWindowState, 0)
                .eval();
        // Orientation.
        state.P_.block<(Config::kMaxPastElements - 1) * 6, 3>(
            State::kNBaseMultiWindowState + 6,
            State::kNBaseMultiWindowState + 3) =
            state.P_
                .block<(Config::kMaxPastElements - 1) * 6, 3>(
                    State::kNBaseMultiWindowState, 6)
                .eval();
        // Rows - position.
        state.P_.block<3, (Config::kMaxPastElements - 1) * 6>(
            State::kNBaseMultiWindowState, State::kNBaseMultiWindowState + 6) =
            state.P_
                .block<3, (Config::kMaxPastElements - 1) * 6>(
                    0, State::kNBaseMultiWindowState)
                .eval();
        state.P_.block<3, (Config::kMaxPastElements - 1) * 6>(
            State::kNBaseMultiWindowState + 3,
            State::kNBaseMultiWindowState + 6) =
            state.P_
                .block<3, (Config::kMaxPastElements - 1) * 6>(
                    6, State::kNBaseMultiWindowState)
                .eval();
        // Third step - copy cross-correlations between imu states and clones.
        // Rows.
        state.P_.block<(Config::kMaxPastElements - 1) * 6,
                       State::kNBaseMultiWindowState>(
            State::kNBaseMultiWindowState + 6, 0) =
            state.P_
                .block<(Config::kMaxPastElements - 1) * 6,
                       State::kNBaseMultiWindowState>(
                    State::kNBaseMultiWindowState, 0)
                .eval();
        // Cols.
        state.P_.block<State::kNBaseMultiWindowState,
                       (Config::kMaxPastElements - 1) * 6>(
            0, State::kNBaseMultiWindowState + 6) =
            state.P_
                .block<State::kNBaseMultiWindowState,
                       (Config::kMaxPastElements - 1) * 6>(
                    0, State::kNBaseMultiWindowState)
                .eval();
    }
    // Fourth step - clone from the current pose to the first-after-imu matrix
    // block. Covariances - position.
    state.P_.block<3, 3>(State::kNBaseMultiWindowState,
                         State::kNBaseMultiWindowState) =
        state.P_.block<3, 3>(0, 0).eval();
    // Orientation.
    state.P_.block<3, 3>(State::kNBaseMultiWindowState + 3,
                         State::kNBaseMultiWindowState + 3) =
        state.P_.block<3, 3>(6, 6).eval();
    // Cross-correlations of position and orientation.
    state.P_.block<3, 3>(State::kNBaseMultiWindowState + 3,
                         State::kNBaseMultiWindowState) =
        state.P_.block<3, 3>(6, 0).eval();
    state.P_.block<3, 3>(State::kNBaseMultiWindowState,
                         State::kNBaseMultiWindowState + 3) =
        state.P_.block<3, 3>(0, 6).eval();
    // Cross-correlations of all states - rows - position.
    state.P_.block<3, State::kNBaseMultiWindowState>(
        State::kNBaseMultiWindowState, 0) =
        state.P_.block<3, State::kNBaseMultiWindowState>(0, 0).eval();
    // Orientation.
    state.P_.block<3, State::kNBaseMultiWindowState>(
        State::kNBaseMultiWindowState + 3, 0) =
        state.P_.block<3, State::kNBaseMultiWindowState>(6, 0).eval();
    // Cross-correlations of all states - cols - position.
    state.P_.block<State::kNBaseMultiWindowState, 3>(
        0, State::kNBaseMultiWindowState) =
        state.P_.block<State::kNBaseMultiWindowState, 3>(0, 0).eval();
    // Orientation.
    state.P_.block<State::kNBaseMultiWindowState, 3>(
        0, State::kNBaseMultiWindowState + 3) =
        state.P_.block<State::kNBaseMultiWindowState, 3>(0, 6).eval();
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
                           State::kNAugmentedState + H_pf.rows());
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
        State::kNAugmentedState +
        3 * closest_to_measurement_state.persistent_features_.size();
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
            correction.block<3, 1>(State::kNAugmentedState + i * 3, 0)
                .transpose();
    }
}

void StateUpdater::prepareMatricesForUpdate(
    const Parameters& parameters, const State& closest_to_measurement_state,
    Features& features, Eigen::MatrixXd& H, Eigen::MatrixXd& R,
    Eigen::MatrixXd& r)
{
    constexpr double kChiSquare1DoFThresholdHigh = 5.02;
    constexpr double kChiSquare1DoFThresholdLow = 0.000982069;
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
                        H_row_index, State::kNAugmentedState +
                                         3 * closest_to_measurement_state
                                                 .persistent_features_.size());

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
    constexpr double kChiSquare1DoFThresholdLow = 0.000982069;
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
                State::kNAugmentedState + 3 * closest_to_measurement_state
                                                  .persistent_features_.size());
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
    constexpr double kChiSquare1DoFThresholdLow = 0.000982069;
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
                State::kNAugmentedState + 3 * closest_to_measurement_state
                                                  .persistent_features_.size());
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
        3, State::kNAugmentedState +
               3 * closest_to_measurement_state.persistent_features_.size());
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
    H_not_reduced.block<3, 3>(0, State::kNAugmentedState + index * 3) =
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
        3, State::kNAugmentedState +
               3 * closest_to_measurement_state.persistent_features_.size());
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
        3, State::kNAugmentedState +
               3 * closest_to_measurement_state.persistent_features_.size());
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
