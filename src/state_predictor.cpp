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

#include "aaucns_rio/state_predictor.h"

#include "aaucns_rio/calc_q.h"
#include "aaucns_rio/debug.h"
#include "aaucns_rio/util.h"

namespace aaucns_rio
{
StatePredictor::StatePredictor()
{
    // Gravity vector.
    g_ << 0, 0, 9.81;
    // Resize system matrices to base size (without augmentation).
    Fd_.resize(State::kNAugmentedState, State::kNAugmentedState);
    Qd_.resize(State::kNAugmentedState, State::kNAugmentedState);
    Fd_.setIdentity();
    Qd_.setZero();
}

const Eigen::Vector3d& StatePredictor::getGravityVector() const { return g_; }

// Eigen stores quaternion as xyzw hence we need the JPL-based matrix formula
// for omega.
Eigen::Matrix<double, 4, 4> StatePredictor::getOmegaJPL(
    const Eigen::Vector3d& angular_vel) const
{
    return (Eigen::Matrix<double, 4, 4>() << 0.0, angular_vel(2),
            -angular_vel(1), angular_vel(0), -angular_vel(2), 0.0,
            angular_vel(0), angular_vel(1), angular_vel(1), -angular_vel(0), 0,
            angular_vel(2), -angular_vel(0), -angular_vel(1), -angular_vel(2),
            0)
        .finished();
}

void StatePredictor::predictState(const double dt, const State& previous_state,
                                  State& current_state)
{
    // Write states which need no computations.
    current_state.b_w_ = previous_state.b_w_;
    current_state.b_a_ = previous_state.b_a_;
    current_state.p_ri_ = previous_state.p_ri_;
    current_state.q_ri_ = previous_state.q_ri_;

    current_state.past_positions_ = previous_state.past_positions_;
    current_state.past_orientations_ = previous_state.past_orientations_;
    current_state.persistent_features_ = previous_state.persistent_features_;

    // Get the matrix for quaternion integration using ->
    // https://www.iri.upc.edu/people/jsola/JoanSola/objectes/notes/kinematics.pdf
    // page 49, formula 225.
    const Eigen::Matrix<double, 3, 1> ew =
        current_state.w_m_ - current_state.b_w_;
    const Eigen::Matrix<double, 3, 1> previous_ew =
        previous_state.w_m_ - previous_state.b_w_;
    const Eigen::Matrix<double, 3, 1> ea =
        current_state.a_m_ - current_state.b_a_;
    const Eigen::Matrix<double, 3, 1> previous_ea =
        previous_state.a_m_ - previous_state.b_a_;
    const Eigen::Matrix<double, 4, 4> omega = getOmegaJPL(ew);
    const Eigen::Matrix<double, 4, 4> previous_omega = getOmegaJPL(previous_ew);
    Eigen::Matrix<double, 4, 4> omega_mean =
        getOmegaJPL((ew + previous_ew) / 2);

    int denominator = 1;
    Eigen::Matrix<double, 4, 4> mat_exponential;
    mat_exponential.setIdentity();
    omega_mean *= 0.5 * dt;
    for (int i = 1; i < 5; ++i)
    {
        denominator *= i;
        mat_exponential = mat_exponential + omega_mean / denominator;
        omega_mean *= omega_mean;
    }

    // First oder quat integration matrix.
    const Eigen::Matrix<double, 4, 4> quat_integration_mat =
        mat_exponential +
        1.0 / 48.0 * (omega * previous_omega - previous_omega * omega) * dt *
            dt;

    // Integrate IMU quaternion and predict IMU position and velocity.
    current_state.q_.coeffs() =
        quat_integration_mat * previous_state.q_.coeffs();
    current_state.q_.normalize();
    const Eigen::Vector3d dv =
        (current_state.q_.toRotationMatrix() * ea +
         previous_state.q_.toRotationMatrix() * previous_ea) /
        2;
    current_state.v_ = previous_state.v_ + (dv - g_) * dt;
    current_state.p_ =
        previous_state.p_ + ((current_state.v_ + previous_state.v_) / 2 * dt);
}

void StatePredictor::predictProcessCovariance(const double dt,
                                              const State& previous_state,
                                              const Parameters& parameters,
                                              State& current_state)
{
    typedef const Eigen::Matrix<double, 3, 3> ConstMatrix3;
    typedef const Eigen::Matrix<double, 3, 1> ConstVector3;
    typedef Eigen::Vector3d Vector3;

    // Noises.
    ConstVector3 nav =
        Vector3::Constant(parameters.noise_acc_ /* / sqrt(dt) */);
    ConstVector3 nbav =
        Vector3::Constant(parameters.noise_accbias_ /* * sqrt(dt) */);
    ConstVector3 nwv =
        Vector3::Constant(parameters.noise_gyr_ /* / sqrt(dt) */);
    ConstVector3 nbwv =
        Vector3::Constant(parameters.noise_gyrbias_ /* * sqrt(dt) */);

    // Stuff below should be zero.
    ConstVector3 nqwvv = Eigen::Vector3d::Constant(parameters.noise_qwv_);
    ConstVector3 nqciv = Eigen::Vector3d::Constant(parameters.noise_qri_);
    ConstVector3 npicv = Eigen::Vector3d::Constant(parameters.noise_pri_);
    ConstVector3 nauxv = Eigen::Vector3d::Constant(parameters.noise_aux_);

    // Bias-corrected IMU readings.
    ConstVector3 ew = current_state.w_m_ - current_state.b_w_;
    ConstVector3 ea = current_state.a_m_ - current_state.b_a_;

    ConstMatrix3 a_sk = util::getSkewSymmetricMat(ea);
    ConstMatrix3 w_sk = util::getSkewSymmetricMat(ew);
    ConstMatrix3 eye3 = Eigen::Matrix<double, 3, 3>::Identity();

    ConstMatrix3 C_eq = current_state.q_.toRotationMatrix();

    const double dt_p2_2 = dt * dt * 0.5;          // dt^2 / 2
    const double dt_p3_6 = dt_p2_2 * dt / 3.0;     // dt^3 / 6
    const double dt_p4_24 = dt_p3_6 * dt * 0.25;   // dt^4 / 24
    const double dt_p5_120 = dt_p4_24 * dt * 0.2;  // dt^5 / 120

    ConstMatrix3 Ca3 = C_eq * a_sk;
    ConstMatrix3 A =
        Ca3 * (-dt_p2_2 * eye3 + dt_p3_6 * w_sk - dt_p4_24 * w_sk * w_sk);
    ConstMatrix3 B =
        Ca3 * (dt_p3_6 * eye3 - dt_p4_24 * w_sk + dt_p5_120 * w_sk * w_sk);
    ConstMatrix3 D = -A;
    ConstMatrix3 E = eye3 - dt * w_sk + dt_p2_2 * w_sk * w_sk;
    ConstMatrix3 F = -dt * eye3 + dt_p2_2 * w_sk - dt_p3_6 * (w_sk * w_sk);
    ConstMatrix3 C = Ca3 * F;

    // Discrete error state propagation Matrix Fd according to:
    // Stephan Weiss and Roland Siegwart.
    // Real-Time Metric State Estimation for Modular Vision-Inertial Systems.
    // IEEE International Conference on Robotics and Automation. Shanghai,
    // China, 2011.
    Eigen::MatrixXd Fd(State::kNAugmentedState, State::kNAugmentedState);
    Fd.setIdentity();
    Fd.block<3, 3>(0, 3) = dt * eye3;
    Fd.block<3, 3>(0, 6) = A;
    Fd.block<3, 3>(0, 9) = B;
    Fd.block<3, 3>(0, 12) = -C_eq * dt_p2_2;
    Fd.block<3, 3>(3, 6) = C;
    Fd.block<3, 3>(3, 9) = D;
    Fd.block<3, 3>(3, 12) = -C_eq * dt;
    Fd.block<3, 3>(6, 6) = E;
    Fd.block<3, 3>(6, 9) = F;

    const std::size_t n_state_variables =
        State::kNAugmentedState + 3 * current_state.persistent_features_.size();

    Fd_.resize(n_state_variables, n_state_variables);
    Fd_.setIdentity();
    Fd_.block(0, 0, State::kNImuState, State::kNImuState) =
        Fd.block(0, 0, State::kNImuState, State::kNImuState);

    Eigen::MatrixXd Qd;
    Qd.resize(State::kNState, State::kNState);
    Qd.setZero();
    calc_Q(dt, current_state.q_, ew, ea, nav, nbav, nwv, nbwv,
           parameters.noise_scale_, nqwvv, nqciv, npicv, nauxv, Qd);
    // Copy over block corresponding to the IMU states.
    Qd_.resize(n_state_variables, n_state_variables);
    Qd_.setZero();
    Qd_.block(0, 0, State::kNImuState, State::kNImuState) =
        Qd.block(0, 0, State::kNImuState, State::kNImuState);

    Eigen::Matrix<double, 6, 6> calib_noise;
    calib_noise.setZero();
    Eigen::Matrix<double, 6, 1> diagonal;
    diagonal << 0.00000001, 0.00000001, 0.00000001, 0.00000001, 0.00000001,
        0.00000001;
    calib_noise.diagonal() = diagonal;
    Qd_.block(State::kNImuState, State::kNImuState, 6, 6) = calib_noise;

    current_state.P_ = Fd_ * previous_state.P_ * Fd_.transpose() + Qd_;
}

}  // namespace aaucns_rio
