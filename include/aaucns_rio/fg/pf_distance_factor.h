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

#ifndef _PF_DISTANCE_FACTOR_H_
#define _PF_DISTANCE_FACTOR_H_

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/slam/BetweenFactor.h>

#include <Eigen/Dense>
#include <vector>

#include "aaucns_rio/features.h"
#include "aaucns_rio/state.h"
#include "aaucns_rio/trail.h"
#include "aaucns_rio/util.h"

namespace aaucns_rio
{
// Factor between a point (position of a landmark) and a pose.
class RadarPFDistanceFactor
    : public gtsam::NoiseModelFactor2<gtsam::Point3, gtsam::Pose3>
{
    gtsam::Pose3 radar_to_imu_transform_;
    // State with persistent features. in EKF `.most_recent_coordinates` holds
    // the estimated value and `.matches_history[0].head(3)` holds latest
    // measurement of a PF.
    Eigen::Vector3d measured_pf_coords_;

   public:
    RadarPFDistanceFactor(const gtsam::Key i, const gtsam::Key j,
                          const gtsam::SharedNoiseModel& model,
                          const gtsam::Pose3& radar_to_imu_transform,
                          const Eigen::Vector3d& measured_pf_coords)
        // head is past, tail is current.
        : gtsam::NoiseModelFactor2<gtsam::Point3, gtsam::Pose3>(model, i, j),
          radar_to_imu_transform_(radar_to_imu_transform),
          measured_pf_coords_(measured_pf_coords)
    {
    }

    double evaluatePFDistanceMeasurementModelAndGetJacobian(
        const gtsam::Point3& pf_gtsam, const gtsam::Pose3& current_p_gtsam,
        Eigen::MatrixXd& jacobian) const
    {
        gtsam::Quaternion q_ri_gtsam =
            radar_to_imu_transform_.rotation().toQuaternion();
        Eigen::Quaternion<double> q_ri(q_ri_gtsam.w(), q_ri_gtsam.x(),
                                       q_ri_gtsam.y(), q_ri_gtsam.z());
        Eigen::Vector3d p_ri(radar_to_imu_transform_.translation());
        q_ri.normalize();

        Eigen::Vector3d pf(pf_gtsam);

        gtsam::Quaternion current_q_gtsam =
            current_p_gtsam.rotation().toQuaternion();
        Eigen::Quaternion<double> current_q(
            current_q_gtsam.w(), current_q_gtsam.x(), current_q_gtsam.y(),
            current_q_gtsam.z());
        Eigen::Vector3d current_p = current_p_gtsam.translation();
        current_q.normalize();

        // Compute the jacobian. 15 (IMU) + 3 (PF position) = 18.
        Eigen::MatrixXd H_not_reduced(3, 18);
        H_not_reduced.setZero();
        // H(0, 0).
        H_not_reduced.block<3, 3>(0, 0) =
            -q_ri.conjugate().toRotationMatrix() *
            current_q.conjugate().toRotationMatrix();
        // H(0, 1) = 0.
        // H(0, 2).
        const Eigen::Vector3d to_skew =
            current_q.conjugate().toRotationMatrix() * (pf - current_p);
        H_not_reduced.block<3, 3>(0, 6) = q_ri.conjugate().toRotationMatrix() *
                                          util::getSkewSymmetricMat(to_skew);
        // H(0, 3) = 0.
        // H(0, 4) = 0.
        // H(0, 5) = 0 (calibration parameters - translation).
        // H(0, 6) = 0 (calibration parameters - rotation).
        // H(0, N * clones) = 0.
        // H(0, 15) PF.
        H_not_reduced.block<3, 3>(0, 15) =
            q_ri.conjugate().toRotationMatrix() *
            current_q.conjugate().toRotationMatrix();

        const Eigen::Vector3d h_evaluated =
            q_ri.conjugate().toRotationMatrix() *
            (current_q.conjugate().toRotationMatrix() * (pf - current_p) -
             p_ri);
        jacobian =
            (h_evaluated.transpose() / h_evaluated.norm()) * H_not_reduced;

        return h_evaluated.norm();
    }

    gtsam::Vector evaluateError(
        const gtsam::Point3& pf, const gtsam::Pose3& p_current,
        gtsam::OptionalMatrixType H1 = static_cast<gtsam::Matrix*>(nullptr),
        gtsam::OptionalMatrixType H2 =
            static_cast<gtsam::Matrix*>(nullptr)) const
    {
        Eigen::MatrixXd jacobian;
        const double evaluated_pf_distance =
            evaluatePFDistanceMeasurementModelAndGetJacobian(pf, p_current,
                                                             jacobian);

        if (H1)
            (*H1) =
                (gtsam::Matrix(1, 3) << jacobian.block(0, 15, 1, 3)).finished();
        if (H2)
            (*H2) = (gtsam::Matrix(1, 6) << jacobian.block(0, 6, 1, 3),
                     jacobian.block(0, 0, 1, 3))
                        .finished();

        const double measured_pf_distance = measured_pf_coords_.norm();
        const double error = evaluated_pf_distance - measured_pf_distance;
        return (gtsam::Vector(1) << error).finished();
    }
};
class RadarPFDistanceFactorFactory
{
   public:
    static std::vector<RadarPFDistanceFactor> createFactors(
        const CircularBuffer<RadarPFDistanceFactorData,
                             ConfigFg::kNPosesInWindow>& pf_factor_data_horizon,
        const Parameters& parameters,
        const gtsam::Pose3& radar_to_imu_transform)
    {
        using gtsam::symbol_shorthand::P;
        using gtsam::symbol_shorthand::X;

        gtsam::noiseModel::Diagonal::shared_ptr pf_distance_factor_noise =
            gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(1)
                 << parameters.noise_meas4_ * parameters.noise_meas4_)
                    .finished());

        gtsam::noiseModel::mEstimator::DCS::shared_ptr huber_loss(
            new gtsam::noiseModel::mEstimator::DCS(1.0));
        gtsam::noiseModel::Robust::shared_ptr pf_distance_robust_noise(
            new gtsam::noiseModel::Robust(huber_loss,
                                          pf_distance_factor_noise));

        std::vector<RadarPFDistanceFactor> radar_pf_distance_factors;

        for (int j = 0; j < pf_factor_data_horizon.size(); ++j)
        {
            for (int i = 0;
                 i <
                 pf_factor_data_horizon[j]
                     .most_recent_updated_state_.persistent_features_.size();
                 ++i)
            {
                const RadarPFDistanceFactor radar_pf_distance_factor =
                    RadarPFDistanceFactor(
                        P(pf_factor_data_horizon[j]
                              .most_recent_updated_state_
                              .persistent_features_[i]
                              .id),
                        X(j),
                        pf_distance_robust_noise
                        /*pf_distance_factor_noise*/,
                        radar_to_imu_transform,
                        pf_factor_data_horizon[j]
                            .most_recent_updated_state_.persistent_features_[i]
                            .matches_history[0]
                            .head(3));
                radar_pf_distance_factors.push_back(radar_pf_distance_factor);
            }
        }
        return radar_pf_distance_factors;
    }
};

}  // namespace aaucns_rio

#endif /* _PF_DISTANCE_FACTOR_H_ */
