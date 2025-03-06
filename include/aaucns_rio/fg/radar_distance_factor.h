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

#ifndef _RADAR_DISTANCE_FACTOR_H_
#define _RADAR_DISTANCE_FACTOR_H_

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
#include "aaucns_rio/trail.h"
#include "aaucns_rio/util.h"
#include "gtsam/linear/LossFunctions.h"

namespace aaucns_rio
{
class RadarDistanceFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>
{
    gtsam::Pose3 radar_to_imu_transform_;
    Eigen::Matrix<double, 6, 1> matched_pair_at_past_pose_;

   public:
    RadarDistanceFactor(
        const gtsam::Key i, const gtsam::Key j,
        const gtsam::SharedNoiseModel& model,
        const gtsam::Pose3& radar_to_imu_transform,
        // head is past, tail is current.
        const Eigen::Matrix<double, 1, 6>& matched_pair_at_past_pose)
        : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(model, i, j),
          radar_to_imu_transform_(radar_to_imu_transform),
          matched_pair_at_past_pose_(matched_pair_at_past_pose.transpose())
    {
    }

    double evaluateDistanceMeasurementModelAndGetJacobian(
        const gtsam::Pose3& past_p_gtsam, const gtsam::Pose3& current_p_gtsam,
        Eigen::MatrixXd& jacobian) const
    {
        gtsam::Quaternion q_ri_gtsam =
            radar_to_imu_transform_.rotation().toQuaternion();
        Eigen::Quaternion<double> q_ri(q_ri_gtsam.w(), q_ri_gtsam.x(),
                                       q_ri_gtsam.y(), q_ri_gtsam.z());
        Eigen::Vector3d p_ri(radar_to_imu_transform_.translation());
        q_ri.normalize();

        gtsam::Quaternion past_q_gtsam = past_p_gtsam.rotation().toQuaternion();
        Eigen::Quaternion<double> past_q(past_q_gtsam.w(), past_q_gtsam.x(),
                                         past_q_gtsam.y(), past_q_gtsam.z());
        Eigen::Vector3d past_p = past_p_gtsam.translation();
        past_q.normalize();

        gtsam::Quaternion current_q_gtsam =
            current_p_gtsam.rotation().toQuaternion();
        Eigen::Quaternion<double> current_q(
            current_q_gtsam.w(), current_q_gtsam.x(), current_q_gtsam.y(),
            current_q_gtsam.z());
        Eigen::Vector3d current_p = current_p_gtsam.translation();
        current_q.normalize();

        // Compute the jacobian. Here the past measurement is NOT transformed
        // into the current frame. 15 (nav) + 6 (past pose) = 21.
        Eigen::MatrixXd H_not_reduced(3, 21);
        H_not_reduced.setZero();
        // H(0, 0)
        H_not_reduced.block<3, 3>(0, 0) =
            -q_ri.conjugate().toRotationMatrix() *
            current_q.conjugate().toRotationMatrix();
        // H(0, 1) = 0
        // H(0, 2)
        const Eigen::Vector3d to_skew =
            -current_q.conjugate().toRotationMatrix() *
            (past_q.toRotationMatrix() *
                 (q_ri.toRotationMatrix() * matched_pair_at_past_pose_.head(3) +
                  p_ri) -
             current_p + past_p);
        H_not_reduced.block<3, 3>(0, 6) = -q_ri.conjugate().toRotationMatrix() *
                                          util::getSkewSymmetricMat(to_skew);
        // H(0, 3) = 0
        // H(0, 4) = 0
        // H(0, 7) - Past pose position.
        H_not_reduced.block<3, 3>(0, 15) =
            q_ri.conjugate().toRotationMatrix() *
            current_q.conjugate().toRotationMatrix();
        // H(0, 8) - past pose orientation.
        H_not_reduced.block<3, 3>(0, 18) =
            q_ri.conjugate().toRotationMatrix() *
            current_q.conjugate().toRotationMatrix() *
            past_q.toRotationMatrix() *
            util::getSkewSymmetricMat(-q_ri.toRotationMatrix() *
                                          matched_pair_at_past_pose_.head(3) -
                                      p_ri);

        // Reduce the jacobian.
        Eigen::Vector3d h_evaluated =
            q_ri.conjugate().toRotationMatrix() *
            (current_q.conjugate().toRotationMatrix() *
                 (past_q.toRotationMatrix() *
                      (q_ri.toRotationMatrix() *
                           matched_pair_at_past_pose_.head(3) +
                       p_ri) -
                  current_p + past_p) -
             p_ri);

        jacobian =
            (h_evaluated.transpose() / h_evaluated.norm()) * H_not_reduced;

        return h_evaluated.norm();
    }

    gtsam::Vector evaluateError(
        const gtsam::Pose3& p_past, const gtsam::Pose3& p_current,
        gtsam::OptionalMatrixType H1 = static_cast<gtsam::Matrix*>(nullptr),
        gtsam::OptionalMatrixType H2 =
            static_cast<gtsam::Matrix*>(nullptr)) const
    {
        Eigen::MatrixXd jacobian;
        const double evaluated_distance =
            evaluateDistanceMeasurementModelAndGetJacobian(p_past, p_current,
                                                           jacobian);

        if (H1)
            (*H1) = (gtsam::Matrix(1, 6) << jacobian.block(0, 18, 1, 3),
                     jacobian.block(0, 15, 1, 3))
                        .finished();
        if (H2)
            (*H2) = (gtsam::Matrix(1, 6) << jacobian.block(0, 6, 1, 3),
                     jacobian.block(0, 0, 1, 3))
                        .finished();

        const double measured_distance =
            matched_pair_at_past_pose_.tail(3).norm();
        const double error = evaluated_distance - measured_distance;
        return (gtsam::Vector(1) << error).finished();
    }
};

class RadarDistanceFactorFactory
{
   public:
    static std::vector<RadarDistanceFactor> createFactors(
        const CircularBuffer<RadarDistanceFactorData,
                             ConfigFg::kNPosesInWindow>&
            distance_factor_data_horizon,
        const Parameters& parameters,
        const gtsam::Pose3& radar_to_imu_transform, const Trail& trail,
        const int poses_in_window)
    {
        using gtsam::symbol_shorthand::X;

        const gtsam::noiseModel::Diagonal::shared_ptr distance_factor_noise =
            gtsam::noiseModel::Diagonal::Variances(
                (gtsam::Vector(1)
                 << parameters.noise_meas1_ * parameters.noise_meas1_)
                    .finished());
        // 0.0001
        gtsam::noiseModel::mEstimator::DCS::shared_ptr huber_loss(
            new gtsam::noiseModel::mEstimator::DCS(0.0001));
        gtsam::noiseModel::Robust::shared_ptr distance_robust_noise(
            new gtsam::noiseModel::Robust(huber_loss, distance_factor_noise));

        std::vector<RadarDistanceFactor> radar_distance_factors;
        Trail traill = trail;
        // std::cout << "---beg trail---" << std::endl;
        for (int i = 0; i < trail.size(); ++i)
        {
            // if (traill[i].matches_history.size() >=
            //    ConfigFg::kNSamplesForFeaturePersistence)
            //{
            // std::cout << "---beg---" << std::endl;
            // std::cout << traill[i].matches_history << std::endl;
            for (int j = 0; j < traill[i].matches_history.size(); ++j)
            {
                traill[i].matches_history[j].segment(3, 3) =
                    traill[i].matches_history[0].segment(3, 3);
                RadarDistanceFactor distance_factor = RadarDistanceFactor(
                    X(poses_in_window - j - 1), X(poses_in_window),
                    distance_robust_noise
                    /*distance_factor_noise*/,
                    radar_to_imu_transform, traill[i].matches_history[j]);
                radar_distance_factors.push_back(distance_factor);
                // std::cout << "Factor : "
                //          << "X(" << window_counter - j - 1 << ")-"
                //          << "X(" << window_counter << ")" << std::endl;
            }
            // std::cout << traill[i].matches_history << std::endl;
            // std::cout << "---end---" << std::endl;
            //}
        }
        // std::cout << "---end trail---" << std::endl;
        return radar_distance_factors;
    }
};

}  // namespace aaucns_rio

#endif /* _RADAR_DISTANCE_FACTOR_H_ */
