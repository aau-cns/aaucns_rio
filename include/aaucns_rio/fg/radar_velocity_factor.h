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

#ifndef _RADAR_VELOCITY_FACTOR_H_
#define _RADAR_VELOCITY_FACTOR_H_

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NoiseModelFactorN.h>

#include <Eigen/Dense>

#include "aaucns_rio/util.h"

namespace aaucns_rio
{
// Measurement model is a function of 3 entities: pose, v, and biases.
class RadialVelocityFactor
    : public gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Vector3,
                                      gtsam::imuBias::ConstantBias>
{
    double measured_radial_velocity_;
    Eigen::Vector3d w_m_;
    // Constant parts of the measurement model, computed once here instead
    // of in every evaluation (this factor is evaluated tens of millions of
    // times per sequence). Computed with the same expressions as before so
    // the results are unchanged.
    // Unit direction to the radar point (row vector).
    Eigen::Matrix<double, 1, 3> direction_;
    Eigen::Vector3d p_ri_;
    Eigen::Matrix3d R_ri_T_;
    // R_ri^T * [p_ri]x - Jacobian w.r.t. the gyro bias before reduction.
    Eigen::Matrix3d R_ri_T_skew_p_ri_;

   public:
    RadialVelocityFactor(const gtsam::Key j, const gtsam::Key i,
                         const gtsam::Key k,
                         const double measured_radial_velocity,
                         const Eigen::Vector3d& radar_point,
                         const gtsam::Pose3& radar_to_imu_transform,
                         const Eigen::Vector3d& w_m,
                         const gtsam::SharedNoiseModel& model)
        : gtsam::NoiseModelFactorN<gtsam::Pose3, gtsam::Vector3,
                                   gtsam::imuBias::ConstantBias>(model, j, i,
                                                                 k),
          measured_radial_velocity_(measured_radial_velocity),
          w_m_(w_m),
          direction_(radar_point.transpose().eval() / radar_point.norm()),
          p_ri_(radar_to_imu_transform.translation())
    {
        const gtsam::Quaternion q_ri_gtsam =
            radar_to_imu_transform.rotation().toQuaternion();
        Eigen::Quaternion<double> q_ri(q_ri_gtsam.w(), q_ri_gtsam.x(),
                                       q_ri_gtsam.y(), q_ri_gtsam.z());
        q_ri.normalize();
        R_ri_T_ = q_ri.conjugate().toRotationMatrix();
        R_ri_T_skew_p_ri_ = R_ri_T_ * util::getSkewSymmetricMat(p_ri_);
    }

    gtsam::Vector evaluateError(
        const gtsam::Pose3& p, const gtsam::Vector3& v,
        const gtsam::imuBias::ConstantBias& b,
        gtsam::OptionalMatrixType H1 = static_cast<gtsam::Matrix*>(nullptr),
        gtsam::OptionalMatrixType H2 = static_cast<gtsam::Matrix*>(nullptr),
        gtsam::OptionalMatrixType H3 =
            static_cast<gtsam::Matrix*>(nullptr)) const
    {
        const gtsam::Quaternion q_gtsam = p.rotation().toQuaternion();
        Eigen::Quaternion<double> q(q_gtsam.w(), q_gtsam.x(), q_gtsam.y(),
                                    q_gtsam.z());
        q.normalize();
        const Eigen::Matrix3d R_T = q.conjugate().toRotationMatrix();
        const Eigen::Vector3d b_w(b.gyroscope());

        // The Jacobians are only needed when linearizing, not when the
        // optimizer just evaluates the error.
        if (H1 || H2 || H3)
        {
            // Position and acceleration bias do not enter the model.
            const Eigen::Matrix<double, 1, 3> zero =
                Eigen::Matrix<double, 1, 3>::Zero();
            if (H1)
            {
                // Orientation.
                const Eigen::Vector3d to_skew = R_T * v;
                const Eigen::Matrix3d H_theta =
                    R_ri_T_ * util::getSkewSymmetricMat(to_skew);
                (*H1) = (gtsam::Matrix(1, 6) << direction_ * H_theta, zero)
                            .finished();
            }
            if (H2)
            {
                // Velocity.
                const Eigen::Matrix3d H_v = R_ri_T_ * R_T;
                (*H2) = direction_ * H_v;
            }
            if (H3)
            {
                // Gyro bias.
                (*H3) = (gtsam::Matrix(1, 6) << zero,
                         direction_ * R_ri_T_skew_p_ri_)
                            .finished();
            }
        }

        const Eigen::Vector3d radar_velocity_in_radar_frame =
            R_ri_T_ * R_T * v +
            R_ri_T_ * util::getSkewSymmetricMat(w_m_ - b_w) * p_ri_;
        const double evaluated_radial_velocity =
            direction_ * radar_velocity_in_radar_frame;
        const double error =
            evaluated_radial_velocity - measured_radial_velocity_;
        return (gtsam::Vector(1) << error).finished();
    }
};

}  // namespace aaucns_rio

#endif /* _RADAR_VELOCITY_FACTOR_H_ */
