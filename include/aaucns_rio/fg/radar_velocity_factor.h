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

#include <Eigen/Dense>

#include "aaucns_rio/util.h"

namespace aaucns_rio
{
// Measurement model is a function of 3 entities: pose, v, and biases.
class RadialVelocityFactor
    : public gtsam::NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3,
                                      gtsam::imuBias::ConstantBias>
{
    double measured_radial_velocity_;
    Eigen::Vector3d radar_point_;
    gtsam::Pose3 radar_to_imu_transform_;
    Eigen::Vector3d w_m_;

   public:
    RadialVelocityFactor(const gtsam::Key j, const gtsam::Key i,
                         const gtsam::Key k,
                         const double measured_radial_velocity,
                         const Eigen::Vector3d& radar_point,
                         const gtsam::Pose3& radar_to_imu_transform,
                         const Eigen::Vector3d& w_m,
                         const gtsam::SharedNoiseModel& model)
        : NoiseModelFactor3<gtsam::Pose3, gtsam::Vector3,
                            gtsam::imuBias::ConstantBias>(model, j, i, k),
          measured_radial_velocity_(measured_radial_velocity),
          radar_point_(radar_point),
          radar_to_imu_transform_(radar_to_imu_transform),
          w_m_(w_m)
    {
    }

    double evaluateRadialVelocityMeasurementModelAndGetJacobian(
        const gtsam::Pose3& p_gtsam, const gtsam::Vector3& v_gtsam,
        const gtsam::imuBias::ConstantBias& b_gtsam,
        Eigen::MatrixXd& jacobian) const
    {
        gtsam::Quaternion q_ri_gtsam =
            radar_to_imu_transform_.rotation().toQuaternion();
        Eigen::Quaternion<double> q_ri(q_ri_gtsam.w(), q_ri_gtsam.x(),
                                       q_ri_gtsam.y(), q_ri_gtsam.z());
        q_ri.normalize();

        Eigen::Vector3d p_ri(radar_to_imu_transform_.translation());
        gtsam::Quaternion q_gtsam = p_gtsam.rotation().toQuaternion();
        Eigen::Quaternion<double> q(q_gtsam.w(), q_gtsam.x(), q_gtsam.y(),
                                    q_gtsam.z());
        q.normalize();

        Eigen::Vector3d v(v_gtsam);
        Eigen::Vector3d b_w(b_gtsam.gyroscope());

        // Compute the jacobian.
        Eigen::MatrixXd H_not_reduced(3, 15);
        H_not_reduced.setZero();
        // H(0, 0) = 0 (position).
        // H(0, 1) (velocity).
        H_not_reduced.block<3, 3>(0, 3) = q_ri.conjugate().toRotationMatrix() *
                                          q.conjugate().toRotationMatrix();
        // H(0, 2) (orientation).
        const Eigen::Vector3d to_skew = q.conjugate().toRotationMatrix() * v;
        H_not_reduced.block<3, 3>(0, 6) = q_ri.conjugate().toRotationMatrix() *
                                          util::getSkewSymmetricMat(to_skew);
        // H(0, 3) (omega bias).
        H_not_reduced.block<3, 3>(0, 9) = q_ri.conjugate().toRotationMatrix() *
                                          util::getSkewSymmetricMat(p_ri);
        // H(0, 4) = 0 (acceleration bias).

        jacobian = (radar_point_.tail(3).transpose().eval() /
                    radar_point_.tail(3).norm()) *
                   H_not_reduced;

        // Compute the evaluated velocity.
        const Eigen::Vector3d radar_velocity_in_radar_frame =
            q_ri.conjugate().toRotationMatrix() *
                q.conjugate().toRotationMatrix() * v +
            q_ri.conjugate().toRotationMatrix() *
                util::getSkewSymmetricMat(w_m_ - b_w) * p_ri;

        const double velocity_from_state_evaluated =
            (radar_point_.transpose().eval() / radar_point_.norm()) *
            radar_velocity_in_radar_frame;
        return velocity_from_state_evaluated;
    }

    gtsam::Vector evaluateError(
        const gtsam::Pose3& p, const gtsam::Vector3& v,
        const gtsam::imuBias::ConstantBias& b,
        gtsam::OptionalMatrixType H1 = static_cast<gtsam::Matrix*>(nullptr),
        gtsam::OptionalMatrixType H2 = static_cast<gtsam::Matrix*>(nullptr),
        gtsam::OptionalMatrixType H3 =
            static_cast<gtsam::Matrix*>(nullptr)) const
    {
        Eigen::MatrixXd jacobian;
        const double evaluated_radial_velocity =
            evaluateRadialVelocityMeasurementModelAndGetJacobian(p, v, b,
                                                                 jacobian);

        if (H1)
            (*H1) = (gtsam::Matrix(1, 6) << jacobian.block(0, 6, 1, 3),
                     jacobian.block(0, 0, 1, 3))
                        .finished();
        if (H2) (*H2) = jacobian.block(0, 3, 1, 3);
        if (H3)
            (*H3) = (gtsam::Matrix(1, 6) << jacobian.block(0, 12, 1, 3),
                     jacobian.block(0, 9, 1, 3))
                        .finished();

        const double error =
            evaluated_radial_velocity - measured_radial_velocity_;
        return (gtsam::Vector(1) << error).finished();
    }
};

}  // namespace aaucns_rio

#endif /* _RADAR_VELOCITY_FACTOR_H_ */
