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

// Checks RadialVelocityExtrinsicFactor (online radar-IMU calibration): its
// error equals that of RadialVelocityFactor with the same (fixed) extrinsics,
// and its analytic Jacobians match numerical ones for all four keys.

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include <algorithm>
#include <cstdio>
#include <functional>
#include <random>

#include "aaucns_rio/fg/radar_velocity_factor.h"

int main()
{
    std::mt19937 rng(7);
    std::normal_distribution<double> n(0.0, 1.0);
    const auto vec3 = [&](const double s) {
        return Eigen::Vector3d(n(rng), n(rng), n(rng)) * s;
    };
    const auto pose = [&](const double r, const double t) {
        return gtsam::Pose3(gtsam::Rot3::Expmap(vec3(r)), gtsam::Point3(vec3(t)));
    };
    const auto noise = gtsam::noiseModel::Isotropic::Sigma(1, 0.05);
    const auto max_diff = [](const gtsam::Matrix& a, const gtsam::Matrix& b) {
        return (a - b).cwiseAbs().maxCoeff();
    };

    double max_error_difference = 0.0;
    double max_jacobian_error = 0.0;
    for (int trial = 0; trial < 200; ++trial)
    {
        const gtsam::Pose3 X = pose(1.0, 5.0);
        const gtsam::Vector3 v = vec3(2.0);
        const gtsam::imuBias::ConstantBias b(vec3(0.1), vec3(0.01));
        // Radar pose in the IMU frame: a 180 deg rotation as on the NTNU
        // platform, perturbed, and a lever arm of a few cm.
        const gtsam::Pose3 E =
            gtsam::Pose3(gtsam::Rot3::Ypr(0.0, M_PI, 0.47),
                         gtsam::Point3(0.079, 0.022, -0.059))
                .compose(pose(0.05, 0.01));
        const Eigen::Vector3d w_m = vec3(0.5);
        const Eigen::Vector3d point = vec3(5.0) + Eigen::Vector3d(6.0, 0, 0);
        const double measured = n(rng);

        const aaucns_rio::RadialVelocityFactor fixed(
            gtsam::Symbol('x', 1), gtsam::Symbol('v', 1), gtsam::Symbol('b', 1),
            measured, point, E, w_m, noise);
        const aaucns_rio::RadialVelocityExtrinsicFactor factor(
            gtsam::Symbol('x', 1), gtsam::Symbol('v', 1), gtsam::Symbol('b', 1),
            gtsam::Symbol('e', 0), measured, point, w_m, noise);

        gtsam::Matrix H1, H2, H3, H4;
        const double e = factor.evaluateError(X, v, b, E, &H1, &H2, &H3, &H4)(0);
        max_error_difference = std::max(
            max_error_difference, std::abs(e - fixed.evaluateError(X, v, b)(0)));

        const std::function<gtsam::Vector1(
            const gtsam::Pose3&, const gtsam::Vector3&,
            const gtsam::imuBias::ConstantBias&, const gtsam::Pose3&)>
            h = [&](const gtsam::Pose3& x, const gtsam::Vector3& vv,
                    const gtsam::imuBias::ConstantBias& bb,
                    const gtsam::Pose3& ee) {
                return gtsam::Vector1(factor.evaluateError(x, vv, bb, ee));
            };
        max_jacobian_error = std::max(
            {max_jacobian_error,
             max_diff(H1, gtsam::numericalDerivative41<
                              gtsam::Vector1, gtsam::Pose3, gtsam::Vector3,
                              gtsam::imuBias::ConstantBias, gtsam::Pose3>(
                              h, X, v, b, E, 1e-6)),
             max_diff(H2, gtsam::numericalDerivative42<
                              gtsam::Vector1, gtsam::Pose3, gtsam::Vector3,
                              gtsam::imuBias::ConstantBias, gtsam::Pose3>(
                              h, X, v, b, E, 1e-6)),
             max_diff(H3, gtsam::numericalDerivative43<
                              gtsam::Vector1, gtsam::Pose3, gtsam::Vector3,
                              gtsam::imuBias::ConstantBias, gtsam::Pose3>(
                              h, X, v, b, E, 1e-6)),
             max_diff(H4, gtsam::numericalDerivative44<
                              gtsam::Vector1, gtsam::Pose3, gtsam::Vector3,
                              gtsam::imuBias::ConstantBias, gtsam::Pose3>(
                              h, X, v, b, E, 1e-6))});
    }
    std::printf(
        "RadialVelocityExtrinsicFactor: 200 cases, max error difference to "
        "RadialVelocityFactor %.3g, max |analytic - numerical| Jacobian %.3g\n",
        max_error_difference, max_jacobian_error);
    const bool ok = max_error_difference < 1e-9 && max_jacobian_error < 1e-5;
    std::printf("%s\n", ok ? "PASSED" : "FAILED");
    return ok ? 0 : 1;
}
