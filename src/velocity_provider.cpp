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

#include "aaucns_rio/velocity_provider.h"

#include <random>

#include "aaucns_rio/debug.h"
#include "aaucns_rio/util.h"

namespace aaucns_rio
{
Eigen::MatrixXd VelocityProvider::getPointsAndVelocities(
    const pcl::PointCloud<RadarPointCloudType>& current_pc2,
    debug::Logger& logger)
{
    const pcl::PointCloud<RadarPointCloudType> filtered_current_pc2 =
        util::applyPyramidFiltering(
            util::applyNearRangeFiltering(current_pc2, 0.1, 0.1));

    Eigen::MatrixXd velocities_and_points;
    int n_points = 0;
    for (int i = 0; i < filtered_current_pc2.width; ++i)
    {
        if (filtered_current_pc2.points[i].intensity > 0.0)
        {
            ++n_points;
            velocities_and_points.conservativeResize(
                n_points, kNPointAndVelocityDimension);
            velocities_and_points.row(n_points - 1)
                << -filtered_current_pc2.points[i].doppler,
                filtered_current_pc2.points[i].x,
                filtered_current_pc2.points[i].y,
                filtered_current_pc2.points[i].z;
        }
    }
    // runRansac(velocities_and_points);
    return velocities_and_points;
}

// This function does not perform the full RANSAC. It returns the best inlier
// set. It is so since we use tightly coupled approach and we don't need
// to estimate the radar velocity.
void VelocityProvider::runRansac(Eigen::MatrixXd& velocities_and_points)
{
    std::random_device random_device;
    auto random_engine = std::default_random_engine{random_device()};
    // Arry of indexes to randomly shuffle.
    std::vector<int> vel_indexes(velocities_and_points.rows());
    std::generate(vel_indexes.begin(), vel_indexes.end(),
                  [n = 0]() mutable { return n++; });

    std::vector<int> final_inlier_indexes;
    if (velocities_and_points.rows() >= kNRansacPoints)
    {
        // Ransac.
        for (int i = 0; i < kNRansacIter; ++i)
        {
            std::shuffle(vel_indexes.begin(), vel_indexes.end(), random_engine);
            Eigen::MatrixXd potential_inliers(kNRansacPoints,
                                              kNPointAndVelocityDimension);
            for (int j = 0; j < potential_inliers.rows(); ++j)
            {
                potential_inliers.row(j) =
                    velocities_and_points.row(vel_indexes[j]);
            }
            // Compute hypothesis model using LLS.
            Eigen::Vector3d estimated_radar_vel;
            if (solveLLS(potential_inliers, estimated_radar_vel))
            {
                const Eigen::VectorXd residuals =
                    (velocities_and_points.col(0) -
                     velocities_and_points.rightCols(
                         kNPointAndVelocityDimension - 1) *
                         estimated_radar_vel)
                        .array()
                        .abs();
                std::vector<int> inlier_indexes;
                for (int k = 0; k < residuals.rows(); ++k)
                {
                    if (residuals(k) < kInlierThreshold)
                    {
                        inlier_indexes.push_back(k);
                    }
                }
                if (inlier_indexes.size() > final_inlier_indexes.size())
                {
                    final_inlier_indexes = inlier_indexes;
                }
            }
        }
        if (!final_inlier_indexes.empty())
        {
            // Pick out inliers from the original input matrix.
            Eigen::MatrixXd resized_velocities_and_points(
                final_inlier_indexes.size(), kNPointAndVelocityDimension);
            for (int i = 0; i < resized_velocities_and_points.rows(); ++i)
            {
                resized_velocities_and_points.row(i) =
                    velocities_and_points.row(final_inlier_indexes[i]);
            }
            velocities_and_points = resized_velocities_and_points;
        }
    }
}

bool VelocityProvider::solveLLS(Eigen::MatrixXd& potential_inliers,
                                Eigen::Vector3d& estimated_radar_vel)
{
    Eigen::Matrix<double, Eigen::Dynamic, kNPointAndVelocityDimension - 1> A =
        potential_inliers.rightCols(kNPointAndVelocityDimension - 1);
    Eigen::Matrix<double, kNPointAndVelocityDimension - 1,
                  kNPointAndVelocityDimension - 1>
        ATA = A.transpose() * A;
    Eigen::JacobiSVD<Eigen::Matrix<double, kNPointAndVelocityDimension - 1,
                                   kNPointAndVelocityDimension - 1>>
        svd(ATA);
    double cond = svd.singularValues()(0) /
                  svd.singularValues()(svd.singularValues().size() - 1);
    if (std::fabs(cond) < 1.0e3)
    {
        Eigen::Matrix<double, Eigen::Dynamic, 1> y = potential_inliers.col(0);
        estimated_radar_vel =
            (ATA).colPivHouseholderQr().solve(A.transpose() * y);
        return true;
    }
    else
    {
        return false;
    }
}

}  // namespace aaucns_rio
