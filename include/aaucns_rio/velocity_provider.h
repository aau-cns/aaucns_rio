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

#ifndef _VELOCITY_PROVIDER_H_
#define _VELOCITY_PROVIDER_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>

#include "aaucns_rio/debug.h"
#include "aaucns_rio/pcl_conversions_rio.h"

namespace aaucns_rio
{
class VelocityProvider
{
   public:
    static constexpr int kNPointAndVelocityDimension = 4;
    static constexpr int kNRansacIter = 17;
    static constexpr int kNRansacPoints = 3;
    static constexpr float kInlierThreshold = 0.15;

    VelocityProvider() = default;
    static Eigen::MatrixXd getPointsAndVelocities(
        const pcl::PointCloud<RadarPointCloudType>& current_pc2,
        debug::Logger& logger);

   private:
    static void runRansac(Eigen::MatrixXd& velocities_and_points);
    static bool solveLLS(Eigen::MatrixXd& potential_inliers,
                         Eigen::Vector3d& estimated_radar_vel);
};

}  // namespace aaucns_rio

#endif /* _VELOCITY_PROVIDER_H_ */
