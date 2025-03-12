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

#ifndef _POINT_ASSOCIATOR_H_
#define _POINT_ASSOCIATOR_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <fstream>

#include "aaucns_rio/circular_buffer.h"
#include "aaucns_rio/config.h"
#include "aaucns_rio/features.h"
#include "aaucns_rio/parameters.h"
#include "aaucns_rio/pcl_conversions_rio.h"
#include "aaucns_rio/state.h"
#include "aaucns_rio/trail.h"

namespace aaucns_rio
{
class PointsAssociator
{
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PointsAssociator(const bool dump_features);
    bool calculateFeaturesPositions(
        const pcl::PointCloud<RadarPointCloudType> &current_pointcloud,
        const State &closest_to_measurement_state,
        const CircularBuffer<pcl::PointCloud<RadarPointCloudType>,
                             Config::kMaxPastElements> &past_pointclouds,
        const Parameters &parameters, Features &features);

    bool calculateFeaturesFromTrail(
        const pcl::PointCloud<RadarPointCloudType> &current_pointcloud,
        State &current_state, const Parameters &parameters, Trail &trail,
        Features &features);

    ~PointsAssociator();

    static pcl::PointCloud<RadarPointCloudType> applyNearRangeFiltering(
        const pcl::PointCloud<RadarPointCloudType> &current_pc2,
        const double threshold_in_meters_x, const double threshold_in_meters_y);

   private:
    bool dump_features_;
    std::ofstream outfile_;

    void writeFeaturesPositionsToFile(
        const Features &features,
        const pcl::PointCloud<RadarPointCloudType> &current_pointcloud,
        const Trail &transformed_past_pointcloud, const int index);

    Eigen::MatrixXd calculateSimilarityMatrix(
        const Eigen::MatrixXd &distances_matrix,
        const Eigen::VectorXd &intensities_vector,
        const std::vector<std::pair<unsigned, unsigned>> &matching);

    bool searchForMatchingFeatures(
        const std::vector<TrailPoint> &new_trail_points,
        std::vector<TrailPoint> &trail_points,
        const std::vector<std::pair<unsigned, unsigned>> &matching,
        MatchingResult &matches, Eigen::MatrixXd &similarity_matrix,
        std::vector<int> &unmatched_new_trailpoints_indices);

    std::vector<std::pair<unsigned, unsigned>> calculateMatching(
        const Eigen::MatrixXd &distances_matrix);

    void calculateDistancesMatrices(
        const std::vector<TrailPoint> &new_trail_points,
        const std::vector<TrailPoint> &transformed_trail_points,
        Eigen::MatrixXd &distances_matrix);

    void calculateIntensitiesVector(
        const std::vector<TrailPoint> &new_trail_points,
        Eigen::VectorXd &intensities_vector);
};

}  // namespace aaucns_rio

#endif /* _POINT_ASSOCIATOR_H_ */
