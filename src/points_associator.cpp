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

#include <pcl_conversions/pcl_conversions.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <vector>

#include "aaucns_rio/DoubleArrayStamped.h"
#include "aaucns_rio/debug.h"
#include "aaucns_rio/rio.h"
#include "aaucns_rio/util.h"
#include "munkres-algorithm/munkres.hpp"

namespace aaucns_rio
{
PointsAssociator::PointsAssociator(const bool dump_features)
    : dump_features_(dump_features)
{
    if (dump_features_)
    {
        outfile_.open("features_positions.txt");
        if (outfile_.is_open())
        {
            outfile_ << "{"
                     << "\n";
            outfile_ << "\"frames\": ["
                     << "\n";
        }
    }
}

PointsAssociator::~PointsAssociator()
{
    if (dump_features_)
    {
        if (outfile_.is_open())
        {
            // Remove the "," and "\n" at the end of the file to conform to
            // JSON.
            const auto pos = outfile_.tellp();
            constexpr std::size_t kNCharsToDeleteFromEnd = 2;
            outfile_.seekp(pos - kNCharsToDeleteFromEnd);
            outfile_ << "\n";
            outfile_ << "]\n}"
                     << "\n";
            outfile_.close();
        }
    }
}

void PointsAssociator::writeFeaturesPositionsToFile(
    const Features& features,
    const pcl::PointCloud<pcl::PointXYZI>& current_pointcloud,
    const Trail& trail, const int index)
{
    if (dump_features_)
    {
        const static Eigen::IOFormat MatrixFormat(Eigen::FullPrecision, 0, ", ",
                                                  ",\n", "[", "]", "[", "]");
        const static Eigen::IOFormat ArrayFormat(Eigen::FullPrecision, 0, ", ",
                                                 "\n", "[", "]", "[", "]");
        Eigen::MatrixXd current_pointcloud_as_matrix =
            util::convertPC2ToEigenMatrix(current_pointcloud);
        Eigen::MatrixXd trail_as_matrix = trail.convertTrailToEigenMatrix();
        if (outfile_.is_open())
        {
            outfile_ << "{"
                     << "\n";
            outfile_ << "\"feature_positions\": \n";
            if (features.positions_and_matched_or_not_[index]
                    .positions.rows() == 1)
            {
                outfile_ << features.positions_and_matched_or_not_[index]
                                .positions.format(ArrayFormat)
                         << ",\n";
            }
            else
            {
                outfile_ << features.positions_and_matched_or_not_[index]
                                .positions.format(MatrixFormat)
                         << ",\n";
            }
            outfile_ << "\"current_pc\": \n";
            if (current_pointcloud_as_matrix.rows() == 1)
            {
                outfile_ << current_pointcloud_as_matrix.format(ArrayFormat)
                         << ",\n";
            }
            else
            {
                outfile_ << current_pointcloud_as_matrix.format(MatrixFormat)
                         << ",\n";
            }
            outfile_ << "\"previous_pc\": \n";
            if (trail_as_matrix.rows() == 1)
            {
                outfile_ << trail_as_matrix.format(ArrayFormat) << "\n";
            }
            else
            {
                outfile_ << trail_as_matrix.format(MatrixFormat) << "\n";
            }
            outfile_ << "},"
                     << "\n";
        }
    }
}

std::vector<std::pair<unsigned, unsigned>> PointsAssociator::calculateMatching(
    const Eigen::MatrixXd& distances_matrix)
{
    const std::vector<double> distances_matrix_as_std_vector =
        util::convertEigenMatrixTo1DStdVector(distances_matrix);
    auto f = [&](unsigned i, unsigned j) {
        return distances_matrix_as_std_vector[i * distances_matrix.cols() + j];
    };
    const std::vector<std::pair<unsigned, unsigned>> matching =
        munkres_algorithm<double>(distances_matrix.rows(),
                                  distances_matrix.cols(), f);
    return matching;
}

void PointsAssociator::calculateIntensitiesVector(
    const std::vector<TrailPoint>& new_trail_points,
    Eigen::VectorXd& intensities_vector)
{
    for (int i = 0; i < intensities_vector.rows(); ++i)
    {
        intensities_vector(i) = new_trail_points[i].intensity;
    }
}

void PointsAssociator::calculateDistancesMatrices(
    const std::vector<TrailPoint>& new_trail_points,
    const std::vector<TrailPoint>& transformed_trail_points,
    Eigen::MatrixXd& distances_matrix)
{
    Eigen::Vector3d a, b;
    for (int i = 0; i < distances_matrix.rows(); ++i)
    {
        a = transformed_trail_points[i].most_recent_coordinates;
        for (int j = 0; j < distances_matrix.cols(); ++j)
        {
            b = new_trail_points[j].most_recent_coordinates;
            distances_matrix(i, j) = (a - b).norm();
        }
    }
}

Eigen::MatrixXd PointsAssociator::calculateSimilarityMatrix(
    const Eigen::MatrixXd& distances_matrix,
    const Eigen::VectorXd& intensities_vector,
    const std::vector<std::pair<unsigned, unsigned>>& matching)
{
    constexpr double kEuclideanDistThreshold = 0.3;
    constexpr double kIntensityThreshold = 0.0;
    Eigen::MatrixXd similarity_matrix =
        Eigen::MatrixXd::Zero(matching.size(), matching.size());
    for (int i = 0; i < similarity_matrix.rows(); ++i)
    {
        for (int j = 0; j < similarity_matrix.cols(); ++j)
        {
            if ((distances_matrix(matching[i].first, matching[j].second) <
                 kEuclideanDistThreshold) &&
                (intensities_vector(matching[j].second) > kIntensityThreshold))
            {
                similarity_matrix(i, j) =
                    1.0 / (1.0 + distances_matrix(matching[i].first,
                                                  matching[j].second));
            }
        }
    }
    return similarity_matrix;
}

bool PointsAssociator::searchForMatchingFeatures(
    const std::vector<TrailPoint>& new_trail_points,
    std::vector<TrailPoint>& trail_points,
    const std::vector<std::pair<unsigned, unsigned>>& matching,
    MatchingResult& matches, Eigen::MatrixXd& similarity_matrix,
    std::vector<int>& unmatched_new_trailpoints_indices)
{
    std::vector<int> matched_new_trailpoints_indices;
    constexpr double kAcceptanceThreshold = 0.7;
    int n_matched = 0;
    int n_unmatched = 0;
    while (similarity_matrix.norm() > 0.0)
    {
        Eigen::MatrixXd::Index max_row, max_col;
        (void)similarity_matrix.maxCoeff(&max_row, &max_col);
        if (similarity_matrix(max_row, max_col) > kAcceptanceThreshold)
        {
            ++n_matched;
            if (n_matched == 1)
            {
                // Got persistent feature.
                if (trail_points[matching[max_row].first].is_persistent)
                {
                    // Save the matched measurement in
                    // `matches_history.head(3)`.
                    trail_points[matching[max_row].first]
                        .matches_history[0]
                        .head(3)(0) = new_trail_points[matching[max_col].second]
                                          .most_recent_coordinates(0);
                    trail_points[matching[max_row].first]
                        .matches_history[0]
                        .head(3)(1) = new_trail_points[matching[max_col].second]
                                          .most_recent_coordinates(1);
                    trail_points[matching[max_row].first]
                        .matches_history[0]
                        .head(3)(2) = new_trail_points[matching[max_col].second]
                                          .most_recent_coordinates(2);
                    // Mark as `active` to distinguish which pf has been matched
                    // in the current cycle.
                    trail_points[matching[max_row].first].is_active = true;
                    // Write zeros to row and column in similarity matrix.
                    similarity_matrix.col(max_col).setZero();
                    similarity_matrix.row(max_row).setZero();
                    // Decrement because persistent feature matches do not
                    // count.
                    --n_matched;
                    continue;
                }

                Eigen::VectorXd matched_features(
                    MatchingResult::kNumberOfMatchedCoordsAndRowColInt);
                matched_features(0) = trail_points[matching[max_row].first]
                                          .most_recent_coordinates(0);
                matched_features(1) = trail_points[matching[max_row].first]
                                          .most_recent_coordinates(1);
                matched_features(2) = trail_points[matching[max_row].first]
                                          .most_recent_coordinates(2);
                matched_features(3) = new_trail_points[matching[max_col].second]
                                          .most_recent_coordinates(0);
                matched_features(4) = new_trail_points[matching[max_col].second]
                                          .most_recent_coordinates(1);
                matched_features(5) = new_trail_points[matching[max_col].second]
                                          .most_recent_coordinates(2);
                matched_features(6) = max_row;
                matched_features(7) = max_col;
                matched_features(8) =
                    new_trail_points[matching[max_col].second].intensity;
                matches.matched_positions.conservativeResize(
                    n_matched,
                    MatchingResult::kNumberOfMatchedCoordsAndRowColInt);
                matches.matched_positions.row(n_matched - 1) = matched_features;
                similarity_matrix.col(max_col).setZero();
                similarity_matrix.row(max_row).setZero();
            }
            else
            {
                if ((similarity_matrix.row(max_row).array() > 0).count() > 1)
                {
                    // Get a feature to-be-matched.
                    Eigen::Vector3d candidate_feature_previous_pc;
                    candidate_feature_previous_pc(0) =
                        trail_points[matching[max_row].first]
                            .most_recent_coordinates(0);
                    candidate_feature_previous_pc(1) =
                        trail_points[matching[max_row].first]
                            .most_recent_coordinates(1);
                    candidate_feature_previous_pc(2) =
                        trail_points[matching[max_row].first]
                            .most_recent_coordinates(2);

                    // Compute the reference distance from already matched
                    // features in the previous pc2 and the feature
                    // to-be-matched.
                    double ref_distance = 0;
                    for (int i = 0; i < matches.matched_positions.rows(); ++i)
                    {
                        ref_distance =
                            ref_distance + (candidate_feature_previous_pc -
                                            matches.matched_positions.row(i)
                                                .head(3)
                                                .transpose())
                                               .norm();
                    }

                    // For each non-zero entry of the row coresponding to the
                    // feature to-be-matched compute distances to all candidate
                    // matches and compare it to the reference distance.
                    double min_norm_distance = 10e8;
                    for (int i = 0; i < matching.size(); ++i)
                    {
                        if (similarity_matrix(max_row, i) > 0.0)
                        {
                            Eigen::Vector3d candidate_feature_current_pc;
                            candidate_feature_current_pc(0) =
                                new_trail_points[matching[i].second]
                                    .most_recent_coordinates(0);
                            candidate_feature_current_pc(1) =
                                new_trail_points[matching[i].second]
                                    .most_recent_coordinates(1);
                            candidate_feature_current_pc(2) =
                                new_trail_points[matching[i].second]
                                    .most_recent_coordinates(2);
                            double current_min_distance = 0;
                            for (int j = 0;
                                 j < matches.matched_positions.rows(); ++j)
                            {
                                current_min_distance =
                                    current_min_distance +
                                    (candidate_feature_current_pc -
                                     matches.matched_positions.row(j)
                                         .tail(3)
                                         .transpose())
                                        .norm();
                            }

                            const double current_min_norm_distance =
                                std::fabs(ref_distance - current_min_distance);
                            if (current_min_norm_distance < min_norm_distance)
                            {
                                max_col = i;
                                min_norm_distance = current_min_norm_distance;
                            }
                        }
                    }
                }

                // Got persistent feature.
                if (trail_points[matching[max_row].first].is_persistent)
                {
                    // Save the matched measurement in
                    // `matches_history[0].head(3)`.
                    trail_points[matching[max_row].first]
                        .matches_history[0]
                        .head(3)(0) = new_trail_points[matching[max_col].second]
                                          .most_recent_coordinates(0);
                    trail_points[matching[max_row].first]
                        .matches_history[0]
                        .head(3)(1) = new_trail_points[matching[max_col].second]
                                          .most_recent_coordinates(1);
                    trail_points[matching[max_row].first]
                        .matches_history[0]
                        .head(3)(2) = new_trail_points[matching[max_col].second]
                                          .most_recent_coordinates(2);
                    // Mark as `active` to distinguish which pf has been matched
                    // in the current cycle.
                    trail_points[matching[max_row].first].is_active = true;
                    // Write zeros to row and column in similaroty matrix.
                    similarity_matrix.col(max_col).setZero();
                    similarity_matrix.row(max_row).setZero();
                    // Decrement because persistent feature matches do not
                    // count.
                    --n_matched;
                    continue;
                }

                matches.matched_positions.conservativeResize(
                    n_matched,
                    MatchingResult::kNumberOfMatchedCoordsAndRowColInt);
                Eigen::VectorXd matched_features(
                    MatchingResult::kNumberOfMatchedCoordsAndRowColInt);
                matched_features(0) = trail_points[matching[max_row].first]
                                          .most_recent_coordinates(0);
                matched_features(1) = trail_points[matching[max_row].first]
                                          .most_recent_coordinates(1);
                matched_features(2) = trail_points[matching[max_row].first]
                                          .most_recent_coordinates(2);
                matched_features(3) = new_trail_points[matching[max_col].second]
                                          .most_recent_coordinates(0);
                matched_features(4) = new_trail_points[matching[max_col].second]
                                          .most_recent_coordinates(1);
                matched_features(5) = new_trail_points[matching[max_col].second]
                                          .most_recent_coordinates(2);
                matched_features(6) = max_row;
                matched_features(7) = max_col;
                matched_features(8) =
                    new_trail_points[matching[max_col].second].intensity;
                matches.matched_positions.row(n_matched - 1) = matched_features;
                similarity_matrix.col(max_col).setZero();
                similarity_matrix.row(max_row).setZero();
            }
            matched_new_trailpoints_indices.push_back(max_col);
        }
        else
        {
            ++n_unmatched;
            matches.unmatched_positions.conservativeResize(
                n_unmatched, MatchingResult::kNumberOfUnmatchedCoordsAndRowCol);
            Eigen::VectorXd unmatched_features(
                MatchingResult::kNumberOfUnmatchedCoordsAndRowCol);
            unmatched_features(0) = new_trail_points[matching[max_col].second]
                                        .most_recent_coordinates(0);
            unmatched_features(1) = new_trail_points[matching[max_col].second]
                                        .most_recent_coordinates(1);
            unmatched_features(2) = new_trail_points[matching[max_col].second]
                                        .most_recent_coordinates(2);
            unmatched_features(3) = max_row;
            unmatched_features(4) = max_col;
            matches.unmatched_positions.row(n_unmatched - 1) =
                unmatched_features;
            similarity_matrix(max_row, max_col) = 0.0;
        }
    }
    // Get all unmatched new points.
    unmatched_new_trailpoints_indices.resize(new_trail_points.size());
    std::generate(unmatched_new_trailpoints_indices.begin(),
                  unmatched_new_trailpoints_indices.end(),
                  [n = 0]() mutable { return n++; });
    std::for_each(
        matched_new_trailpoints_indices.begin(),
        matched_new_trailpoints_indices.end(), [&](const int matched_index) {
            unmatched_new_trailpoints_indices.erase(
                std::remove_if(unmatched_new_trailpoints_indices.begin(),
                               unmatched_new_trailpoints_indices.end(),
                               [&](const int unmatched_index) {
                                   return ((unmatched_index ==
                                            matching[matched_index].second));
                               }),
                unmatched_new_trailpoints_indices.end());
        });
    return static_cast<bool>(n_matched);
}

bool PointsAssociator::calculateFeaturesFromTrail(
    const pcl::PointCloud<pcl::PointXYZI>& current_pointcloud,
    State& current_state, const Parameters& parameters, Trail& trail,
    Features& features)
{
    // Merge into the trail persistent features from the state.
    trail.appendPersistentFeaturesToTrail(current_state.persistent_features_);
    trail.setHistoryTrailPointsToInactive();

    const pcl::PointCloud<pcl::PointXYZI> filtered_current_pointcloud =

        util::applyPyramidFiltering(
            util::applyNearRangeFiltering(current_pointcloud, 0.1, 0.1));

    std::vector<TrailPoint> new_trail_points =
        util::convertPC2ToTrailPoints(filtered_current_pointcloud);
    Trail::setTrailPointsActiveStatus(new_trail_points, true);
    // Rotate and translate the previous pc2 using the transformation
    // between current and previous radar poses in the state vector.
    // Persistent features are transformed into the radar local frame
    // for matching.
    const std::vector<TrailPoint> transformed_trail_points =
        trail.rotateAndTranslateMostRecentTrailCoordinates(current_state,
                                                           parameters);

    // Calculate the cost matrix based on euclidean distances between
    // points.
    Eigen::MatrixXd distances_matrix(transformed_trail_points.size(),
                                     new_trail_points.size());
    calculateDistancesMatrices(new_trail_points, transformed_trail_points,
                               distances_matrix);
    Eigen::VectorXd intensities_vector(new_trail_points.size());
    calculateIntensitiesVector(new_trail_points, intensities_vector);
    // Calculate the linear sum assignment using munkres algorithm.
    const std::vector<std::pair<unsigned, unsigned>> matching =
        calculateMatching(distances_matrix);
    // Calculate the similarity matrix according to milli-RIO paper.
    Eigen::MatrixXd similarity_matrix = calculateSimilarityMatrix(
        distances_matrix, intensities_vector, matching);

    // calculate matrix with correspondences [xp1 yp1 zp1 xp2 yp2 zp2]
    //                                       [xp1 yp1 zp1 xp2 yp2 zp2]
    //                                                  ...
    // In our setting p1 is previous pointcloud and p2 is the current one.
    std::vector<TrailPoint>& trail_points = trail.getTrailPoints();

    // In the call below, matching information is written into persistent
    // features appended into the trail because we pass `trail_points` as
    // non-const reference. In the `trail_points` persistent features are in the
    // global frame (are not transformed).
    MatchingResult matches;
    std::vector<int> unmatched_new_trailpoints_indices;
    const bool matches_found = searchForMatchingFeatures(
        new_trail_points, trail_points, matching, matches, similarity_matrix,
        unmatched_new_trailpoints_indices);

    // Write into the state the new persistent features and remove old ones.
    trail.updatePersistentFeaturesMatches(current_state, parameters);

    // Update the non-persistent features (msckf features).
    trail.update(matches, new_trail_points, matching,
                 unmatched_new_trailpoints_indices);

    // Remove trailpoints marked as inactive and persistent features.
    trail.removeInactiveAndActivePersistent();

    // Produce `Features` objects from trail and matches.
    // Note that coordinates in the matches history are not transformed to
    // the common frame.
    trail.getFeaturesFromHistory(features);

    return matches_found;
}

}  // namespace aaucns_rio
