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

#ifndef _TRAIL_H_
#define _TRAIL_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Dense>
#include <algorithm>
// experimental
#include <functional>
#include <vector>

#include "aaucns_rio/config.h"
#include "aaucns_rio/debug.h"
#include "aaucns_rio/features.h"
#include "aaucns_rio/id_manager.h"
#include "aaucns_rio/parameters.h"
#include "aaucns_rio/state.h"
#include "aaucns_rio/trailpoint.h"
#include "aaucns_rio/util.h"

namespace aaucns_rio
{
class Trail
{
   public:
    Trail() : initialized_(false) { trail_.clear(); }

    bool isInitialized() const { return initialized_; }

    void removeInactiveAndActivePersistent()
    {
        trail_.erase(std::remove_if(trail_.begin(), trail_.end(),
                                    [](const auto& trail_point) {
                                        return !trail_point.is_active ||
                                               (trail_point.is_active &&
                                                trail_point.is_persistent);
                                    }),
                     trail_.end());
    }

    Eigen::MatrixXd convertTrailToEigenMatrix() const
    {
        Eigen::MatrixXd trailpoints_as_matrix =
            Eigen::MatrixXd::Zero(trail_.size(), 3);
        for (int i = 0; i < trailpoints_as_matrix.rows(); ++i)
        {
            trailpoints_as_matrix.row(i) = trail_[i].most_recent_coordinates;
        }
        return trailpoints_as_matrix;
    }

    const std::vector<TrailPoint>& getTrailPoints() const { return trail_; }
    std::vector<TrailPoint>& getTrailPoints() { return trail_; }

    static void setTrailPointsActiveStatus(
        std::vector<TrailPoint>& trail_points, const bool status)
    {
        std::for_each(trail_points.begin(), trail_points.end(),
                      [status](TrailPoint& trail_point) {
                          trail_point.is_active = status;
                      });
    }

    void setHistoryTrailPointsToInactive()
    {
        setTrailPointsActiveStatus(trail_, false);
    }

    void appendPersistentFeaturesToTrail(
        const std::vector<TrailPoint>& persistent_features)
    {
        // Appended pfs are in the global frame.
        // TODO(jan): `if()` could be removed since by construction
        // all trailpoints in `persistent_features_` are persisent.
        std::copy_if(persistent_features.begin(), persistent_features.end(),
                     std::back_inserter(trail_),
                     [](const auto& item) { return item.is_persistent; });
    }

    void updatePersistentFeaturesMatches(State& state,
                                         const Parameters& parameters)
    {
        // TODO(jan): implement using `copy_if()` as above.
        state.persistent_features_.clear();
        for (int i = 0; i < trail_.size(); ++i)
        {
            if (trail_[i].is_persistent)
            {
                state.persistent_features_.push_back(trail_[i]);
            }
        }
        // Pf becomes `active` only on a match so we do a pass
        // over all pfs and those which were not matched (are not `active`)
        // are removed together with their covariance blocks.
        std::vector<std::size_t> removed_pfs_ids =
            state.removePersistentFeaturesAndUpdateCovariance();
        id_manager_.releaseIds(removed_pfs_ids);
    }

    void update(const MatchingResult& matches,
                const std::vector<TrailPoint>& new_trail_points,
                const std::vector<std::pair<unsigned, unsigned>>& matching,
                std::vector<int>& unmatched_new_trailpoints_indices)
    {
        // Mark all points in the trail which have been matched to active.
        for (int i = 0; i < matches.matched_positions.rows(); ++i)
        {
            trail_[matching[static_cast<Eigen::MatrixXd::Index>(
                                matches.matched_positions.row(i)(6))]
                       .first]
                .matches_history.push_front(
                    matches.matched_positions.row(i).head(6));
            trail_[matching[static_cast<Eigen::MatrixXd::Index>(
                                matches.matched_positions.row(i)(6))]
                       .first]
                .is_active = true;
            // Features in the trail are never persistent.
            trail_[matching[static_cast<Eigen::MatrixXd::Index>(
                                matches.matched_positions.row(i)(6))]
                       .first]
                .is_persistent = false;
            trail_[matching[static_cast<Eigen::MatrixXd::Index>(
                                matches.matched_positions.row(i)(6))]
                       .first]
                .intensity = matches.matched_positions.row(i)(8);
            trail_[matching[static_cast<Eigen::MatrixXd::Index>(
                                matches.matched_positions.row(i)(6))]
                       .first]
                .most_recent_coordinates =
                matches.matched_positions.row(i).segment(3, 3);
        }
        // Accept all unmatched new points.
        std::for_each(unmatched_new_trailpoints_indices.begin(),
                      unmatched_new_trailpoints_indices.end(),
                      [&, this](const int index) {
                          trail_.push_back(new_trail_points[index]);
                      });
    }

    void getFeaturesFromHistory(Features& features) const
    {
        // Build `Features` object from the history - use only history - not
        // most recent coordinates. These are used only for matching and are the
        // same as the last entry in the history.
        // In `Features` object the youngest matches are held at the top of the
        // matrix and the oldest at the bottom. This is because the
        // `matches_history` field gets new matches with `push_front()` method.
        features.reset();
        for (int i = 0; i < trail_.size(); ++i)
        {
            for (int j = 0; j < features.positions_and_matched_or_not_.size();
                 ++j)
            {
                if (trail_[i].matches_history.size() > j)
                {
                    features.positions_and_matched_or_not_[j].true_or_false =
                        true;
                    int n_matches = features.positions_and_matched_or_not_[j]
                                        .positions.rows();
                    ++n_matches;
                    features.positions_and_matched_or_not_[j]
                        .positions.conservativeResize(n_matches, 6);
                    features.positions_and_matched_or_not_[j]
                        .positions.row(n_matches - 1)
                        .head(3) = trail_[i].matches_history.at(j).head(3);
                    features.positions_and_matched_or_not_[j]
                        .positions.row(n_matches - 1)
                        .tail(3) = trail_[i].matches_history.at(0).tail(3);
                }
            }
        }
    }

    void findAndAddPersistentFeaturesToState(State& current_updated_state,
                                             const Parameters& parameters)
    {
        // Called after update when all inactive (unmatched) trailpoints have
        // been removed.
        for (int i = 0; i < trail_.size(); ++i)
        {
            if ((trail_[i].matches_history.size() >=
                 Config::kNSamplesForFeaturePersistence) &&
                !trail_[i].is_persistent)
            {
                trail_[i].is_persistent = true;
                trail_[i].id = id_manager_.getFreeId();
                // Initialize the persistent feature in the global frame.
                trail_[i].most_recent_coordinates =
                    current_updated_state.q_.toRotationMatrix() *
                        (current_updated_state.q_ri_.toRotationMatrix() *
                             trail_[i]
                                 .most_recent_coordinates.transpose()
                                 .eval() +
                         current_updated_state.p_ri_) +
                    current_updated_state.p_;
                current_updated_state.acceptPersistentFeature(trail_[i],
                                                              parameters);
                trail_.erase(trail_.begin() + i);
                --i;
            }
        }
    }

    void initialize(const pcl::PointCloud<pcl::PointXYZI>& pc2)
    {
        // Initialization routine called only once from the radar callback in
        // order to to initialize the trail.
        const pcl::PointCloud<pcl::PointXYZI> filtered_pc2 =
            util::applyNearRangeFiltering(pc2, 0.2, 0.15);
        for (int i = 0; i < filtered_pc2.width; ++i)
        {
            TrailPoint trail_point;
            trail_point.most_recent_coordinates
                << filtered_pc2.points[i].data[0],
                filtered_pc2.points[i].data[1], filtered_pc2.points[i].data[2];
            trail_point.is_active = true;
            trail_point.is_persistent = false;
            trail_point.intensity = filtered_pc2.points[i].intensity;
            trail_.push_back(trail_point);
        }
        initialized_ = true;
    }

    std::vector<TrailPoint> rotateAndTranslateMostRecentTrailCoordinates(
        State& current_state, const Parameters& parameters) const
    {
        // Make sure to call this function only after removing the inactive
        // elements.
        std::vector<TrailPoint> transformed_trail;
        for (int i = 0; i < trail_.size(); ++i)
        {
            TrailPoint::CoordType transformed_vec;
            if (trail_[i].is_persistent)
            {
                // Transform from global to radar frame since PFs are stored in
                // global frame.
                transformed_vec =
                    current_state.q_ri_.conjugate().toRotationMatrix() *
                    (current_state.q_.conjugate().toRotationMatrix() *
                         (trail_[i].most_recent_coordinates.transpose().eval() -
                          current_state.p_) -
                     current_state.p_ri_);
            }
            else
            {
                transformed_vec = rotateAndTranslateSingleVectorAtIndex(
                    current_state, parameters, 0,
                    trail_[i].most_recent_coordinates);
            }

            TrailPoint trail_point;
            // Check here the value when it's transposed and not.
            trail_point.most_recent_coordinates = transformed_vec.transpose();
            trail_point.is_active = trail_[i].is_active;
            trail_point.is_persistent = trail_[i].is_persistent;
            trail_point.id = trail_[i].id;
            trail_point.matches_history = trail_[i].matches_history;
            trail_point.intensity = trail_[i].intensity;
            transformed_trail.push_back(trail_point);
        }
        return transformed_trail;
    }

    static TrailPoint::CoordType rotateAndTranslateSingleVectorAtIndex(
        const State& current_state, const Parameters& parameters,
        const int index, const TrailPoint::CoordType& vec_to_transform)
    {
        TrailPoint::CoordType transformed_vec = vec_to_transform;
        transformed_vec =
            current_state.q_ri_.conjugate().toRotationMatrix() *
            (current_state.q_.conjugate().toRotationMatrix() *
                 (current_state.past_orientations_.at(index)
                          .toRotationMatrix() *
                      (current_state.q_ri_.toRotationMatrix() *
                           transformed_vec.transpose().eval() +
                       current_state.p_ri_) +
                  current_state.past_positions_.at(index) - current_state.p_) -
             current_state.p_ri_);
        return transformed_vec;
    }

    std::vector<TrailPoint>::size_type size() const { return trail_.size(); };

    void resetIdManager() { id_manager_.reset(); };
    std::vector<std::size_t> getOccupiedIds()
    {
        return id_manager_.getCurrentAndPastActiveIds();
    }

    bool hasIdBeenInitialized(const std::size_t index)
    {
        return id_manager_.hasIdBeenInitialized(index);
    }

    void initializeId(const std::size_t index)
    {
        id_manager_.initializeId(index);
    }

    void resetIdsExceptCurrentlyActive()
    {
        id_manager_.resetIdsExceptCurrentlyActive();
    }

    void resetId(const std::size_t index) { id_manager_.resetId(index); }

    std::vector<std::size_t> getAllInitializedIds()
    {
        return id_manager_.getAllInitializedIds();
    }

    const TrailPoint& operator[](const std::size_t index) const
    {
        return trail_[index];
    }

    TrailPoint& operator[](const std::size_t index) { return trail_[index]; }

   private:
    std::vector<TrailPoint> trail_;
    bool initialized_ = false;
    IdManager id_manager_;
};

inline std::ostream& operator<<(std::ostream& os, const Trail& trail)
{
    os << "trail start" << std::endl;
    for (const auto& trailpoint : trail.getTrailPoints())
    {
        os << trailpoint << std::endl;
    }
    os << "trail end" << std::endl;
    return os;
}

}  // namespace aaucns_rio

#endif /* _TRAIL_H_ */
